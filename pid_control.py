import time
import datetime
import numpy as np
import socket
import threading
import joblib


from ball_balance_table_controller_v2 import BallBalanceTableControllerv2

controller = BallBalanceTableControllerv2()
controller.start()

K_P = (0.25, 0.25)
K_I = (0.01, 0.01)  # Introduce small integral gain for now
K_D = (25, 25)

e_filter = 15
d_filter = 75

desired_position = [122.5, 100.5]  # use list, so it can be updated in thread

current_servo_positions = (0, 0)  # Servo position (degrees)
b_pos = (0, 0)  # Ball position in mm
p_error = (0.0, 0.0)
current_error = (0.0, 0.0)
last_error = (0.0, 0.0)
delta_error = (0.0, 0.0)
last_time = datetime.datetime.now()
current_time = datetime.datetime.now()
dt = 0.0
p = (0.0, 0.0)
i = (0.0, 0.0)  # Integral term added
d = (0.0, 0.0)
pid = (0.0, 0.0)
e_terms = [(0, 0)] * e_filter
d_terms = [(0, 0)] * d_filter
sum_e = (0.0, 0.0)
sum_d = (0.0, 0.0)
sum_error = (0.0, 0.0)  # To accumulate error over time for the integral term
e_counter = 0
d_counter = 0

model = joblib.load("mlp_model1.pkl")
scaler = joblib.load("minmax_scaler.pkl")

MAX_WIDTH_MM = 355
MAX_HEIGHT_MM = 285

maze_walls_mm = [
    (-177.5, 142.5, -177.5, -142.5),  # left
    (-177.5, -142.5, 177.5, -142.5),  # bottom
    (177.5, -142.5, 177.5, 142.5),    # right
    (177.5, 142.5, -177.5, 142.5),    # top
    (17.5, -142.5, 17.5, -62.5),
    (12.5, -57.5, 102.5, -57.5),
    (177.5, 0.0, -62.5, 0.0),
    (-63.0, -85.0, -63.0, 85.0),
    (-58.0, 80.0, -93.0, 80.0),
    (-58.0, -90.0, -93.0, -90.0),
    (42.5, 0.5, 42.5, 30.5),
    (22.5, 142.5, 22.5, 82.5),
    (17.5, 87.5, 107.5, 87.5),
    (102.5, 92.5, 102.5, 47.5),
]

ball_pos_sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
gui_ip = "192.168.0.115"  # Replace with the IP of the machine running the GUI
gui_port = 6007         # This must match the port the GUI listens on

# GUI update lock
position_lock = threading.Lock()

def listen_for_hand_pose():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("192.168.0.139", 5006))

    step_mm = 1.0

    while True:
        try:
            data, _ = sock.recvfrom(2048)
            floats = np.frombuffer(data, dtype=np.float32)

            if floats.shape[0] != 40:
                continue

            left = floats[:20].reshape(1, -1)
            right = floats[20:].reshape(1, -1)

            left_scaled = scaler.transform(left)
            right_scaled = scaler.transform(right)

            left_probs = model.predict_proba(left_scaled)[0]
            right_probs = model.predict_proba(right_scaled)[0]

            x_class = np.argmax(left_probs) + 1 if np.max(left_probs) >= 0.93 else 0
            y_class = np.argmax(right_probs) + 1 if np.max(right_probs) >= 0.93 else 0

            with position_lock:
                new_x = desired_position[0]
                new_y = desired_position[1]

                if x_class == 1:
                    new_x -= step_mm
                elif x_class == 2:
                    new_x += step_mm

                if y_class == 1:
                    new_y += step_mm  # flipped because Y increases upward in center-origin
                elif y_class == 2:
                    new_y -= step_mm

                # Clamp to physical bounds
                new_x = max(-MAX_WIDTH_MM/2 + ball_radius_mm, min(MAX_WIDTH_MM/2 - ball_radius_mm, new_x))
                new_y = max(-MAX_HEIGHT_MM/2 + ball_radius_mm, min(MAX_HEIGHT_MM/2 - ball_radius_mm, new_y))

                # Collision-safe update
                if not hits_wall_mm(new_x, desired_position[1]):
                    desired_position[0] = new_x
                if not hits_wall_mm(desired_position[0], new_y):
                    desired_position[1] = new_y

        except Exception as e:
            print(f"Error receiving hand pose: {e}")

def point_to_segment_dist(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    if dx == dy == 0:
        return ((px - x1)**2 + (py - y1)**2)**0.5

    t = max(0, min(1, ((px - x1)*dx + (py - y1)*dy) / (dx**2 + dy**2)))
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    return ((px - proj_x)**2 + (py - proj_y)**2)**0.5

ball_radius_mm = 20  

def hits_wall_mm(x_mm, y_mm):
    for x1, y1, x2, y2 in maze_walls_mm:
        if point_to_segment_dist(x_mm, y_mm, x1, y1, x2, y2) <= ball_radius_mm:
            return True
    return False

threading.Thread(target=listen_for_hand_pose, daemon=True).start()

while True:
    current_time = datetime.datetime.now()
    time_delta = current_time - last_time
    dt = time_delta.total_seconds()  # Ensure dt is in seconds for better timing accuracy
    last_time = current_time

    b_pos = controller.get_ball_position_in_mm()
    pos_bytes = np.array(b_pos, dtype=np.float32).tobytes()
    ball_pos_sender.sendto(pos_bytes, (gui_ip, gui_port))

    last_error = current_error

    with position_lock:
        target = tuple(desired_position)  # copy safely

    current_error = (target[0] - b_pos[0], target[1] - b_pos[1])

    # Proportional term (P)
    p_error = current_error
    e_terms[e_counter] = p_error

    sum_e = (0, 0)

    for j in range(e_filter):
        sum_e = (sum_e[0] + e_terms[j][0], sum_e[1] + e_terms[j][1])

    p_error = (sum_e[0] / e_filter, sum_e[1] / e_filter)
    e_counter += 1

    if e_counter >= e_filter:
        e_counter = 0

    # Accumulate error over time for the integral term (I)
    sum_error = (sum_error[0] + current_error[0] * dt, sum_error[1] + current_error[1] * dt)

    # Derivative term (D)
    delta_error = (current_error[0] - last_error[0], current_error[1] - last_error[1])

    d_terms[d_counter] = delta_error

    sum_d = (0, 0)

    for j in range(d_filter):
        sum_d = (sum_d[0] + d_terms[j][0], sum_d[1] + d_terms[j][1])

    delta_error = (sum_d[0] / d_filter, sum_d[1] / d_filter)
    d_counter += 1

    if d_counter >= d_filter:
        d_counter = 0

    # PID control
    p = (p_error[0] * K_P[0], p_error[1] * K_P[1])
    i = (sum_error[0] * K_I[0], sum_error[1] * K_I[1])  # Integral term based on accumulated error
    d = (delta_error[0] * K_D[0], delta_error[1] * K_D[1])

    pid = (p[0] + i[0] + d[0], p[1] + i[1] + d[1])

    # PID -60-60
    current_servo_positions = (-(pid[0] - 5), -(pid[1] - 5))
    controller._servo_controller.set_degrees_bbt(current_servo_positions)
    
    time.sleep(0.004)  # Small delay for control loop timing
