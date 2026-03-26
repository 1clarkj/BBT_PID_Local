import time
import datetime
import numpy as np
import socket
import threading
import warnings
import sys
import json
warnings.filterwarnings("ignore")


from pid_local.planning.occupancy_planner import VisibilityPlanner
from pid_local.hardware.table_controller import BallBalanceTableControllerv2
 
controller = BallBalanceTableControllerv2()
controller.start()

K_P = (0.25, 0.25)
K_I = (0.01, 0.01)  # Introduce small integral gain for now
K_D = (25, 25)

e_filter = 7
d_filter = 30

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

MAX_WIDTH_MM = 355
MAX_HEIGHT_MM = 285
ball_radius_mm = 20
wall_safety_margin_mm = 4.0
planner_ball_radius_mm = ball_radius_mm
planner_wall_safety_margin_mm = 0.0
planner_strict_edge_clearance_mm = 0.5

GOAL_REPLAN_THRESHOLD_MM = 10.0
WAYPOINT_REACHED_MM = 8.0
STUCK_REPLAN_TIMEOUT_S = 0.6
PROGRESS_EPS_MM = 0.5
REPLAN_MIN_INTERVAL_S = 0.08
WAYPOINT_STUCK_REPLAN_MIN_DIST_MM = 16.0
NO_MOTION_EPS_MM = 0.6
NO_MOTION_DESIRED_SEP_MM = 3.0
NO_MOTION_REPLAN_TIMEOUT_S = 0.25

maze_walls_mm = [
    (-177.5, 142.5, -177.5, -142.5),  # left
    (-177.5, -142.5, 177.5, -142.5),  # bottom
    (177.5, -142.5, 177.5, 142.5),    # right
    (177.5, 142.5, -177.5, 142.5),    # top
    (17.5, -142.5, 17.5, -62.5),
    (12.5, -67.5, 102.5, -67.5),
    (177.5, 0.0, -62.5, 0.0),
    (-63.0, -85.0, -63.0, 85.0),
    (-58.0, 80.0, -93.0, 80.0),
    (-58.0, -90.0, -93.0, -90.0),
    (42.5, 0.5, 42.5, 30.5),
    (22.5, 142.5, 22.5, 82.5),
    (17.5, 87.5, 107.5, 87.5),
    (102.5, 92.5, 102.5, 47.5),
]

CENTER_LINES_MM = [
    (-132.5, -122.5, -132.5, 122.5),
    (-132.5, 122.5, -22.5, 122.5),
    (57.5, 122.5, 150.5, 122.5),
    (-87.5, 82.5, -22.5, 82.5),
    (-132.5, 42.5, 150.5, 42.5),
    (-132.5, -42.5, 7.5, -42.5),
    (107.5, -42.5, 150.5, -42.5),
    (-132.5, -122.5, -22.5, -122.5),
    (57.5, -122.5, 150.5, -122.5),
    (-22.5, 42.5, -22.5, 122.5),
    (-22.5, -122.5, -22.5, -42.5),
    (150.5, 42.5, 150.5, 122.5),
    (150.5, -122.5, 150.5, -42.5),
    (-22.5, -42.5, 22.5, -28.5),
    (22.5, -28.5, 97.5, -28.5),
    (97.5, -28.5, 150.5, -42.5),
]

ball_pos_sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
gui_ip = "0.0.0.0"  # Will be determined through the handshake
gui_port = 6007         # This must match the port the GUI listens on to send position data back
hand_pose_port = 5006
hand_pose_bind_ip = "0.0.0.0"

HANDSHAKE_PORT = 8008
DEFAULT_GUI_POSITION_PORT = 6007
TARGET_TIMEOUT_S = 0.5
TARGET_HOLD_TIMEOUT_S = 2.0
BALL_TRACK_HOLD_S = 0.15
INTEGRAL_CLAMP_MM_S = 600.0
MAX_SERVO_DEG = 35.0

# GUI update lock
position_lock = threading.Lock()
endpoint_lock = threading.Lock()
target_lock = threading.Lock()
last_target_seq = -1
last_target_rx_time = time.monotonic()
latest_target_goal = np.array(desired_position, dtype=float)
has_received_target = False

planner_lock = threading.Lock()
VisibilityPlanner.STRICT_EDGE_CLEARANCE_MM = planner_strict_edge_clearance_mm
planner = VisibilityPlanner(
    walls_mm=maze_walls_mm,
    inflation_mm=(planner_ball_radius_mm + planner_wall_safety_margin_mm),
    helper_edge_length_mm=32.0,
    center_lines_mm=CENTER_LINES_MM,
)
planner_path = []
planner_waypoint_idx = 0
planner_goal = np.array(desired_position, dtype=float)
planner_replan_requested = True
last_replan_time = 0.0
last_progress_time = time.monotonic()
last_waypoint_distance = None
escape_mode_active = False
escape_target = None
last_valid_ball_pos = None
last_ball_seen_time = 0.0
last_motion_pos = None
last_motion_time = time.monotonic()

def set_gui_endpoint(ip: str, port: int, source: str):
    global gui_ip, gui_port
    with endpoint_lock:
        gui_ip = ip
        gui_port = int(port)
    print(f"GUI endpoint updated via {source}: {gui_ip}:{gui_port}")

def handshake_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", HANDSHAKE_PORT))
    print(f"Listening for handshake on 0.0.0.0:{HANDSHAKE_PORT}")

    while True:
        try:
            data, addr = sock.recvfrom(2048)
            msg = json.loads(data.decode("utf-8"))
        except Exception as e:
            print(f"Handshake parse error: {e}")
            continue

        if not isinstance(msg, dict) or msg.get("type") != "discover":
            continue

        reply_port = int(msg.get("reply_port", 0))
        requested_position_port = int(msg.get("position_port", DEFAULT_GUI_POSITION_PORT))
        laptop_ip = addr[0]
        set_gui_endpoint(laptop_ip, requested_position_port, "handshake")

        ack = {
            "type": "ack",
            "pi_ip": socket.gethostbyname(socket.gethostname()),
            "control_port": hand_pose_port,
            "position_port": requested_position_port,
        }

        if reply_port > 0:
            try:
                sock.sendto(json.dumps(ack).encode("utf-8"), (laptop_ip, reply_port))
                print(f"Handshake ACK sent to {laptop_ip}:{reply_port}; stream target {gui_ip}:{gui_port}")
            except Exception as e:
                print(f"Handshake ACK send error: {e}")

def apply_desired_target(x_mm: float, y_mm: float):
    with position_lock:
        new_x = max(-MAX_WIDTH_MM / 2 + ball_radius_mm, min(MAX_WIDTH_MM / 2 - ball_radius_mm, x_mm))
        new_y = max(-MAX_HEIGHT_MM / 2 + ball_radius_mm, min(MAX_HEIGHT_MM / 2 - ball_radius_mm, y_mm))
        prev_target = (desired_position[0], desired_position[1])
        if not hits_wall_mm(new_x, desired_position[1]):
            desired_position[0] = new_x
        if not hits_wall_mm(desired_position[0], new_y):
            desired_position[1] = new_y

        # If axis-wise projection is still blocked, nudge to nearest free planner cell
        # around the requested target so the control target remains actionable.
        if (
            abs(desired_position[0] - prev_target[0]) < 1e-6
            and abs(desired_position[1] - prev_target[1]) < 1e-6
            and (abs(new_x - prev_target[0]) > 1e-6 or abs(new_y - prev_target[1]) > 1e-6)
        ):
            free_world = planner.nearest_free_world((new_x, new_y), max_radius=16)
            if free_world is not None and not hits_wall_mm(float(free_world[0]), float(free_world[1])):
                desired_position[0] = float(free_world[0])
                desired_position[1] = float(free_world[1])

def set_target_goal_from_message(x_mm: float, y_mm: float):
    global latest_target_goal, planner_replan_requested
    candidate = np.array([x_mm, y_mm], dtype=float)
    candidate[0] = max(-MAX_WIDTH_MM / 2 + ball_radius_mm, min(MAX_WIDTH_MM / 2 - ball_radius_mm, candidate[0]))
    candidate[1] = max(-MAX_HEIGHT_MM / 2 + ball_radius_mm, min(MAX_HEIGHT_MM / 2 - ball_radius_mm, candidate[1]))
    with planner_lock:
        prev_goal = latest_target_goal.copy()
        latest_target_goal = candidate
        if np.linalg.norm(candidate - prev_goal) >= GOAL_REPLAN_THRESHOLD_MM:
            planner_replan_requested = True

def update_planner_setpoint(current_pos_xy):
    global planner_path, planner_waypoint_idx, planner_goal, planner_replan_requested
    global last_replan_time, last_progress_time, last_waypoint_distance
    global escape_mode_active, escape_target

    now = time.monotonic()
    current_pos = np.array(current_pos_xy, dtype=float)

    # If estimate lands in a blocked cell, drive to nearest free space first.
    if hits_wall_mm(float(current_pos[0]), float(current_pos[1])):
        nearest_free = planner.nearest_free_world(current_pos, max_radius=80)
        if nearest_free is not None:
            with planner_lock:
                escape_mode_active = True
                escape_target = np.array(nearest_free, dtype=float)
            apply_desired_target(float(escape_target[0]), float(escape_target[1]))
            return

    if escape_mode_active:
        with planner_lock:
            escape_mode_active = False
            escape_target = None
            planner_replan_requested = True

    with planner_lock:
        goal = latest_target_goal.copy()
        path_snapshot = list(planner_path)
        waypoint_idx_snapshot = planner_waypoint_idx
        need_goal_replan = np.linalg.norm(goal - planner_goal) >= GOAL_REPLAN_THRESHOLD_MM
        need_replan = planner_replan_requested or need_goal_replan or not path_snapshot

    if need_replan and (now - last_replan_time) >= REPLAN_MIN_INTERVAL_S:
        new_path = planner.plan(current_pos, goal)
        with planner_lock:
            planner_path = [tuple(p) for p in new_path]
            planner_waypoint_idx = 1 if len(planner_path) > 1 else 0
            planner_goal = goal
            planner_replan_requested = False
            last_replan_time = now
            last_progress_time = now
            last_waypoint_distance = None
            path_snapshot = list(planner_path)
            waypoint_idx_snapshot = planner_waypoint_idx

    if not path_snapshot:
        apply_desired_target(float(goal[0]), float(goal[1]))
        return

    if waypoint_idx_snapshot >= len(path_snapshot):
        waypoint_idx_snapshot = len(path_snapshot) - 1

    waypoint = np.array(path_snapshot[waypoint_idx_snapshot], dtype=float)
    distance_to_waypoint = float(np.linalg.norm(current_pos - waypoint))

    if distance_to_waypoint <= WAYPOINT_REACHED_MM and waypoint_idx_snapshot < len(path_snapshot) - 1:
        with planner_lock:
            planner_waypoint_idx += 1
            waypoint_idx_snapshot = planner_waypoint_idx
            waypoint = np.array(planner_path[waypoint_idx_snapshot], dtype=float)
            distance_to_waypoint = float(np.linalg.norm(current_pos - waypoint))
            last_progress_time = now
            last_waypoint_distance = distance_to_waypoint

    with planner_lock:
        if last_waypoint_distance is None or distance_to_waypoint < (last_waypoint_distance - PROGRESS_EPS_MM):
            last_waypoint_distance = distance_to_waypoint
            last_progress_time = now
        elif (now - last_progress_time) > STUCK_REPLAN_TIMEOUT_S and distance_to_waypoint > WAYPOINT_REACHED_MM:
            planner_replan_requested = True

    apply_desired_target(float(waypoint[0]), float(waypoint[1]))

def listen_for_control():
    global last_target_seq, last_target_rx_time, has_received_target
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((hand_pose_bind_ip, hand_pose_port))
    print(f"Listening for control on {hand_pose_bind_ip}:{hand_pose_port}")

    while True:
        try:
            data, addr = sock.recvfrom(2048)
            try:
                msg = json.loads(data.decode("utf-8"))
            except Exception:
                continue

            if not isinstance(msg, dict):
                continue

            msg_type = msg.get("type")
            if msg_type == "position_subscribe":
                requested_port = int(msg.get("position_port", DEFAULT_GUI_POSITION_PORT))
                set_gui_endpoint(addr[0], requested_port, "position_subscribe")
                continue

            if msg_type == "target_position":
                x_mm = msg.get("x_mm")
                y_mm = msg.get("y_mm")
                seq = int(msg.get("seq", -1))
                if x_mm is None or y_mm is None:
                    continue
                try:
                    x_mm = float(x_mm)
                    y_mm = float(y_mm)
                except (TypeError, ValueError):
                    continue

                now = time.monotonic()
                with target_lock:
                    # If sender restarts, sequence may reset to 0; allow reset
                    # after target stream has been stale for a short interval.
                    if seq <= last_target_seq and (now - last_target_rx_time) <= TARGET_TIMEOUT_S:
                        continue
                    last_target_seq = seq
                    last_target_rx_time = now
                    has_received_target = True
                set_target_goal_from_message(x_mm, y_mm)

        except Exception as e:
            print(f"Error receiving control: {e}")

def point_to_segment_dist(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    if dx == dy == 0:
        return ((px - x1)**2 + (py - y1)**2)**0.5

    t = max(0, min(1, ((px - x1)*dx + (py - y1)*dy) / (dx**2 + dy**2)))
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    return ((px - proj_x)**2 + (py - proj_y)**2)**0.5

def hits_wall_mm(x_mm, y_mm):
    for x1, y1, x2, y2 in maze_walls_mm:
        if point_to_segment_dist(x_mm, y_mm, x1, y1, x2, y2) <= ball_radius_mm:
            return True
    return False


def reset_pid_state():
    global p_error, current_error, last_error, delta_error, p, i, d, pid
    global e_terms, d_terms, sum_error, e_counter, d_counter

    p_error = (0.0, 0.0)
    current_error = (0.0, 0.0)
    last_error = (0.0, 0.0)
    delta_error = (0.0, 0.0)
    p = (0.0, 0.0)
    i = (0.0, 0.0)
    d = (0.0, 0.0)
    pid = (0.0, 0.0)
    e_terms = [(0.0, 0.0)] * e_filter
    d_terms = [(0.0, 0.0)] * d_filter
    sum_error = (0.0, 0.0)
    e_counter = 0
    d_counter = 0

threading.Thread(target=handshake_listener, daemon=True).start()
threading.Thread(target=listen_for_control, daemon=True).start()

try:
    while True:
        current_time = datetime.datetime.now()
        time_delta = current_time - last_time
        dt = time_delta.total_seconds()
        last_time = current_time
        dt = max(dt, 1e-4)

        raw_b_pos = controller.get_ball_position_in_mm()
        now_mono = time.monotonic()
        ball_contact = bool(getattr(controller, "ball_contact", False))
        ball_visible = ball_contact or (last_valid_ball_pos is not None and (now_mono - last_ball_seen_time) <= BALL_TRACK_HOLD_S)

        if ball_contact:
            b_pos = raw_b_pos
            last_valid_ball_pos = b_pos
            last_ball_seen_time = now_mono
        elif last_valid_ball_pos is not None:
            b_pos = last_valid_ball_pos
        else:
            b_pos = raw_b_pos

        pos_bytes = np.array(b_pos, dtype=np.float32).tobytes()
        with endpoint_lock:
            target_ip = gui_ip
            target_port = gui_port
        if target_ip not in ("0.0.0.0", "", None):
            ball_pos_sender.sendto(pos_bytes, (target_ip, target_port))

        with target_lock:
            target_age = now_mono - last_target_rx_time
            stale_target = target_age > TARGET_TIMEOUT_S
            hold_timeout = target_age > TARGET_HOLD_TIMEOUT_S
            stream_seen = has_received_target
        if stream_seen and stale_target and hold_timeout and ball_visible:
            # Only freeze target to current ball after a real target stream has existed.
            set_target_goal_from_message(b_pos[0], b_pos[1])

        if not ball_visible:
            with planner_lock:
                planner_replan_requested = True
                escape_mode_active = False
                escape_target = None
            reset_pid_state()
            controller._servo_controller.set_degrees_bbt((0.0, 0.0))
            print("Ball not detected; holding neutral platform.")
            time.sleep(0.01)
            continue

        if last_motion_pos is None:
            last_motion_pos = np.array(b_pos, dtype=float)
            last_motion_time = now_mono
        else:
            motion = float(np.linalg.norm(np.array(b_pos, dtype=float) - last_motion_pos))
            if motion > NO_MOTION_EPS_MM:
                last_motion_pos = np.array(b_pos, dtype=float)
                last_motion_time = now_mono
            else:
                with position_lock:
                    desired_snapshot = np.array(desired_position, dtype=float)
                desired_sep = float(np.linalg.norm(desired_snapshot - np.array(b_pos, dtype=float)))
                if desired_sep > NO_MOTION_DESIRED_SEP_MM and (now_mono - last_motion_time) > NO_MOTION_REPLAN_TIMEOUT_S:
                    with planner_lock:
                        planner_replan_requested = True
                    last_motion_time = now_mono

        update_planner_setpoint(b_pos)

        last_error = current_error

        with position_lock:
            target = tuple(desired_position)

        current_error = (target[0] - b_pos[0], target[1] - b_pos[1])

        # P
        p_error = current_error
        e_terms[e_counter] = p_error
        sum_e = (0, 0)
        for j in range(e_filter):
            sum_e = (sum_e[0] + e_terms[j][0], sum_e[1] + e_terms[j][1])
        p_error = (sum_e[0] / e_filter, sum_e[1] / e_filter)
        e_counter = (e_counter + 1) % e_filter

        # I
        sum_error = (sum_error[0] + current_error[0] * dt,
                     sum_error[1] + current_error[1] * dt)
        sum_error = (
            max(-INTEGRAL_CLAMP_MM_S, min(INTEGRAL_CLAMP_MM_S, sum_error[0])),
            max(-INTEGRAL_CLAMP_MM_S, min(INTEGRAL_CLAMP_MM_S, sum_error[1])),
        )

        # D
        delta_error = (current_error[0] - last_error[0], current_error[1] - last_error[1])
        d_terms[d_counter] = delta_error
        sum_d = (0, 0)
        for j in range(d_filter):
            sum_d = (sum_d[0] + d_terms[j][0], sum_d[1] + d_terms[j][1])
        delta_error = (sum_d[0] / d_filter, sum_d[1] / d_filter)
        d_counter = (d_counter + 1) % d_filter

        # PID
        p = (p_error[0] * K_P[0], p_error[1] * K_P[1])
        i = (sum_error[0] * K_I[0], sum_error[1] * K_I[1])
        d = (delta_error[0] * K_D[0], delta_error[1] * K_D[1])
        pid = (p[0] + i[0] + d[0], p[1] + i[1] + d[1])

        current_servo_positions = (
            max(-MAX_SERVO_DEG, min(MAX_SERVO_DEG, -(pid[0] - 5))),
            max(-MAX_SERVO_DEG, min(MAX_SERVO_DEG, -(pid[1] - 5))),
        )
        controller._servo_controller.set_degrees_bbt(current_servo_positions)

        print(f"Ball Position: {b_pos}, Desired Position: {desired_position}")
        time.sleep(0.004)

except KeyboardInterrupt:
    print("\nKeyboard interrupt received. Shutting down gracefully...")
    controller._servo_controller.set_degrees_bbt((0, 0))  # Optional: reset platform
    ball_pos_sender.close()
    sys.exit(0)
