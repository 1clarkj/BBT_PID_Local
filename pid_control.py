import time
import datetime

from ball_balance_table_controller_v2 import BallBalanceTableControllerv2

controller = BallBalanceTableControllerv2()
controller.start()

K_P = (0.25, 0.25)
K_I = (0.01, 0.01)  # Introduce small integral gain for now
K_D = (25, 25)

e_filter = 15
d_filter = 75

desired_position_x = 0
desired_position_y = 0

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

while True:
    current_time = datetime.datetime.now()
    time_delta = current_time - last_time
    dt = time_delta.total_seconds()  # Ensure dt is in seconds for better timing accuracy
    last_time = current_time

    b_pos = controller.get_ball_position_in_mm()

    last_error = current_error

    desired_position = (desired_position_x, desired_position_y)
    current_error = (desired_position[0] - b_pos[0], desired_position[1] - b_pos[1])

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

    print(f"Ball position: {b_pos}, PID output: {pid}, Desired position: {desired_position}")
    
    time.sleep(0.004)  # Small delay for control loop timing
