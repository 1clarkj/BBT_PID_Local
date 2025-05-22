import time
import datetime
import math

from ball_balance_table_controller_v2 import BallBalanceTableControllerv2

# Initialize the controller
controller = BallBalanceTableControllerv2()
controller.start()

# PID controller parameters
K_P = (0.25, 0.25)
K_I = (0.01, 0.01)
K_D = (25, 25)

# Filter parameters
e_filter = 15
d_filter = 75

# Circle parameters
radius = 50  # radius of the circle in mm
x_center = 0  # center of the circle (x)
y_center = 0  # center of the circle (y)
omega = 0.5  # angular velocity in radians per second

# Servo and error initializations
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
i = (0.0, 0.0)
d = (0.0, 0.0)
pid = (0.0, 0.0)
e_terms = [(0, 0)] * e_filter
d_terms = [(0, 0)] * d_filter
sum_e = (0.0, 0.0)
sum_d = (0.0, 0.0)
sum_error = (0.0, 0.0)
e_counter = 0
d_counter = 0

# Start time for circular motion
start_time = time.time()

# Function to calculate desired position in circular trajectory
def update_desired_position_circular(t, x_center, y_center, radius, omega):
    """
    Update the desired position to follow a circular trajectory.
    
    Parameters:
        t (float): Time in seconds.
        x_center (float): X-coordinate of the circle's center.
        y_center (float): Y-coordinate of the circle's center.
        radius (float): Radius of the circle.
        omega (float): Angular velocity (rad/s).
        
    Returns:
        (float, float): Desired X and Y positions.
    """
    desired_position_x = x_center + radius * math.cos(omega * t)
    desired_position_y = y_center + radius * math.sin(omega * t)
    return desired_position_x, desired_position_y


while True:
    current_time = datetime.datetime.now()
    time_delta = current_time - last_time
    dt = time_delta.total_seconds()
    last_time = current_time

    # Get the current ball position
    b_pos = controller.get_ball_position_in_mm()

    # Get the current time for circular motion
    t = time.time() - start_time

    # Call the function to update the desired position for circular motion
    desired_position_x, desired_position_y = update_desired_position_circular(t, x_center, y_center, radius, omega)

    # Calculate current error
    last_error = current_error
    current_error = (desired_position_x - b_pos[0], desired_position_y - b_pos[1])

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
    i = (sum_error[0] * K_I[0], sum_error[1] * K_I[1])
    d = (delta_error[0] * K_D[0], delta_error[1] * K_D[1])

    pid = (p[0] + i[0] + d[0], p[1] + i[1] + d[1])

    # PID -60-60
    current_servo_positions = (-(pid[0] - 5), -(pid[1] - 5))
    controller._servo_controller.set_degrees_bbt(current_servo_positions)

    # Print current ball and desired positions
    print(f"Ball position: {b_pos}, Desired position: ({desired_position_x:.2f}, {desired_position_y:.2f})")

    # Small delay for control loop timing
    time.sleep(0.004)
