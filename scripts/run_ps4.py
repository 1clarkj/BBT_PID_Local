import time
import datetime
import threading
import pygame
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from pid_local.hardware.table_controller import BallBalanceTableControllerv2

# Initialize pygame and joystick
pygame.init()
pygame.joystick.init()

# Check if there are any joysticks connected
joystick_count = pygame.joystick.get_count()
if joystick_count == 0:
    print("No joysticks connected.")
    exit()

# Get the first joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

analog_keys = {0: 0, 1: 0}  # Axis 0 for X and Axis 1 for Y
desired_position_x = 0
desired_position_y = 0

K_P = (0.25, 0.25)
K_I = (0.0, 0.0)
K_D = (25, 25)

e_filter = 15
d_filter = 75

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

controller = BallBalanceTableControllerv2()
controller.start()

# Function to update the desired position based on joystick input
def update_position_joystick():
    global desired_position_x, desired_position_y
    while True:
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                analog_keys[event.axis] = event.value

        # Use joystick axis 0 and 1 to update the desired position
        # Multiplier to scale joystick input to position (can be adjusted)
        multiplier = 5
        desired_position_x += analog_keys[0] * multiplier
        desired_position_y += analog_keys[1] * multiplier

        # Limit the position values to a reasonable range (adjust according to your setup)
        desired_position_x = max(-100, min(100, desired_position_x))
        desired_position_y = max(-100, min(100, desired_position_y))

        print(f"Desired Position X: {desired_position_x}, Y: {desired_position_y}")
        time.sleep(0.03)

# Start the joystick position update thread
joystick_thread = threading.Thread(target=update_position_joystick)
joystick_thread.daemon = True
joystick_thread.start()

# Main PID control loop
while True:
    current_time = datetime.datetime.now()
    time_delta = current_time - last_time
    dt = time_delta.seconds
    last_time = current_time

    b_pos = controller.get_ball_position_in_mm()

    last_error = current_error

    desired_position = (desired_position_x, desired_position_y)
    current_error = (desired_position[0] - b_pos[0], desired_position[1] - b_pos[1])

    p_error = current_error
    e_terms[e_counter] = p_error

    sum_e = (0, 0)

    for j in range(e_filter):
        sum_e = (sum_e[0] + e_terms[j][0], sum_e[1] + e_terms[j][1])

    p_error = (sum_e[0] / e_filter, sum_e[1] / e_filter)
    e_counter += 1

    if e_counter >= e_filter:
        e_counter = 0

    sum_error = (sum_error[0] + current_error[0], sum_error[1] + current_error[1])

    delta_error = (current_error[0] - last_error[0], current_error[1] - last_error[1])

    d_terms[d_counter] = delta_error

    sum_d = (0, 0)

    for j in range(d_filter):
        sum_d = (sum_d[0] + d_terms[j][0], sum_d[1] + d_terms[j][1])

    delta_error = (sum_d[0] / d_filter, sum_d[1] / d_filter)
    d_counter += 1

    if d_counter >= d_filter:
        d_counter = 0

    p = (p_error[0] * K_P[0], p_error[1] * K_P[1])
    i = (sum_error[0] * K_I[0], sum_error[1] * K_I[1])
    d = (delta_error[0] * K_D[0], delta_error[1] * K_D[1])

    pid = (p[0] + i[0] + d[0], p[1] + i[1] + d[1])

    # PID -60-60
    current_servo_positions = (-(pid[0] - 5), -(pid[1] - 5))
    controller._servo_controller.set_degrees_bbt(current_servo_positions)
    print(f"Ball position: {b_pos}, Desired position: {desired_position}")

    time.sleep(0.004)
