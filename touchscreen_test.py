import evdev
from evdev import InputDevice, categorize, ecodes

from evdev import InputDevice, ecodes
import time

# Open the touchscreen input device
dev = InputDevice('/dev/input/event2')

def ball_position_raw():
    x_pos = y_pos = None
    for event in dev.read_loop():
        if event.type == ecodes.EV_ABS:
            if event.code == ecodes.ABS_X:
                x_pos = event.value
            elif event.code == ecodes.ABS_Y:
                y_pos = event.value

        # Return a full coordinate only when both x and y are valid
        if x_pos is not None and y_pos is not None:
            return (x_pos, y_pos)
while True:
    pos = ball_position_raw()
    print(f"Ball position: {pos}")
    time.sleep(0.01)
