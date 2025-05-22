import evdev
from evdev import InputDevice, categorize, ecodes

# Replace this with your actual device path (found using evtest)
# determine with python3 -m evdev.evtest
DEVICE_PATH = "/dev/input/event2"

try:
    dev = InputDevice(DEVICE_PATH)
    print(f"Listening for touch events on: {dev.name} ({dev.path})")
except FileNotFoundError:
    print(f"Device not found at {DEVICE_PATH}")
    exit(1)

x = y = None
touching = False

for event in dev.read_loop():
    if event.type == ecodes.EV_ABS:
        absevent = categorize(event)
        if event.code == ecodes.ABS_MT_POSITION_X:
            x = event.value
        elif event.code == ecodes.ABS_MT_POSITION_Y:
            y = event.value

    elif event.type == ecodes.EV_KEY and event.code == ecodes.BTN_TOUCH:
        if event.value == 1:
            touching = True
        elif event.value == 0:
            touching = False
            if x is not None and y is not None:
                print(f"Touch released at ({x}, {y})")

    if touching and x is not None and y is not None:
        print(f"Touching at ({x}, {y})")
