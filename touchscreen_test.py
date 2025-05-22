from evdev import InputDevice, ecodes
import time

# Connect to the correct touchscreen input event
dev = InputDevice('/dev/input/event2')

def ball_position_raw():
    x = y = None

    try:
        for event in dev.read_loop():
            if event.type == ecodes.EV_ABS:
                if event.code == ecodes.ABS_X:
                    x = event.value
                elif event.code == ecodes.ABS_Y:
                    y = event.value

                if x is not None and y is not None:
                    print(f"Ball position: ({x}, {y})")
                    # Optional: sleep a bit to reduce CPU usage
                    time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nExited cleanly via Ctrl+C.")

if __name__ == "__main__":
    ball_position_raw()
