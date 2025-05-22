from evdev import InputDevice, ecodes
import threading
import time

dev = InputDevice('/dev/input/event2')

def ball_position_raw():
    x_pos = y_pos = None
    touching = False

    try:
        for event in dev.read_loop():
            if event.type == ecodes.EV_KEY and event.code == ecodes.BTN_TOUCH:
                touching = bool(event.value)

            elif event.type == ecodes.EV_ABS:
                if event.code == ecodes.ABS_X:
                    x_pos = event.value
                elif event.code == ecodes.ABS_Y:
                    y_pos = event.value

            if touching and x_pos is not None and y_pos is not None:
                print(f"Touching at: ({x_pos}, {y_pos})")
                # Optional: add a small sleep to reduce CPU usage
                time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nExiting cleanly...")

if __name__ == "__main__":
    ball_position_raw()
