import socket
import numpy as np
import time
from evdev import InputDevice, ecodes

# Configuration
TOUCH_DEVICE = "/dev/input/event2"
LAPTOP_IP = "192.168.0.115"  # Replace with your laptop IP
UDP_PORT = 6006

# UDP setup
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Open touchscreen device
dev = InputDevice(TOUCH_DEVICE)

# Touchscreen calibration boundaries
RAW_X_MIN, RAW_X_MAX = 157, 1976
RAW_Y_MIN, RAW_Y_MAX = 121, 1920
MM_X_MIN, MM_X_MAX = -172, 172
MM_Y_MIN, MM_Y_MAX = -136, 136  # Inverted later for GUI

def translate(value, raw_min, raw_max, mm_min, mm_max):
    value = max(min(value, raw_max), raw_min)
    scale = (mm_max - mm_min) / (raw_max - raw_min)
    return mm_min + (value - raw_min) * scale

def run_tracking_loop():
    x_raw = y_raw = None

    try:
        for event in dev.read_loop():
            if event.type == ecodes.EV_ABS:
                if event.code == ecodes.ABS_X:
                    x_raw = event.value
                elif event.code == ecodes.ABS_Y:
                    y_raw = event.value

                if x_raw is not None and y_raw is not None:
                    x_mm = translate(x_raw, RAW_X_MIN, RAW_X_MAX, MM_X_MIN, MM_X_MAX)
                    y_mm = translate(y_raw, RAW_Y_MIN, RAW_Y_MAX, MM_Y_MAX, MM_Y_MIN)  # Y flipped

                    data = np.array([x_mm, y_mm], dtype=np.float32).tobytes()
                    sock.sendto(data, (LAPTOP_IP, UDP_PORT))
                    print(f"Sent: ({x_mm:.1f}, {y_mm:.1f})")
                    time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopped by user.")

if __name__ == "__main__":
    run_tracking_loop()
