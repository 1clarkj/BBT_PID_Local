import socket
import time

UDP_IP = "127.0.0.1"  # IP address where your C++ receiver is running
UDP_PORT = 6006       # Port your C++ UDPSender is listening on

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

toggle_state = False

print("Sending toggle messages to", UDP_IP, "on port", UDP_PORT)

try:
    while True:
        # Toggle the state
        toggle_state = not toggle_state
        message = b"1" if toggle_state else b"0"
        print(f"Sending toggle: {message.decode()}")

        sock.sendto(message, (UDP_IP, UDP_PORT))

        time.sleep(5)  # Wait 5 seconds before toggling again

except KeyboardInterrupt:
    print("\nStopped sending.")
    sock.close()
