import socket
import json
import threading
import time


def udp_send(ip: str, port: int, payload: dict):
    buf = json.dumps(payload).encode("utf-8")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.sendto(buf, (ip, port))
        print(f"Sent {payload} to {ip}:{port}")
    finally:
        sock.close()


def udp_reader(listen_port: int):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", listen_port))
    print(f"Listening for UDP on port {listen_port}...")

    while True:
        data, addr = sock.recvfrom(2048)
        try:
            msg = json.loads(data.decode("utf-8"))
        except json.JSONDecodeError:
            msg = data.decode("utf-8", errors="replace")

        print(f"Received {msg} from {addr}")


if __name__ == "__main__":
    pi_hostname = "raspberrypi.local"
    pi_ip = socket.gethostbyname(pi_hostname)
    pi_port = 8008

    gui_listening_port = 53256

    # Start listening in the background
    threading.Thread(target=udp_reader, args=(gui_listening_port,), daemon=True).start()

    # Build handshake payload
    payload = {"type": "hello", "port": gui_listening_port}


    # Send handshake
    udp_send(pi_ip, pi_port, payload)

    # Keep main alive and optionally publish periodically
    while True:
        time.sleep(2)
        udp_send(pi_ip, pi_port, {"type": "status", "t": time.time(), "port": gui_listening_port})
