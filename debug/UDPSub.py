import socket

def udp_reader(port: int):
    # Create a UDP socket
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        
        sock.bind(("0.0.0.0", port))  
        print(f"Listening for UDP messages on port {port}...")

        while True:
            # Receive message from the socket
            data, addr = sock.recvfrom(1024)  # buffer size is 1024 bytes
            print(f"Received message: {data.decode()} from {addr}")

if __name__ == "__main__":
    listen_port = 53256  # Port to listen on
    udp_reader(listen_port)