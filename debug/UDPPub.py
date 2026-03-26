import socket 


def udp_publisher(ip: str, port: int, message: str):
    # Create a UDP socket
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        # Encode the message and send it
        sock.sendto(message.encode(), (ip, port))
        print(f"Sent message '{message}' to {ip}:{port}")

if __name__ == "__main__":

    target_ip = "127.0.0.1"  
    target_port = 53256      

    message = ""
    udp_publisher(target_ip, target_port, message)