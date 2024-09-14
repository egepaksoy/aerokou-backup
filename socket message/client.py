import socket
import sys

def send_message(host, port, message):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client_socket:
        client_socket.sendto(message.encode(), (host, port))
        response, _ = client_socket.recvfrom(1024)
        print(f"Gonderildi: {response.decode()}")

if __name__ == "__main__":
    send_message(sys.argv[1], int(sys.argv[2]), sys.argv[3])  # Hedef IP ve port
