import socket

def send_message(host, port, message):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client_socket:
        client_socket.sendto(message.encode(), (host, port))
        response, _ = client_socket.recvfrom(1024)
        print(f"Gonderildi: {response.decode()}")

if __name__ == "__main__":
    send_message("172.16.13.28", 65432, 'Merhaba Dünya')  # Hedef IP ve port
