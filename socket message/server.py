import socket

def start_server(host, port):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server_socket:
        server_socket.bind((host, port))
        print(f"Listening on {host}:{port}")
        while True:
            print("bekleniyor...")
            data, addr = server_socket.recvfrom(1024)
            print(f"{addr} adersinden gonderildi: {data.decode()}")
            server_socket.sendto(data, addr)  # İsteğe bağlı olarak geri yanıt gönderir

if __name__ == "__main__":
    start_server('0.0.0.0', 65432)  # Sunucu IP ve port
