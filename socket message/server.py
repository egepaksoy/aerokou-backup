import socket
import threading
import queue
import sys
import time

def start_server(host, port, result_q):
    global running

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server_socket:
        server_socket.settimeout(2.0)
        server_socket.bind((host, port))
        print(f"Listening on {host}:{port}")
        while running:
            try:
                print("bekleniyor...")
                data, addr = server_socket.recvfrom(1024)
                print(f"{addr} adersinden gonderildi: {data.decode()}")
                result_q.put(data.decode())

                response = "success"
                server_socket.sendto(response.encode(), addr)
                print(f"{addr} adresine geri veri gönderildi: {response}")

                break
            except socket.timeout:
                continue

running = True

result_q = queue.Queue()

server_thread = threading.Thread(target=start_server, args=("0.0.0.0", 9000, result_q))
server_thread.start()

print("Server mesajı bekleniyor")

try:
    start_time = time.time()
    while server_thread.is_alive():
        if time.time() - start_time > 2:
            print("Mesaj bekelniyor")
            start_time = time.time()

    print(len(result_q.get()))

except KeyboardInterrupt:
    running = False
    print("koddan cikildi")

except Exception as e:
    print("Hata: ", e)

finally:
    print("Bitti")