import time
import math
import sys
import queue
import socket
import threading
sys.path.append('../pymavlink_custom')
from pymavlink_custom import Vehicle, calc_hipo_angle, calc_location

def failsafe_drone_id(vehicle, drone_id):
    print(f"Drone {drone_id}, Failsafe alıyor")
    vehicle.set_mode(mode="RTL", drone_id=drone_id)

def failsafe(vehicle):
    thraeds = []
    for d_id in vehicle.drone_ids:
        thrd = threading.Thread(target=failsafe_drone_id, args=(vehicle, d_id))
        thrd.start()
        thraeds.append(thrd)

    for t in thraeds:
        t.join()

    print(f"Dronlar {vehicle.drone_ids} Failsafe aldi")

# KONUM BEKLEME THREAD FONKSIYONU
def start_server(host, port, result_q):
    global running

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server_socket:
        server_socket.settimeout(1.0)
        server_socket.bind((host, port))
        print(f"Konumlar bekleniyor {host}:{port}")
        while running:
            try:
                data, addr = server_socket.recvfrom(1024)
                print(f"{addr} adersinden ateş konumu: {data.decode()}")
                konum = data.decode().split(",")

                result_q.put((float(konum[0]), float(konum[1])))

                response = "success"
                server_socket.sendto(response.encode(), addr)
                print("Termal tarama kodu durduruldu")
                break

            except socket.timeout:
                continue

if len(sys.argv) != 2:
    print("Bağlantı yok")
    exit()

vehicle = Vehicle(sys.argv[1])

running = True

loc = (40.7121125, 30.0245072)

alt = 5

drone_id = 3

result_q = queue.Queue()
server_thread = threading.Thread(target=start_server, args=("0.0.0.0", 9000, result_q))

try:
    waypoints = [[loc[0], loc[1], alt], [loc[0], loc[1], alt]]
    vehicle.send_all_waypoints(wp_list=waypoints, drone_id=drone_id)

    vehicle.set_mode(mode="GUIDED", drone_id=drone_id)
    vehicle.arm_disarm(arm=True, drone_id=drone_id)
    vehicle.takeoff(alt=alt, drone_id=drone_id)
    takeoff_pos = vehicle.get_pos(drone_id=drone_id)
    print("Drone takeoff aldı: ", takeoff_pos)
    server_thread.start()

    vehicle.set_mode(mode="AUTO", drone_id=drone_id)

    start_time = time.time()

    while True:
        if time.time() - start_time > 2:
            print("mode: ", vehicle.get_mode(drone_id=drone_id))
            print("pos: ", vehicle.get_pos(drone_id=drone_id))
            start_time = time.time()
        
        if vehicle.on_location(loc=loc, seq=1, sapma=1, drone_id=drone_id) or vehicle.on_location(loc=loc, seq=0, sapma=1, drone_id=drone_id):
            print("Görev bitti")
            break
    
    vehicle.set_mode(mode="LAND", drone_id=drone_id)

except KeyboardInterrupt:
    failsafe(vehicle)
    print("Koddan cıkıldı")

except Exception as e:
    running = False
    failsafe(vehicle)
    print("Error: ", e)

finally:
    running = False