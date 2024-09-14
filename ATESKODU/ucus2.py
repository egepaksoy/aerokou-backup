import time
import math
import sys
import socket
import queue
import threading
sys.path.append('../pymavlink_custom')
from pymavlink_custom import Vehicle, calc_hipo_angle, calc_location

# KONUM BEKLEME THREAD FONKSIYONU
def start_server(host, port, result_q):
    global running

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server_socket:
        server_socket.settimeout(1.0)
        server_socket.bind((host, port))
        print(f"Listening on {host}:{port}")
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


def failsafe_drone_id(vehicle, drone_id):
    print(f"Drone {drone_id}, Failsafe alıyor")
    vehicle.set_mode(mode="RTL", drone_id=drone_id)

def failsafe(vehicle):
    thraeds = []
    for d_id in vehicle.drone_ids:
        thraeds.append(threading.Thread(target=failsafe_drone_id, args=(vehicle, d_id)))
        thraeds[-1].start()

    for t in thraeds:
        t.join()

    print(f"Dronlar {vehicle.drone_ids} Failsafe aldi")

if len(sys.argv) != 2:
    print("Bağlantı yok")
    exit()

vehicle = Vehicle(sys.argv[1])

tarama_pos = (-35.36305704, 149.16518240)
ates_pos = []

alt = 6
top_birakma_alt = 4

drone_id = 1

area_meter = 5
distance_meter = 1

running = True

try:
    result_q = queue.Queue()
    server_thread = threading.Thread(target=start_server, args=("0.0.0.0", 9000, result_q))

    waypoints = [[tarama_pos[0], tarama_pos[1], alt]]
    waypoints += vehicle.scan_area_wpler(center_lat=tarama_pos[0], center_lon=tarama_pos[1], alt=alt, area_meter=area_meter, distance_meter=distance_meter)
    vehicle.send_all_waypoints(wp_list=waypoints, drone_id=drone_id)

    vehicle.set_mode(mode="GUIDED", drone_id=drone_id)
    vehicle.arm_disarm(arm=True, drone_id=drone_id)
    vehicle.takeoff(alt=alt, drone_id=drone_id)
    takeoff_pos = vehicle.get_pos(drone_id=drone_id)
    print("Drone takeoff aldı: ", takeoff_pos)

    vehicle.set_mode(mode="AUTO", drone_id=drone_id)
    
    start_time = time.time()
    mode = vehicle.get_mode(drone_id=drone_id)
    while True:
        current_mode = vehicle.get_mode(drone_id=drone_id)
        if time.time() - start_time > 2:
            print("Tarama devam ediyor...")
            print("wp: ", vehicle.get_miss_wp(drone_id=drone_id))
            print("mode: ", vehicle.get_mode(drone_id=drone_id))
            start_time = time.time()
        
        if mode != current_mode:
            print(f"!!!! UYARI MOD DEGISTI: {mode}->{current_mode}")
            mode = current_mode
        
        if server_thread.is_alive() == False:
            drone_pos = vehicle.get_pos(drone_id=drone_id)
            uzaklik, aci = calc_hipo_angle(screen_rat_x_y=(32, 24), x_y=(result_q.get()), alt=float(drone_pos[2]), yaw=float(vehicle.get_yaw(drone_id=drone_id)), alt_met=32)
            ates_pos = calc_location(uzaklik=uzaklik, aci=aci, lat=float(drone_pos[0]), lon=float(drone_pos[1]))
            print("ateş bulundu: ", ates_pos)

        if vehicle.on_location(loc=waypoints[len(waypoints) - 1], seq=len(waypoints) - 1, sapma=1, drone_id=drone_id):
            if ateş_pos:
                print("Tarama bitti ateşe gidiliyor...")
                break

            else:
                print("Tarama bitti ateş bulunamadı rtl alınıyor")
                failsafe(vehicle)
                exit()
    
    vehicle.set_mode(mode="GUIDED", drone_id=drone_id)
    vehicle.go_to(ates_pos[0], ates_pos[1], alt, drone_id=drone_id)

    while vehicle.on_location(loc=ates_pos, seq=0, sapma=1, drone_id=drone_id) == False:
        continue

    print("Ateşe ulaşıldı")
    vehicle.go_to(ates_pos[0], ates_pos[1], top_birakma_alt, drone_id=drone_id)
    print("Top bırakma noktasına gidiliyor")

    while vehicle.get_pos(drone_id=drone_id)[2] > top_birakma_alt * 1.1:
        continue

    print("Top bırakılıyor...")
    vehicle.set_servo(channel=9, pwm=1000, drone_id=drone_id)
    time.sleep(1.5)
    vehicle.set_servo(channel=9, pwm=1750, drone_id=drone_id)
    time.sleep(1.5)
    vehicle.set_servo(channel=9, pwm=0, drone_id=drone_id)

    print("Top bırakıldı dönülüyor...")
    vehicle.go_to(ates_pos[0], ates_pos[1], alt, drone_id=drone_id)

    while vehicle.get_pos(drone_id=drone_id)[2] < alt * 0.9:
        continue

    vehicle.go_to(lat=takeoff_pos[0], lon=takeoff_pos[1], alt=alt, drone_id=drone_id)
    print("Dönülüyor...")

    while vehicle.on_location(loc=takeoff_pos, seq=0, sapma=1, drone_id=drone_id) == False:
        continue

    print("Dönüldü")
    vehicle.set_mode(mode="LAND", drone_id=drone_id)


except KeyboardInterrupt:
    failsafe(vehicle)
    print("Kod kapatıldı")

except Exception as e:
    failsafe(vehicle)
    print(f"Error: {e}")