import time
import sys
import threading
import socket
import queue
sys.path.append('../pymavlink_custom')
from pymavlink_custom import *

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

def alaca_miss(vehicle, alaca_takeoff_pos, drone_id: int=None):
    global running

    result_q = queue.Queue()
    server_thread = threading.Thread(target=start_server, args=("0.0.0.0", 9000, result_q))
    server_thread.start()

    vehicle.set_mode(mode="AUTO", drone_id=drone_id)
    print("Alaca taramaya başlıyor...")
    ates_pos = []

    start_time = time.time()
    while running:
        if time.time() - start_time > 2:
            print("Alaca taramaya devam ediyor...")
            start_time = time.time()

        if server_thread.is_alive() == False:
            drone_pos = vehicle.get_pos(drone_id=drone_id)
            uzaklik, aci = calc_hipo_angle(screen_rat_x_y=(32, 24), x_y=(result_q.get()), alt=float(drone_pos[2]), yaw=float(vehicle.get_yaw(drone_id=drone_id)), alt_met=32)
            ates_pos = calc_location(uzaklik=uzaklik, aci=aci, lat=float(drone_pos[0]), lon=float(drone_pos[1]))
            print("ateş bulundu: ", ates_pos)
            break
        
        if vehicle.get_miss_wp(drone_id=drone_id) == len(alaca_scan_positions) - 1:
            break

    if running == False:
        return

    if len(ates_pos) == 0:
        print("Ates pozisyonu bulunamadı RTL alıyor...")
        failsafe(vehicle)
        return
    
    start_time = time.time()
    while running:
        if time.time() - start_time > 2:
            print("Alaca taramaya devam ediyor...")
            start_time = time.time()

        if vehicle.get_miss_wp(drone_id=drone_id) == len(alaca_scan_positions) - 1:
            break
    
    if running == False:
        return
    
    print("Alaca taramasını tamamladı ateşe gidiyor...")
    vehicle.set_mode(mode="GUIDED", drone_id=drone_id)
    vehicle.go_to(lat=ates_pos[0], lon=ates_pos[1], alt=alaca_alt, drone_id=drone_id)
    
    while running:
        if vehicle.on_location(loc=ates_pos, seq=0, sapma=1, drone_id=drone_id):
            break
    
    if running == False:
        return

    print("Alaca ateşe geldi top bırakıyor...")

    vehicle.go_to(lat=ates_pos[0], lon=ates_pos[1], alt=alaca_top_birakma_alt, drone_id=drone_id)
    while running:
        if vehicle.get_pos(drone_id=drone_id)[2] <= alaca_top_birakma_alt * 1.1:
            break
    
    if running == False:
        return
    
    vehicle.set_servo(channel=9, pwm=1000, drone_id=drone_id)
    time.sleep(1.5)
    vehicle.set_servo(channel=9, pwm=1750, drone_id=drone_id)
    time.sleep(1.5)
    vehicle.set_servo(channel=9, pwm=0, drone_id=drone_id)

    vehicle.go_to(lat=ates_pos[0], lon=ates_pos[1], alt=alaca_alt, drone_id=drone_id)
    while running:
        if vehicle.get_pos(drone_id=drone_id)[2] >= alaca_alt * 0.9:
            break;
    
    if running == False:
        return
    
    print("Alaca top bıraktı dönüş yapıyor...")

    vehicle.set_mode(mode="RTL", drone_id=drone_id)



if len(sys.argv) != 2:
    exit()
vehicle = Vehicle(sys.argv[1])

running = True

korfez_id = 1
alaca_id = 3

if len(vehicle.drone_ids) != 2:
    print("Dronlar bağlı değil")
    #exit()

korfez_scan_meter2 = 10
korfez_distance_meter2 = 2
korfez_scan_meter1 = 5
korfez_distance_meter1 = 1

alaca_scan_meter = 3
alaca_distance_meter = 1

korfez_pos = (40.7119979, 30.0245386)

alaca_pos = []
ates_pos = []

korfez_alt = 7
alaca_alt = 4

alaca_top_birakma_alt = 3

try:
    print("Uçuş başlıyor")
    
    korfez_missions = [[korfez_pos[0], korfez_pos[1], korfez_alt]]
    korfez_missions1 = vehicle.scan_area_wpler(center_lat=korfez_pos[0], center_lon=korfez_pos[1], alt=korfez_alt, area_meter=korfez_scan_meter1, distance_meter=korfez_distance_meter1)
    korfez_missions += korfez_missions1
    vehicle.send_all_waypoints(wp_list=korfez_missions, drone_id=korfez_id)
    print("Korfez waypointleri atıldı")

    vehicle.set_mode(mode="GUIDED", drone_id=korfez_id)
    vehicle.arm_disarm(arm=True, drone_id=korfez_id)
    vehicle.takeoff(alt=korfez_alt, drone_id=korfez_id)
    korfez_takeoff_pos = vehicle.get_pos(drone_id=korfez_id)
    print("Körfez kalkış pozisyonu: ", korfez_takeoff_pos)

    vehicle.set_mode(mode="AUTO", drone_id=korfez_id)

    start_time = time.time()
    mission_time = time.time()
    current_wp = vehicle.get_miss_wp(drone_id=korfez_id)
    while True and running:
        if time.time() - start_time > 2:
            print("Görev devam ediyor")
            print("wp: ", vehicle.get_miss_wp(drone_id=korfez_id))
            print("mode: ", vehicle.get_mode(drone_id=korfez_id))
            start_time = time.time()

        if vehicle.get_miss_wp(drone_id=korfez_id) != current_wp:
            print("Görev devam ediyor...")
            print("mode: ", vehicle.get_mode(drone_id=korfez_id))
            print("wp: ", vehicle.get_miss_wp(drone_id=korfez_id))
            start_time = time.time()
            current_wp = vehicle.get_miss_wp(drone_id=korfez_id)

        with open("./gorev_dosyaları/korfez_konum.txt", "r") as f:
            drone_pos = vehicle.get_pos(drone_id=korfez_id)
            drone_yaw = vehicle.get_yaw(drone_id=korfez_id)
            for line in f:
                pixel_pos = get_pixel_pos(line)
                uzaklik, aci = calc_hipo_angle(screen_rat_x_y=(640, 480), x_y=pixel_pos, alt=drone_pos[2], yaw=drone_yaw, alt_met=640)
                ates_pos = calc_location(uzaklik=uzaklik, aci=aci, lat=drone_pos[0], lon=drone_pos[1])
                print("Ateş konumu çekildi: ", ates_pos)

            if len(ates_pos) != 0:
                break
        
        if vehicle.get_miss_wp(drone_id=korfez_id) == len(korfez_missions1) - 1:
            print("Körfez taramasını tamamladı")
            vehicle.set_mode(mode="RTL", drone_id=korfez_id)
            break

    while True:
        if time.time() - start_time > 2:
            print("Görev devam ediyor...")
            print("mode: ", vehicle.get_mode(drone_id=korfez_id))
            print("wp: ", vehicle.get_miss_wp(drone_id=korfez_id))
            start_time = time.time()
        
        if vehicle.get_miss_wp(drone_id=korfez_id) != current_wp:
            print("Görev devam ediyor...")
            print("mode: ", vehicle.get_mode(drone_id=korfez_id))
            print("wp: ", vehicle.get_miss_wp(drone_id=korfez_id))
            start_time = time.time()
            current_wp = vehicle.get_miss_wp(drone_id=korfez_id)

        if vehicle.get_miss_wp(drone_id=korfez_id) == len(korfez_missions1) - 1:
            print("Körfez taramasını tamamladı")
            vehicle.set_mode(mode="RTL", drone_id=korfez_id)
            break

    if len(ates_pos) == 0:
        print("Ates pozisyonu bulunamadı RTL alıyor...")
        failsafe(vehicle)
        exit()

    # #################### ALACA #####################
    alaca_pos = ates_pos
    print("Alaca görev konumu: ", alaca_pos)
    print("Alaca kalkışa geçti")
    vehicle.set_mode(mode="GUIDED", drone_id=alaca_id)
    vehicle.arm_disarm(arm=True, drone_id=alaca_id)
    vehicle.takeoff(alt=alaca_alt, drone_id=alaca_id)
    alaca_takeoff_pos = vehicle.get_pos(drone_id=alaca_id)
    print("Alaca Takeoff yaptı: ", alaca_takeoff_pos)
    
    print("Alaca görevler atılıyor...")
    # ALACA WAYPOINTLER
    alaca_scan_positions = [[alaca_pos[0], alaca_pos[1], alaca_alt]]
    alaca_scan_positions += vehicle.scan_area_wpler(center_lat=alaca_pos[0], center_lon=alaca_pos[1], alt=alaca_alt, area_meter=alaca_scan_meter, distance_meter=alaca_distance_meter)
    vehicle.send_all_waypoints(wp_list=alaca_scan_positions, drone_id=alaca_id)
    print("Alaca'nın görevleri atıldı uçuş başlıyor...")
    alaca_miss_thread = threading.Thread(target=alaca_miss, args=(vehicle, alaca_takeoff_pos, alaca_id))
    alaca_miss_thread.start()

    while alaca_miss_thread.is_alive():
        continue
    
except Exception as e:
    print("Hata: ", e)
    failsafe(vehicle)
    running = False

except KeyboardInterrupt:
    print("Koddan çıkıldı ucus durduruluyor...")
    failsafe(vehicle)
    running = False

finally:
    print("Kod sonlandı")
    running = False
    vehicle.vehicle.close()
