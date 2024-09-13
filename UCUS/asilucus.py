import time
import sys
import threading
sys.path.append('../pymavlink_custom')
from pymavlink_custom import Vehicle

def alaca_miss(vehicle, alaca_takeoff_pos, drone_id: int=None):
    global running

    vehicle.set_mode(mode="AUTO", drone_id=drone_id)
    print("Alaca taramaya başlıyor...")

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
    vehicle.go_to(lat=alaca_pos[0], lon=alaca_pos[1], alt=alaca_alt, drone_id=drone_id)
    
    while running:
        if vehicle.on_location(loc=alaca_pos, seq=0, sapma=1, drone_id=drone_id):
            break
    
    if running == False:
        return

    print("Alaca ateşe geldi top bırakıyor...")

    vehicle.go_to(lat=alaca_pos[0], lon=alaca_pos[1], alt=alaca_top_birakma_alt, drone_id=drone_id)
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

    vehicle.go_to(lat=alaca_pos[0], lon=alaca_pos[1], alt=alaca_alt, drone_id=drone_id)
    while running:
        if vehicle.get_pos(drone_id=drone_id)[2] >= alaca_alt * 0.9:
            break;
    
    if running == False:
        return
    
    print("Alaca top bıraktı dönüş yapıyor...")

    vehicle.go_to(lat=alaca_takeoff_pos[0], lon=alaca_takeoff_pos[1], alt=alaca_alt, drone_id=drone_id)

    while running:
        if vehicle.on_location(loc=alaca_takeoff_pos, seq=0, sapma=1, drone_id=drone_id):
            break
    
    if running == False:
        return
    
    print("Alaca dönüş yaptı LAND alıyor")
    vehicle.set_mode(mode="LAND", drone_id=drone_id)


def feniks_miss(vehicle, feniks_takeoff_pos, drone_id: int=None):
    global running

    vehicle.set_mode(mode="AUTO", drone_id=drone_id)
    print("Feniks uçuşa başlıyor...")

    start_time = time.time()
    while running:
        if time.time() - start_time > 2:
            print("Feniks konuma gidiyor...")
            start_time = time.time()
        
        if vehicle.on_location(loc=feniks_pos, seq=1, sapma=1, drone_id=drone_id):
            break
    
    if running == False:
        return
    
    vehicle.set_mode(mode="GUIDED", drone_id=drone_id)
    vehicle.go_to(lat=feniks_pos[0], lon=feniks_pos[1], alt=feniks_top_birakma_alt, drone_id=drone_id)
    print("Feniks görevi yapıyor")

    while vehicle.get_pos(drone_id=drone_id)[2] > feniks_top_birakma_alt * 1.1 and running:
        continue
    
    if running == False:
        return
    
    vehicle.set_servo(channel=9, pwm=1000, drone_id=drone_id)
    time.sleep(1.5)
    vehicle.set_servo(channel=9, pwm=1750, drone_id=drone_id)
    time.sleep(1.5)
    vehicle.set_servo(channel=9, pwm=0, drone_id=drone_id)

    vehicle.go_to(lat=feniks_pos[0], lon=feniks_pos[1], alt=feniks_alt, drone_id=drone_id)
    
    while vehicle.get_pos(drone_id=drone_id)[2] < feniks_alt * 0.9 and running:
        continue
    
    if running == False:
        return

    print("Feniks görevi tamamladı dönüşe geçti")
    
    vehicle.go_to(lat=feniks_takeoff_pos[0], lon=feniks_takeoff_pos[1], alt=feniks_alt, drone_id=drone_id)

    start_time = time.time()
    while running:
        if time.time() - start_time > 2:
            print("Feniks dönüyor...")
            start_time = time.time()
        
        if vehicle.on_location(loc=feniks_takeoff_pos, seq=0, sapma=1, drone_id=drone_id):
            break
    
    if running == False:
        return
    
    print("Feniks dönüş yaptı LAND alıyor")
    vehicle.set_mode(mode="LAND", drone_id=drone_id)


if len(sys.argv) != 2:
    exit()
vehicle = Vehicle(sys.argv[1])

running = True

korfez_id = 1
feniks_id = 2
alaca_id = 3

if len(vehicle.drone_ids) != 3:
    print("Dronlar bağlı değil")
    exit()

korfez_scan_meter2 = 10
korfez_distance_meter2 = 2
korfez_scan_meter1 = 5
korfez_distance_meter1 = 1

alaca_scan_meter = 5
alaca_distance_meter = 1

korfez_pos = (40.7119922, 30.0245655)
feniks_pos = korfez_pos
alaca_pos = (korfez_pos[0] + vehicle.DEG * 1.5, korfez_pos[1])

korfez_alt = 10
feniks_alt = 4
alaca_alt = 7

alaca_top_birakma_alt = 4
feniks_top_birakma_alt = 3

try:
    print("Uçuş başlıyor")
    
    korfez_missions = [[korfez_pos[0], korfez_pos[1], korfez_alt]]
    korfez_missions1 = vehicle.scan_area_wpler(center_lat=korfez_pos[0], center_lon=korfez_pos[1], alt=korfez_alt, area_meter=korfez_scan_meter1, distance_meter=korfez_distance_meter1)
    korfez_missions2 = vehicle.scan_area_wpler(center_lat=korfez_pos[0], center_lon=korfez_pos[1], alt=korfez_alt, area_meter=korfez_scan_meter2, distance_meter=korfez_distance_meter2)
    korfez_missions += korfez_missions1 + korfez_missions2
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
    while True and running:
        if time.time() - start_time > 2:
            print("Görev devam ediyor")
            print("wp: ", vehicle.get_miss_wp(drone_id=korfez_id))
            print("mode: ", vehicle.get_mode(drone_id=korfez_id))
            start_time = time.time()
        
        if vehicle.get_miss_wp(drone_id=korfez_id) == len(korfez_missions1):
            print("Körfez taramasını tamamladı")
            print("Izlemeye geciyor...")
            break

    # #################### ALACA #####################
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

    # #################### FENIKS #####################
    print("Feniks kalkışa geçti")
    vehicle.set_mode(mode="GUIDED", drone_id=feniks_id)
    vehicle.arm_disarm(arm=True, drone_id=feniks_id)
    vehicle.takeoff(alt=feniks_alt, drone_id=feniks_id)
    feniks_takeoff_pos = vehicle.get_pos(drone_id=feniks_id)
    print("Feniks Takeoff yaptı: ", feniks_takeoff_pos)

    print("Feniks görevler atılıyor...")
    # FENIKS WAYPOINTLER
    feniks_wpler = [[feniks_pos[0], feniks_pos[1], feniks_alt], [feniks_pos[0], feniks_pos[1], feniks_alt]]
    vehicle.send_all_waypoints(wp_list=feniks_wpler, drone_id=feniks_id)
    print("Feniks'in gorevleri atıldı ucus baslıyor...")
    feniks_miss_thread = threading.Thread(target=feniks_miss, args=(vehicle, feniks_takeoff_pos, feniks_id))
    feniks_miss_thread.start()

    while True:
        if vehicle.get_miss_wp(drone_id=korfez_id) == len(korfez_missions1) + len(korfez_missions2) - 1:
            print("Körfez taramasını tamamladı dönüş yapıyor...")
            vehicle.set_mode(mode="GUIDED", drone_id=korfez_id)

            vehicle.go_to(lat=korfez_takeoff_pos[0], lon=korfez_takeoff_pos[1], alt=korfez_alt, drone_id=korfez_id)
            break

    while True:
        if vehicle.on_location(loc=korfez_takeoff_pos, seq=0, sapma=1, drone_id=korfez_id):
            vehicle.set_mode(mode="LAND", drone_id=korfez_id)
            print("Körfez dönüş yaptı LAND alıyor")
            break

    while True:
        if feniks_miss_thread.is_alive() or alaca_miss_thread.is_alive():
            continue
    
except Exception as e:
    print("Hata: ", e)
    running = False

except KeyboardInterrupt:
    print("Koddan çıkıldı ucus durduruluyor...")
    running = False

finally:
    for drone_id in vehicle.drone_ids:
        vehicle.set_mode(mode="RTL", drone_id=drone_id)
    print("Kod sonlandı")
    running = False
    vehicle.vehicle.close()
    exit()
