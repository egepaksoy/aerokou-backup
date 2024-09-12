import time
import sys
import threading
sys.path.append('../pymavlink_custom')
from pymavlink_custom import Vehicle

def alaca_miss(vehicle, alaca_takeoff_pos, drone_id: int=None):
    global running

    start_time = time.time()
    while running:
        if time.time() - start_time > 2:
            print("Alaca taramaya devam ediyor...")
            start_time = time.time()
        
        if vehicle.on_location(loc=alaca_scan_values[1], seq=alaca_scan_values[0], sapma=1, drone_id=drone_id):
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
korfez_scan_meter = 5
korfez_distance_meter = 1

alaca_scan_meter = 5
alaca_distance_meter = 1

korfez_pos = (40.7119793, 30.0245888)
feniks_pos = korfez_pos
alaca_pos = (korfez_pos[0] + vehicle.DEG * 1.5, korfez_pos[1])

korfez_alt = 11
feniks_alt = 6
alaca_alt = 8

alaca_top_birakma_alt = 5
feniks_top_birakma_alt = 4

try:
    print("Uçuş başlıyor")
    for drone_id in vehicle.drone_ids:
        vehicle.set_mode(mode="GUIDED", drone_id=drone_id)
    
    # KORFEZ WAYPOINTLER
    vehicle.add_mission(0, korfez_pos[0], korfez_pos[1], korfez_alt, korfez_id)
    korfez_scan_values1 = vehicle.scan_area(1, korfez_pos[0], korfez_pos[1], korfez_alt, korfez_scan_meter, korfez_distance_meter, korfez_id)
    korfez_scan_values2 = vehicle.scan_area(korfez_scan_values1[0] + 1, korfez_pos[0], korfez_pos[1], korfez_alt, korfez_scan_meter2, korfez_distance_meter2, korfez_id)

    # ALACA WAYPOINTLER
    vehicle.add_mission(0, alaca_pos[0], alaca_pos[1], alaca_alt, alaca_id)
    alaca_scan_values = vehicle.scan_area(1, alaca_pos[0], alaca_pos[1], alaca_alt, alaca_scan_meter, alaca_distance_meter, alaca_id)

    # FENIKS WAYPOINTLER
    vehicle.add_mission(0, feniks_pos[0], feniks_pos[1], feniks_alt, feniks_id)
    vehicle.add_mission(1, feniks_pos[0], feniks_pos[1], feniks_alt, feniks_id)

    print("Dronlara waypointler atıldı")


    vehicle.arm_disarm(arm=True, drone_id=korfez_id)
    vehicle.takeoff(alt=korfez_alt, drone_id=korfez_id)
    korfez_takeoff_pos = vehicle.get_pos(drone_id=korfez_id)

    print("Körfez kalkış yaptı: ", korfez_takeoff_pos)

    vehicle.set_mode(mode="AUTO", drone_id=korfez_id)

    start_time = time.time()
    while True and running:
        if time.time() - start_time > 2:
            print("Görev devam ediyor")
            start_time = time.time()
        
        if vehicle.get_miss_wp(drone_id=korfez_id) == korfez_scan_values1[0]:
            print("Körfez taramasını tamamladı")
            print("Izlemeye geciyor...")
            break

    vehicle.arm_disarm(arm=True, drone_id=alaca_id)
    vehicle.takeoff(alt=alaca_alt, drone_id=alaca_id)
    alaca_takeoff_pos = vehicle.get_pos(drone_id=alaca_id)
    print("Alaca Takeoff yaptı: ", alaca_takeoff_pos)
    vehicle.set_mode(mode="AUTO", drone_id=alaca_id)
    print("Alaca taramaya başlıyor...")

    vehicle.arm_disarm(arm=True, drone_id=feniks_id)
    vehicle.takeoff(alt=feniks_alt, drone_id=feniks_id)
    feniks_takeoff_pos = vehicle.get_pos(drone_id=feniks_id)
    print("Feniks Takeoff yaptı: ", feniks_takeoff_pos)
    vehicle.set_mode(mode="AUTO", drone_id=feniks_id)
    print("Feniks uçuşa başlıyor...")


    feniks_miss_thread = threading.Thread(target=feniks_miss, args=(vehicle, feniks_takeoff_pos, feniks_id))
    alaca_miss_thread = threading.Thread(target=alaca_miss, args=(vehicle, alaca_takeoff_pos, alaca_id))

    feniks_miss_thread.start()
    alaca_miss_thread.start()

    while True:
        if vehicle.on_location(loc=korfez_scan_values2[1], seq=korfez_scan_values2[0], sapma=1, drone_id=korfez_id) or vehicle.get_miss_wp(drone_id=korfez_id) == korfez_scan_values2[0]:
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
