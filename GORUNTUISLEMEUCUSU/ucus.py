import time
import sys
import threading
sys.path.append('../pymavlink_custom')
from pymavlink_custom import Vehicle, calc_hipo_angle, calc_location, get_pixel_pos

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

if len(sys.argv) != 2:
    print("Bağlantı yok")
    exit()

vehicle = Vehicle(sys.argv[1])

running = True

loc = (40.7120092, 30.0245854)
ates_pos = []

area_meter = 5
distance_meter = 1

alt = 9
top_birakma_alt = 6

drone_id = 1

try:
    waypoints = [[loc[0], loc[1], alt]]
    waypoints += vehicle.scan_area_wpler(center_lat=loc[0], center_lon=loc[1], alt=alt, area_meter=area_meter, distance_meter=distance_meter)
    vehicle.send_all_waypoints(wp_list=waypoints, drone_id=drone_id)

    vehicle.set_mode(mode="GUIDED", drone_id=drone_id)
    vehicle.arm_disarm(arm=True, drone_id=drone_id)
    vehicle.takeoff(alt=alt, drone_id=drone_id)
    takeoff_pos = vehicle.get_pos(drone_id=drone_id)
    print("Kalkış tamamlandı")

    vehicle.set_mode(mode="AUTO", drone_id=drone_id)
    print("Görev Başladı")

    start_time = time.time()
    current_wp = vehicle.get_miss_wp(drone_id=drone_id)
    while True:
        if time.time() - start_time > 2:
            print("Görev devam ediyor...")
            print("mode: ", vehicle.get_mode(drone_id=drone_id))
            print("wp: ", vehicle.get_miss_wp(drone_id=drone_id))
            start_time = time.time()
        
        if vehicle.get_miss_wp(drone_id=drone_id) != current_wp:
            print("Görev devam ediyor...")
            print("mode: ", vehicle.get_mode(drone_id=drone_id))
            print("wp: ", vehicle.get_miss_wp(drone_id=drone_id))
            start_time = time.time()
            current_wp = vehicle.get_miss_wp(drone_id=drone_id)

        with open("./gorev_dosyaları/korfez_konum.txt", "r") as f:
            drone_pos = vehicle.get_pos(drone_id=drone_id)
            drone_yaw = vehicle.get_yaw(drone_id=drone_id)
            for line in f:
                pixel_pos = get_pixel_pos(line)
                uzaklik, aci = calc_hipo_angle(screen_rat_x_y=(640, 480), x_y=pixel_pos, alt=drone_pos[2], yaw=drone_yaw, alt_met=640)
                ates_pos = calc_location(uzaklik=uzaklik, aci=aci, lat=drone_pos[0], lon=drone_pos[1])
                print("Ateş konumu çekildi: ", ates_pos)

            if len(ates_pos) != 0:
                break
        
        if vehicle.on_location(loc=waypoints[len(waypoints) - 1], seq=len(waypoints) - 1, sapma=1, drone_id=drone_id):
            if len(ates_pos) == 0:
                print("Ates pozisyonu bulunamadı RTL alıyor...")
                failsafe(vehicle)
                exit()

            print("Ateş konumu bulundu: ", ates_pos)
            break
    
    while True:
        if time.time() - start_time > 2:
            print("Görev devam ediyor...")
            print("mode: ", vehicle.get_mode(drone_id=drone_id))
            print("wp: ", vehicle.get_miss_wp(drone_id=drone_id))
            start_time = time.time()
        
        if vehicle.get_miss_wp(drone_id=drone_id) != current_wp:
            print("Görev devam ediyor...")
            print("mode: ", vehicle.get_mode(drone_id=drone_id))
            print("wp: ", vehicle.get_miss_wp(drone_id=drone_id))
            start_time = time.time()
            current_wp = vehicle.get_miss_wp(drone_id=drone_id)

        if vehicle.on_location(loc=waypoints[len(waypoints) - 1], seq=len(waypoints) - 1, sapma=1, drone_id=drone_id):
            if len(ates_pos) == 0:
                print("Ates pozisyonu bulunamadı RTL alıyor...")
                failsafe(vehicle)
                exit()

            print("Ateş konumu bulundu: ", ates_pos)
            break
    
    vehicle.set_mode(mode="GUIDED", drone_id=drone_id)
    vehicle.go_to(lat=ates_pos[0], lon=ates_pos[1], alt=alt, drone_id=drone_id)

    while vehicle.on_location(loc=ates_pos, seq=0, sapma=1, drone_id=drone_id) == False:
        continue

    print("Top bırakılıyor...")
    vehicle.go_to(lat=ates_pos[0], lon=ates_pos[1], alt=top_birakma_alt, drone_id=drone_id)

    while vehicle.get_pos(drone_id=drone_id)[2] > top_birakma_alt * 1.1:
        continue

    vehicle.set_servo(channel=9, pwm=1000, drone_id=drone_id)
    time.sleep(1.5)
    vehicle.set_servo(channel=9, pwm=1750, drone_id=drone_id)
    time.sleep(1.5)
    vehicle.set_servo(channel=9, pwm=0, drone_id=drone_id)

    print("Toplar bırakıldı dönülüyor...")

    vehicle.go_to(lat=ates_pos[0], lon=ates_pos[1], alt=alt, drone_id=drone_id)

    while vehicle.get_pos(drone_id=drone_id)[2] < alt * 0.9:
        continue

    vehicle.go_to(lat=takeoff_pos[0], lon=takeoff_pos[1], alt=alt, drone_id=drone_id)

    start_time = time.time()
    while True:
        if time.time() - start_time > 2:
            print("Dönülüyor...")
            start_time = time.time()

        if vehicle.on_location(loc=takeoff_pos, seq=0, sapma=1, drone_id=drone_id):
            print("Konuma dönüldü")
            break
    
    vehicle.set_mode(mode="LAND", drone_id=drone_id)
    print("Land alındı görev bitti")



except KeyboardInterrupt:
    running = False
    failsafe(vehicle)
    print("Programdan çıkıldı")

except Exception as e:
    print("Hata: ", e)
    running = False
    failsafe(vehicle)

finally:
    running = False
    vehicle.vehicle.close()
    print("Programdan çıkıldı")