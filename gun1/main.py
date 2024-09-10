import time
import sys
sys.path.append('../pymavlink_custom')
from pymavlink_custom import Vehicle

if len(sys.argv) != 2:
    exit()
vehicle = Vehicle(sys.argv[1])

ALT = 5
lat, lon = 40.7119771, 30.0245517
# lat, lon = -35.36307583, 149.16519825
top_birakma_alt = 2
area_meter = 3
distance_meter = 1

try:
    vehicle.arm_disarm(arm=True)
    vehicle.takeoff(alt=ALT)
    takeoff_loc = vehicle.get_pos()
    print("Kalkış yapıldı, Takeoff konumu: ", takeoff_loc)

    vehicle.add_mission(seq=0, lat=lat, lon=lon, alt=ALT)
    last_seq, last_wp = vehicle.scan_area(seq=1, center_lat=lat, center_lon=lon, alt=ALT, area_meter=area_meter, distance_meter=distance_meter)
    vehicle.set_mode("AUTO")

    start_time = time.time()
    print("Görev başladı")
    on_mission = True

    while on_mission:
        if time.time() - start_time > 2:
            print("Konuma gidiliyor...")
            print("Mod: ", vehicle.get_mode())
            start_time = time.time()

        if vehicle.on_location(loc=last_wp, seq=last_seq, sapma=1):
            print("Top birakma konumuna gelindi")
            vehicle.set_mode("GUIDED")
            vehicle.go_to(lat=lat, lon=lon, alt=top_birakma_alt)
            while vehicle.get_pos()[2] >= top_birakma_alt * 1.1:
                if time.time() - start_time > 2:
                    print("Alcaliyor...")
                    start_time = time.time()
            print("Yangın topları birakiliyor...")
            time.sleep(3)
            print("Yangın topları birakildi RTL aliniyor...")
            vehicle.go_to(lat=lat, lon=lon, alt=ALT)
            while vehicle.get_pos()[2] <= top_birakma_alt * 0.9:
                if time.time() - start_time > 2:
                    print("Yukseliyor...")
                    start_time = time.time()
            
            vehicle.go_to(lat=takeoff_loc[0], lon=takeoff_loc[1], alt=ALT)
            on_mission = False
    
    while True:
        if time.time() - start_time > 2:
            print("Home konumuna donuluyor...")
            start_time = time.time()

        if vehicle.on_location(loc=takeoff_loc, seq=0, sapma=1):
            print("Home konumuna donuldu")
            vehicle.set_mode("LAND")
            break

except Exception as e:
    print("Hata cikti: ", e)
    vehicle.set_mode("RTL")

except KeyboardInterrupt:
    print("CTRL+C ile cikildi")
    vehicle.set_mode("RTL")

finally:
    vehicle.clear_wp_target()
    vehicle.vehicle.close()