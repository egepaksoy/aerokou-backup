import sys
sys.path.append("./pymavlink_custom/")
from pymavlink_custom import Vehicle
from pymavlink import mavutil
import time

vehicle = Vehicle(address="tcp:127.0.0.1:5762")
lat, lon = -35.36312501, 149.16520965
lat2, lon2 = -35.36317809, 149.16518145
alt = 5
top_birakma_alt = 2
distance_meter = 1
area_meter = 10

try:
    start_time = time.time()
    vehicle.arm_disarm(arm=True)
    vehicle.takeoff(alt=alt)
    takeoff_loc = vehicle.get_pos()
    vehicle.add_mission(seq=0, lat=lat, lon=lon, alt=alt)
    scan_seq, scan_wp = vehicle.scan_area(seq=1, center_lat=lat, center_lon=lon, alt=alt, area_meter=area_meter, distance_meter=distance_meter)
    vehicle.set_mode(mode="AUTO")

    on_mission = True
    on_miss2 = False
    end_mission = False

    while on_mission:
        if time.time() - start_time > 2:
            print("Gidiliyor...")
            print(vehicle.get_mode())
            start_time = time.time()
        
        if not on_miss2 and not end_mission:
            if vehicle.on_location(loc=scan_wp, seq=scan_seq, sapma=1):
                print("Tarama bitti")
                print("Görev 2 basliyor...")
                vehicle.set_mode(mode="GUIDED")
                vehicle.go_to(lat=lat2, lon=lon2, alt=alt)
                print(f"Dron {lat2, lon2} konumuna gidiyor...")
                on_miss2 = True
        
        if on_miss2 and not end_mission:
            if vehicle.on_location(loc=(lat2, lon2), seq=0, sapma=1):
                print("Gorev konumuna gelindi alcaliyor...")
                vehicle.go_to(lat=lat2, lon=lon2, alt=top_birakma_alt)

                start_time = time.time()
                while vehicle.get_pos()[2] >= top_birakma_alt * 1.1:
                    if time.time() - start_time > 1:
                        print("Alcaliyor...")
                        start_time = time.time()
                
                print(f"{top_birakma_alt} Yüksekligine alcalindi top birakiliyor...")
                vehicle.set_servo(pwm=1000)
                time.sleep(1.5)
                vehicle.set_servo(pwm=1750)
                time.sleep(1.5)
                vehicle.set_servo(pwm=0)
                print("Top birakildi yukseliyor...")
                vehicle.go_to(lat=lat2, lon=lon2, alt=alt)

                start_time = time.time()
                while vehicle.get_pos()[2] <= alt * 0.9:
                    if time.time() - start_time > 1:
                        print("Yükseliyor...")
                        start_time = time.time()
                print(f"{alt} Metreye yükseldi RTL aliyor...")
                vehicle.go_to(lat=takeoff_loc[0], lon=takeoff_loc[1], alt=alt)
                on_miss2 = False
                end_mission = True
        
        if end_mission:
            start_time = time.time()
            while not vehicle.on_location(loc=takeoff_loc, seq=0, sapma=1):
                if time.time() - start_time > 2:
                    print("Home konumuna gidiliyor...")
            print("Home konumuna gelindi land aldi")
            vehicle.set_mode(mode="LAND")
            on_mission = False
    
    while vehicle.is_armed():
        print("Iniyor...")
    
    print("Inis tamamlandi")
    vehicle.set_mode("GUIDED")


except KeyboardInterrupt:
    vehicle.set_mode("RTL")
    print("Klavyeden cikildi")

finally:
    vehicle.clear_wp_target()
    vehicle.vehicle.close()