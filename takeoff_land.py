import sys
sys.path.append("./pymavlink_custom/")
from pymavlink_custom import Vehicle
import time

vehicle = Vehicle(address="tcp:127.0.0.1:5762")
lat, lon = -35.36317362, 149.16516718
lat2, lon2 = -35.36318787, 149.16528598
alt = 3

try:
    vehicle.request_message_interval(message_input="VFR_HUD", frequency_hz=4, drone_ids=[1])
    msg = vehicle.vehicle.recv_match(type="VFR_HUD", blocking=True)
    if msg is not None and msg.get_srcSystem() == 1:
        speed = msg.groundspeed
        print(f"Speed: {speed}")
    vehicle.arm_disarm(arm=True)
    vehicle.takeoff(alt=alt)
    vehicle.add_mission(0, lat, lon, alt)
    vehicle.add_mission(1, lat, lon, alt)
    last_seq, last_wp = vehicle.scan_area(seq=2, center_lat=lat, center_lon=lon, alt=alt, area_meter=10, distance_meter=1)
    vehicle.set_mode("AUTO")

    wpler = vehicle.get_wp_list()
    print(f"{len(wpler)} adet waypoint var")

    start_time = time.time()
    while True:
        if time.time() - start_time > 2:
            msg = vehicle.vehicle.recv_match(type="VFR_HUD", blocking=True)
            if msg is not None and msg.get_srcSystem() == 1:
                speed = msg.groundspeed
                print(f"Speed: {speed}")
            print(vehicle.get_mode())
            print(vehicle.get_all_drone_ids())
            start_time = time.time()
        
        if vehicle.on_location(loc=(lat,lon), seq=1, sapma=1):
            print("Konuma geldi tarama basliyor...")
            vehicle.set_mode("RTL")
        
        if vehicle.on_location(loc=(last_wp), seq=last_seq, sapma=1):
            print("Tarama bitti ates konumuna gidiliyor...")
            vehicle.set_mode("GUIDED")
            vehicle.go_to(lat=lat2, lon=lon2, alt=alt)
            
        if vehicle.on_location(loc=(lat2,lon2), seq=0, sapma=1) and vehicle.get_mode() == "GUIDED":
            print("Ates konumuna ulasildi...")
            print("RTL Aliniyor...")
            vehicle.set_mode("RTL")


except KeyboardInterrupt:
    vehicle.set_mode(mode="RTL")
    print("Klavyeden cikildi")

finally:
    print("Bitti")
    vehicle.clear_wp_target()
    vehicle.vehicle.close()