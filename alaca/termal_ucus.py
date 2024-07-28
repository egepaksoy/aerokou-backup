#! RASPBERRY
import time
import json
import math
import sys
sys.path.append("../pymavlink_custom/")
from pymavlink_custom import Vehicle, calc_hipo_angle

ALT = 5
lat = 40.7119890
lon = 30.0245963

screen_rat = (32, 24)

vehicle = Vehicle("COM7", drone_id=1)

try:
    print("Takeoff alınıyor..")
    vehicle.takeoff_mode(alt=ALT, mode="GUIDED")
    print("Takeoff alindi")
    vehicle.set_mode("AUTO")
    vehicle.add_mission(seq=0, lat=lat, lon=lon, alt=ALT)
    vehicle.add_mission(seq=1, lat=lat, lon=lon, alt=ALT)
    scan_last_seq, scan_last_waypoint = vehicle.scan_area(seq=2, center_lat=lat, center_lon=lon, alt=ALT, area_meter=5, distance_meter=1)

    wpler = vehicle.get_wp_list()

    print(len(wpler), " waypoint bulundu")
    for i, waypoint in enumerate(wpler):
        print(f"Waypiont {i}, pozisyonu: {waypoint}")

    on_mission = True
    on_miss2 = False
    start_time = time.time()
    fire_poses = []
    pos = []

    while on_mission:
        if time.time() - start_time > 2:
            print("Gidiliyor...")
            print(f"Mevcut waypoint: {vehicle.get_miss_wp()}\n")
            start_time = time.time()

        if on_miss2 == False:
            ############ KONUM HESAPLAMA KISMI ###########
            with open("./konum.txt", "r") as loc_file:
                for t in loc_file:
                    file_line = t.strip().split(",")
                    fire_pos = (float(file_line[0]), float(file_line[1]))

                    if fire_pos not in fire_poses:
                        fire_poses.append(fire_pos)
                        drone_pos = vehicle.get_pos()
                        hipo, angle = calc_hipo_angle(screen_rat_x_y=screen_rat, x_y=(fire_pos[0], fire_pos[1]), alt=drone_pos[2], yaw=vehicle.get_yaw(), alt_met=screen_rat[0])
                        location = (drone_pos[0] + (hipo*math.sin(math.radians(angle)))*vehicle.DEG, drone_pos[1] + (hipo*math.cos(math.radians(angle)))*vehicle.DEG)

                        pos.append(location)
                        print(f"Ates algilandi\n{location} konumunda")
            ############ KONUM HESAPLAMA KISMI ###########
        
            ############## KONUMA GITME KISMI ############
            if vehicle.on_location(loc=scan_last_waypoint, seq=scan_last_seq):
                print("Tarama bitti")
                print("Ates konumlari elde edildi")
                vehicle.add_mission(seq=scan_last_seq+1, lat=lat, lon=lon, alt=ALT)
                print("Son konum girildi")
                on_miss2 = True
                '''
                on_miss2 = True
                
                if len(pos) != 0:
                    seq = scan_last_seq + 1
                    for loc in pos:
                        vehicle.add_mission(seq=seq, lat=loc[0], lon=loc[1], alt=ALT)
                        last_seq = seq
                        last_loc = loc
                        seq += 1
                    vehicle.add_mission(seq=seq, lat=lat, lon=lon, alt=ALT)
                    print("Ates konumları girildi")
                
                else:
                    print("Ates konumları bulunmadı")
                    on_mission = False
                '''
            ############## KONUMA GITME KISMI ############

        if on_miss2 == True:
            if vehicle.on_location(loc=(lat, lon), seq=scan_last_seq + 1):
                print("RTL aliniyor...")
                vehicle.set_mode("RTL")
                on_mission = False
        
        '''
        if on_miss2 == True:
            for i, loc in enumerate(pos):
                if vehicle.on_location(loc=loc, seq=scan_last_seq + 1 + i):
                    print(f"ates {i+1}'e gelindi")

            if vehicle.on_location(loc=last_loc, seq=last_seq):
                print("Tum ateslere gelindi")
        
            if vehicle.on_location(loc=(lat, lon), seq=seq):
                print("Gorev bitti")
                print("RTL Alındı...")
                vehicle.set_mode("RTL")
                on_mission = False
        '''

except KeyboardInterrupt:
    print("Koddan çıkıldı.")
    vehicle.set_mode("RTL")

print("Gorev Tamamlandı atesin konumları: ", pos)
print("MÖÖÖÖÖÖÖÖÖÖÖÖÖÖÖÖÖÖÖÖÖÖÖÖÖ")