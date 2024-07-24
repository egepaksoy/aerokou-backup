import time
import json
import math
import sys
sys.path.append("../pymavlink_custom/")
from pymavlink_custom import Vehicle, calc_hipo_angle

ALT = 5
lat = 40.7126653
lon = 30.0259418

screen_rat = (640, 480)


vehicle = Vehicle("/dev/ttyACM0")

vehicle.takeoff_mod(alt=ALT, mod="AUTO")

try:
    vehicle.add_mission(0, lat=lat, lon=lon, alt=ALT)
    vehicle.add_mission(1, lat=lat, lon=lon, alt=ALT)

    wpler = vehicle.get_wp_list

    print(len(wpler), " waypoint bulundu")
    for i, waypoint in enumerate(wpler):
        print(f"Waypiont {i}, pozisyonu: {waypoint}")

    on_mission = True
    on_miss2 = False
    start_time = time.time()
    loc = []
    pos = []

    while on_mission:
        if time.time() - start_time > 2:
            print("Gidiliyor...")
            print(f"Current waypoint: {vehicle.get_miss_wp()}\n")
            start_time = time.time()

        if on_miss2 == False:
            # Goruntu algılama ile posterin konumunu elde etme
            with open("./konum.txt", "r") as loc_file:
                for t in loc_file:
                    line = t.strip().split(",")
                    line = (int(line[0]), int(line[1]))

                    if line not in loc:
                        loc.append(line)
                        drone_pos = vehicle.get_pos()
                        hipo, angle = calc_hipo_angle(screen_rat_x_y=screen_rat, x_y=(int(line[0]), int(line[1])), alt=drone_pos[2], yaw=vehicle.get_yaw(), alt_met=screen_rat[0])
                        location = (drone_pos[0] + (int(hipo)*math.sin(math.radians(angle)))*vehicle.DEG, drone_pos[1] + (int(hipo)*math.cos(math.radians(angle)))*vehicle.DEG)
                        a = 0
                        for l in pos:
                            if abs(location[0] - l[0]) <= vehicle.DEG/4 and abs(location[1] - l[1]) <= vehicleDEG/4:
                                a = 1
                        if a == 0:
                            pos.append(location)
                            print(f"Ates algilandi\n{location} konumunda")
        
        if on_miss2 == False:
            if abs(vehicle.get_pos()[0] - lat) <= vehicle.DEG/2 and abs(vehicle.get_pos()[1] - lon) <= vehicle.DEG/2:
                print("Konuma gelindi")
                if len(pos) == 0:
                    print("Ates bulunamadı")
                    print("Gorev iptal edildi")
                    on_mission = False
                
                else:
                    on_miss2 = True
                    print("Atese gidiliyor...")
                    seq = 2
                    for loc in pos:
                        vehicle.add_mission(seq, loc[0], loc[1], ALT)
                        last_seq = seq
                        last_pos = loc
                        seq += 1

                    print("Waypointler eklendi atese gidiliyor...")
        
        if on_miss2 == True:
            if vehicle.get_miss_wp() == last_seq and abs(vehicle.get_pos()[0] - last_pos[0]) <= vehicle.DEG/2 and abs(vehicle.get_pos()[1] - last_pos[1]) <= vehicle.DEG/2:
                print("Atese gelindi")
                print("Gorev bitti")
                vehicle.set_mode("RTL")
                print("RTL moduna alındı")
                on_mission = False


except KeyboardInterrupt:
    print("Koddan çıkıldı.")
    vehicle.set_mode("RTL")