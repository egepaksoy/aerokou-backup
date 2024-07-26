#! GIBI
import time
import json
import math
import sys
sys.path.append("../pymavlink_custom/")
from pymavlink_custom import Vehicle, calc_hipo_angle

screen_rat = (640, 480)


vehicle = Vehicle("COM7", drone_id=1)

try:
    with open("./poster_konum.txt", "w+") as pos_file:
        pos_file.close()
    on_mission = True
    start_time = time.time()
    loc = []
    pos = []

    while on_mission:
        if time.time() - start_time > 2:
            print("Gorev Devam Ediyor...")
            print(f"Pozisyon: {vehicle.get_pos()}\n")
            start_time = time.time()

        ############ KONUM HESAPLAMA KISMI ###########
        with open("./konum.txt", "r") as loc_file:
            for t in loc_file:
                file_line = t.strip().split(",")
                line = (float(file_line[0]), float(file_line[1]))

                if line not in loc:
                    loc.append(line)
                    drone_pos = vehicle.get_pos()
                    hipo, angle = calc_hipo_angle(screen_rat_x_y=screen_rat, x_y=(float(line[0]), float(line[1])), alt=drone_pos[2], yaw=vehicle.get_yaw(), alt_met=screen_rat[0])
                    location = (drone_pos[0] + (float(hipo)*math.sin(math.radians(angle)))*vehicle.DEG, drone_pos[1] + (float(hipo)*math.cos(math.radians(angle)))*vehicle.DEG)
                    pos.append(location)
                    print(f"Ates algilandi\n{location} konumunda")
                    with open("./poster_konum.txt", "a") as pos_file:
                        pos_file.write(f"{location[0]},{location[1]}\n")
        ############ KONUM HESAPLAMA KISMI ###########
        
except KeyboardInterrupt:
    print("Koddan çıkıldı.")

print("Gorev Tamamlandı atesin konumları: ", pos)
print("MÖÖÖÖÖÖÖÖÖÖÖÖÖÖÖÖÖÖÖÖÖÖÖÖÖ")

#! POSTER KONUMU LONG YANLIS ALIYOR