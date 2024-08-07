#! ALACA
import time
import json
import math
import sys
sys.path.append("../pymavlink_custom/")
from pymavlink_custom import Vehicle, calc_hipo_angle

screen_rat = (32, 24)

ALT = 5
top_birakma_alt = 3

servo_birakma_suresi = 2

ates_konumu_lat, ates_konumu_lon = 
scan_last_waypoint = ()
takeoff_konumu = ()

alaca_id = 3

ates_seq = 
scan_last_seq = 
donus_wp_seq = 

try:
    vehicle = Vehicle("/dev/ttyACM0", drone_id=alaca_id)
    print("Takeoff alınıyor..")

    print(f"Drone arm edildi ve modu: GUIDED\nTakeoff alınıyor...")
    start_time = time.time()
    while current_alt < ALT * 0.95:
        current_alt = vehicle.get_pos()[2]
        if time.time() - start_time > 2:
            print(f"Anlık irtifa: {current_alt} metre")
            start_time = time.time()
    print(f"{ALT} metreye yükseldi")

    print("Takeoff alindi")
    print("Takeoff konumu: ", takeoff_konumu)

    for i in range(7):
        print(f"Waypoint {i} ekleniyor...")
    print("Waypointlar eklendi")

    on_mission = True
    on_miss2 = False
    start_time = time.time()

    termal_time = time.time()
    while on_mission:
        if time.time() - start_time > 2:
            print("Gidiliyor...")
            print(f"Mevcut waypoint: {vehicle.get_miss_wp()}\n")
            start_time = time.time()

        if on_miss2 == False:
            ############ KONUM HESAPLAMA KISMI ###########
            if time.time() - termal_time > 10:
                print(f"Ates konumu bulundu: {(ates_konumu_lat, ates_konumu_lon)}")
                termal_time = time.time()
            ############ KONUM HESAPLAMA KISMI ###########

            ############## KONUMA GITME KISMI ############
            if vehicle.on_location(loc=scan_last_waypoint, seq=scan_last_seq, sapma=1):
                print(f"Ates algilandi\n{(ates_konumu_lat, ates_konumu_lon)} konumunda")
                print("Tarama bitti")
                print("Ates konumlari elde edildi")
                with open("./miss-file.txt", "w+") as miss_file:
                    miss_file.write("end-mission")
                on_miss2 = True
                print("Waypoint 8 ekleniyor...")
                print("Konum elde edildi gidilcek")
            ############## KONUMA GITME KISMI ############

        if on_miss2 == True:
            if vehicle.on_location(loc=(ates_konumu_lat, ates_konumu_lon), seq=ates_seq, sapma=1):
                print("Ates konumuna gelindi")
                print("Waypoint 9 ekleniyor...")
                print("Alcaliyor...")
                start_time = time.time()
                termal_time = time.time()
                while vehicle.get_pos()[2] > top_birakma_alt * 1.05 or time.time() - termal_time < 5:
                    if time.time() - start_time > 2:
                        print("Alcaliyor...")
                        start_time = time.time()

                print("Alcalindi top birakiliyor")
                time.sleep(servo_birakma_suresi)
                print("Top birakildi takeoff konumuna gidiliyor")
                print("Waypoint 10 ekleniyor...")
                print("Waypoint 11 ekleniyor...")
                on_mission = False

    start_time = time.time()
    while vehicle.on_location(seq=donus_wp_seq, loc=takeoff_konumu, sapma=1) == False:
        if time.time() - start_time > 2:
            print("Takeoff konumuna donuluyor...")
            start_time = time.time()

    print("Takeoff konumuna donuldu")
    print("LAND aliniyor...")

except KeyboardInterrupt:
    print("Takeoff konumuna donuldu")
    print("LAND aliniyor...")

finally:
    print("Gorev tamamlandi")
    vehicle.vehicle.close()
