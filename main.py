import sys
sys.path.append("./pymavlink_custom")
from pymavlink_custom import Vehicle, calc_hipo_angle
import time
import math


try:
    vehicle = Vehicle("COM6")
    #? DRONE_IDLER
    alaca_id = 2
    korfez_id = 1
    feniks_id = 3

    #? KAMERA ORANLARI
    korfez_kamera_oran = (640, 480)
    termal_kamera_oran = (32, 24)

    #? KONTROL DEGISKENLERI
    on_mission = True
    #######################
    alaca_mission = False
    #######################
    korfez_mission = False
    #######################
    feniks_mission = True
    feniks_alcaliyor = False
    #######################
    first_arm = True

    #? ZAMAN DEGISKENLERI
    feniks_time = korfez_time = alaca_time = time.time()

    #? IRTIFALAR
    feniks_alt = 5
    alaca_alt = 5
    top_birakma_alt = 3

    #? KONUMLAR
    poster_pozisyonlar = []
    poster_konumlar = [[40.7114525, 30.0246350]]
    poster_konumu = [40.7114525, 30.0246350]
    ##########################
    ates_konumlar = []
    ates_pozisyonlar = []
    ates_sicakliklar = []
    ##########################
    feniks_takeoff = []
    alaca_takeoff = []
    ##########################

    #? TARAMA DEGISKENLERI
    distance_meter = 1
    scan_area = 5

    with open("./gorev_dosyaları/mission_file.txt", "w+") as miss_file:
        miss_file.close()

    while on_mission:
        if korfez_mission:
            if time.time() - korfez_time > 2:
                print("Ates posteri bulunuyor...")
                korfez_time = time.time()

            ################# POSTER KONUMU HESAPLAMA ###############
            with open("./gorev_dosyaları/korfez_konum.txt", "r") as poster_file:
                for file_line in poster_file:
                    konum = list(map(float, file_line.strip().split(",")))
                    if konum not in poster_pozisyonlar:
                        poster_pozisyonlar.append(konum)
                        drone_konum = vehicle.get_pos(drone_id=korfez_id)
                        uzaklik, aci = calc_hipo_angle(screen_rat_x_y=korfez_kamera_oran, x_y=konum, alt=drone_konum[2]\
                            , yaw=vehicle.get_yaw(drone_id=korfez_id), alt_met=korfez_kamera_oran[0])
                        poster_konum = [drone_konum[0] + (uzaklik*math.cos(math.radians(aci)))*vehicle.DEG, drone_konum[1] + (uzaklik*math.sin(math.radians(aci)))*vehicle.DEG]
                        poster_konumlar.append(poster_konum)
                        print("Poster konumu bulundu: ", poster_konum)
                        korfez_mission = False
                        feniks_mission = False
                        alaca_mission = False
                        with open("./gorev_dosyaları/mission_file.txt", "a") as miss_file:
                            miss_file.write("korfez=False\n")


        if feniks_mission:
            if first_arm:
                # if len(poster_konumlar) == 0:
                    # print("Poster konumlari cekilemedi gorev iptal edildi")
                    # exit()
                print("Feniks goreve basliyor...")
                feniks_takeoff = vehicle.get_pos(drone_id=feniks_id)
                if not feniks_takeoff:
                    print("Feniks takeoff konumu alınamadi gorev iptal")
                    exit()
                print("Feniks takeoff konumu: ", feniks_takeoff)
                vehicle.takeoff_mode(mode="GUIDED", alt=feniks_alt, drone_id=feniks_id)
                print("Feniks takeoff aldi")
                poster_konumu = poster_konumlar[0]
                vehicle.add_mission(seq=0, lat=poster_konumu[0], lon=poster_konumu[1], alt=feniks_alt, drone_id=feniks_id)
                vehicle.add_mission(seq=1, lat=poster_konumu[0], lon=poster_konumu[1], alt=feniks_alt, drone_id=feniks_id)

                wpler = vehicle.get_wp_list(drone_id=feniks_id)

                print(len(wpler), " waypoint bulundu")
                for i, waypoint in enumerate(wpler):
                    print(f"Waypiont {i}, pozisyonu: {waypoint}")
                
                print("vehicle id: ", vehicle.vehicle.target_system)

                vehicle.set_mode(mode="AUTO", drone_id=feniks_id)
                print("Feniks konumlari elde etti gorev basliyor...")
                first_arm = False

            if time.time() - feniks_time > 2:
                print("Feniks konuma gidiyor...") 
                feniks_time = time.time()
            
            # eger feniks konumdaysa
            if len(poster_konumu) != 0:
                if vehicle.on_location(loc=poster_konumu, seq=1, drone_id=feniks_id):
                    # feniks konuma ilk gelisiyse
                    if feniks_alcaliyor == False:
                        print(f"Feniks konuma geldi {top_birakma_alt} metreye feniks_alcaliyor...")
                        vehicle.add_mission(seq=2, lat=poster_konumu[0], lon=poster_konumu[1], alt=top_birakma_alt, drone_id=feniks_id)
                        feniks_alcaliyor = True

                    # feniks topu birakma yuksekligindeyse
                    if vehicle.get_pos(drone_id=feniks_id)[2] <= top_birakma_alt * 1.05 and vehicle.get_pos(drone_id=feniks_id)[2] >= top_birakma_alt * 0.95:
                        print(f"Feniks {top_birakma_alt} mt'ye indirildi")
                        vehicle.set_servo(drone_id=feniks_id, pvm=1750)
                        time.sleep(1.5)
                        vehicle.set_servo(drone_id=feniks_id, pvm=1000)
                        time.sleep(1.5)
                        vehicle.set_servo(drone_id=feniks_id, pvm=0)
                        print("Feniks top birakti")
                        vehicle.add_mission(seq=3, lat=poster_konumu[0], lon=poster_konumlar[1], alt=feniks_alt, drone_id=feniks_id)
                        vehicle.add_mission(seq=4, lat=feniks_takeoff[0], lon=feniks_takeoff[1], alt=feniks_alt, drone_id=feniks_id)
                        print("Feniks kalkis konumuna gidiyor...")
                        feniks_mission = False
                        feniks_alcaliyor = False
        
        if feniks_mission == False and len(feniks_takeoff) != 0:
            if vehicle.on_location(loc=(feniks_takeoff[0], feniks_takeoff[1]), seq=4, drone_id=feniks_id):
                vehicle.set_mode("LAND", drone_id=feniks_id)
                print("Feniks kalkis konumuna geldi LAND aliyor...")

except KeyboardInterrupt:
    print("Klavyeden cikildi")

finally:
    print("Bitti")

print(poster_konumlar)

#! listleler tupple olmamasına dikkat et
#! konum hesaplamada cos sin'e dikkat et
#! posterin koseleri koordinatini cek
#! posterin boyutunu gercek boyutuyla karsilastir ona gore hesapla

#!!!!! lat=dikey lon=yatay