#! GIBI
import time

poster_konumu = ()

try:
    on_mission = True
    ates_algila = True
    start_time = time.time()

    while on_mission:
        if time.time() - start_time > 2:
            print("Gorev Devam Ediyor...")
            start_time = time.time()

        if ates_algila:
            ############ KONUM HESAPLAMA KISMI ###########
            with open("./konum.txt", "r") as loc_file:
                for t in loc_file:
                    if t:
                        print(f"Ates algilandi\n{poster_konumu} konumunda")
                        with open("./poster_konum.txt", "a") as pos_file:
                            pos_file.write(f"{location[0]},{location[1]}\n")
                        
                        with open("./miss-file.txt", "w+") as miss_file:
                            miss_file.write("end-mission")
            ############ KONUM HESAPLAMA KISMI ###########
            ates_algila = False

finally:
    print("Gorev Tamamlandı atesin konumları: ", poster_konumu)