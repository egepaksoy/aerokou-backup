import time
import sys
sys.path.append('../pymavlink_custom')
from pymavlink_custom import Vehicle

vehicle = Vehicle("COM7")

ALT = 5
lat = 40.711
lon = 30.025

try:
    vehicle.takeoff_mode(alt=ALT, mode="GUIDED")
    vehicle.add_mission(seq=0, lat=lat, lon=lon, alt=ALT)
    vehicle.add_mission(seq=1, lat=lat, lon=lon, alt=ALT)

    start_time = time.time()
    print("Görev başladı")
    on_mission = True

    while on_mission:
        if time.time() - start_time > 2:
            print("Konuma gidiliyor...")
            start_time = time.time()

        if vehicle.on_location(loc=(lat, lon), seq=1, sapma=2):
            print("Konuma gelindi top birakiliyor...")
            vehicle.set_servo()
            time.sleep(2)
            print("Yangın topları birakildi RTL aliniyor...")
            vehicle.set_mode("RTL")
            on_mission = False

except KeyboardInterrupt:
    print("CTRL+C ile cikildi")
    vehicle.set_mode("RTL")

finally:
    print("Islem tamamlandi")
    print("MÖÖÖÖÖÖÖÖÖÖÖÖÖÖÖÖÖ")