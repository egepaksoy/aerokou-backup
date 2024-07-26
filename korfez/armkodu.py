import time
import sys
sys.path.append("../pymavlink_custom/")
from pymavlink_custom import Vehicle

vehicle = Vehicle("COM7")

try:
    vehicle.set_mode(mode="GUIDED")
    vehicle.arm_disarm(arm=True)
    vehicle.takeoff(alt=3)
    start_time = time.time()
    while True:
        if time.time() - start_time > 2:
            print(f"Arac {vehicle.get_mode()} modunda")
            print(f"Arac {vehicle.is_armed()} durumunda")
            start_time = time.time()


except KeyboardInterrupt:
    print("Koddan çıkıldı.")

finally:
    print("Kod bitti.")