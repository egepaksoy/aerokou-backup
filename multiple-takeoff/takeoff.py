import time
import sys
sys.path.append('../pymavlink_custom')
from pymavlink_custom import Vehicle

if len(sys.argv) != 2:
    exit()
vehicle = Vehicle(sys.argv[1])

ALT = 3

try:
    for drone_id in vehicle.drone_ids:
        vehicle.arm_disarm(arm=True, drone_id=drone_id, force_arm=True)
        vehicle.takeoff(alt=ALT, drone_id=drone_id)
        print(f"{drone_id} kalkış yaptı")

    start_time = time.time()
    miss_time = time.time()
    while True:
        if time.time() - start_time > 2:
            for drone_id in vehicle.drone_ids:
                print(f"{drone_id}: {vehicle.get_pos(drone_id=drone_id)}")
            start_time = time.time()
        
        if time.time() - miss_time > 5:
            print("Görev bitti")
            break

except KeyboardInterrupt:
    print("Kod durduruldu")

except Exception as e:
    print("Hata: ", e)

finally:
    for drone_id in vehicle.drone_ids:
        vehicle.set_mode(mode="LAND", drone_id=drone_id)
    vehicle.vehicle.close()