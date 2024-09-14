import time
import sys
sys.path.append('../pymavlink_custom')
from pymavlink_custom import Vehicle

vehicle = Vehicle(sys.argv[1])

try:
    for d_id in vehicle.drone_ids:
        vehicle.set_mode(mode="GUIDED", drone_id=d_id)
        vehicle.arm_disarm(arm=True, drone_id=d_id)
        vehicle.takeoff(alt=6, drone_id=d_id)
        time.sleep(2)

except KeyboardInterrupt:
    print("Cikildi")

except Exception as e:
    print("Hata: ",e)

finally:
    print("Bitti")
    for d_id in vehicle.drone_ids:
        vehicle.set_mode(mode="RTL", drone_id=d_id)