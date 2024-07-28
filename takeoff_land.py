import sys
sys.path.append("./pymavlink_custom")
from pymavlink_custom import Vehicle
import time

try:
    vehicle = Vehicle("COM6")
    vehicle.arm_disarm(arm=True, drone_id=3)
    vehicle.arm_disarm(arm=False, drone_id=3)


except KeyboardInterrupt:
    print("Klavyeden cikildi")

finally:
    print("Bitti")
    vehicle.vehicle.close()