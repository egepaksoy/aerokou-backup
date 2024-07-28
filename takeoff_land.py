import sys
sys.path.append("./pymavlink_custom")
from pymavlink_custom import Vehicle
import time

try:
    vehicle = Vehicle("COM6")
    vehicle.clear_wp_target(drone_id=3)

    vehicle.takeoff_mode(mode="GUIDED", drone_id=3, alt=3)
    vehicle.set_mode(mode="AUTO", drone_id=3)
    time.sleep(2)
    print("mode: ", vehicle.get_mode(drone_id=3))
    vehicle.set_mode(mode="LAND", drone_id=3)
    time.sleep(1)
    print("mode: ", vehicle.get_mode(drone_id=3))
    vehicle.arm_disarm(arm=False, drone_id=3)

except KeyboardInterrupt:
    print("Klavyeden cikildi")

finally:
    print("Bitti")
    vehicle.vehicle.close()