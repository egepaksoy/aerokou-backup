import time
import sys
sys.path.append('../pymavlink_custom')
from pymavlink_custom import Vehicle

vehicle = Vehicle(sys.argv[1])

for d_id in vehicle.drone_ids:
    vehicle.set_mode(mode="GUIDED", drone_id=d_id)
    vehicle.arm_disarm(arm=True, drone_id=d_id)
    time.sleep(5)