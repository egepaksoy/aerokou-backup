import time
import sys
sys.path.append('../pymavlink_custom')
from pymavlink_custom import Vehicle

vehicle = Vehicle(sys.argv[1])

vehicle.arm_disarm(arm=True, drone_id=3)
vehicle.takeoff(alt=3, drone_id=3)
vehicle.set_mode(mode="LAND", drone_id=3)