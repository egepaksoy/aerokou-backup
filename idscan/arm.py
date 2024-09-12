import time
import sys
sys.path.append('../pymavlink_custom')
from pymavlink_custom import Vehicle

vehicle = Vehicle(sys.argv[1])

vehicle.arm_disarm(arm=True, drone_id=1)