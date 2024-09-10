import time
import sys
sys.path.append('../pymavlink_custom')
from pymavlink_custom import Vehicle

if len(sys.argv) != 2:
    exit()
vehicle = Vehicle(sys.argv[1])

while True:
    print(vehicle.get_pos(drone_id=1))
    print(vehicle.get_pos(drone_id=2))

vehicle.vehicle.close()