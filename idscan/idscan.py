import time
import sys
sys.path.append('../pymavlink_custom')
from pymavlink_custom import Vehicle

vehicle = Vehicle(sys.argv[1])

print(vehicle.get_all_drone_ids())

start_time = time.time()

while True:
    print(vehicle.get_all_drone_ids())
    for i in vehicle.drone_ids:
        print(f"{i}: ",vehicle.get_pos(drone_id=i))
        print(f"{i}: {vehicle.get_mode(drone_id=i)}")
