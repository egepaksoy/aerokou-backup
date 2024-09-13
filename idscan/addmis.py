import time
import sys
sys.path.append('../pymavlink_custom')
from pymavlink_custom import Vehicle

vehicle = Vehicle(sys.argv[1])

start_time = time.time()

for i in range(0,10):
    print(vehicle.add_mission(seq=i, lat=40.7121187, lon=30.0245113, alt=5, drone_id=2))