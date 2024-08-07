import sys
sys.path.append("./pymavlink_custom")
from pymavlink_custom import Vehicle
import time
import os

vehicle = Vehicle(address="COM12", drone_id=1)

try:
    start_time = time.time()
    while True:
        if time.time() - start_time > 2:
            print("pos: ", vehicle.get_pos())
            print("yaw: ", vehicle.get_yaw())
            start_time = time.time()
except KeyboardInterrupt:
    print("Exit")
finally:
    vehicle.vehicle.close()