import time
import sys
sys.path.append('../pymavlink_custom')
from pymavlink_custom import Vehicle

if len(sys.argv) != 2:
    exit()
vehicle = Vehicle(sys.argv[1])

DRONE2 = 2

try:
    print("fd")
    while True:
        print("2: ",vehicle.vehicle.recv_match())

except KeyboardInterrupt:
    print("Cikildi")

finally:
    vehicle.vehicle.close()