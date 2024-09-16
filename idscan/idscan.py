import time
import sys
sys.path.append('../pymavlink_custom')
from pymavlink_custom import Vehicle

vehicle = Vehicle(sys.argv[1])

start_time = time.time()

try:
    while True:
        print(1)
        for i in vehicle.drone_ids:
            if vehicle.get_mode(drone_id=i) != "GUIDED":
                vehicle.set_mode(mode="GUIDED", drone_id=i)
            else:
                vehicle.set_mode(mode="STABILIZE", drone_id=i)
            print(f"{i}: {vehicle.get_mode(drone_id=i)}")
            print(f"{i}: {vehicle.get_pos(drone_id=i)}")
except KeyboardInterrupt:
    print("Exiting...")

except Exception as e:
    print(e)

finally:
    vehicle.vehicle.close()