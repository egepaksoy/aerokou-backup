import sys
sys.path.append("./pymavlink_custom")
from pymavlink_custom import Vehicle
import time

vehicle = Vehicle("COM6")

print("idler: ", vehicle.get_vehicle_ids())
vehicle.vehicle.close()