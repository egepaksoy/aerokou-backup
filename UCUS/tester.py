import time
import sys
import threading
sys.path.append('../pymavlink_custom')
from pymavlink_custom import Vehicle

vehicle = Vehicle("tcp:127.0.0.1:5762")

tarama_wpler = vehicle.scan_area_wpler(center_lat=-35.36305187, center_lon=149.16517342,alt=10, area_meter=10, distance_meter=2)

vehicle.send_all_waypoints(wp_list=tarama_wpler, drone_id=1)
vehicle.set_mode(mode="GUIDED", drone_id=1)
vehicle.arm_disarm(arm=True, drone_id=1)
vehicle.takeoff(alt=10, drone_id=1)

vehicle.set_mode(mode="AUTO", drone_id=1)
while vehicle.on_location(loc=tarama_wpler[len(tarama_wpler)-1], seq=len(tarama_wpler)-1, sapma=1, drone_id=1) == False:
    time.sleep(0.1)

vehicle.set_mode(mode="RTL", drone_id=1)