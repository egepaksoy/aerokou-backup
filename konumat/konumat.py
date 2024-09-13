import time
import sys
import threading
sys.path.append('../pymavlink_custom')
from pymavlink_custom import Vehicle

#vehicle = Vehicle("tcp:127.0.0.1:5762")
vehicle = Vehicle("com7")

korfez_pos = (40.7119738, 30.0245555)
korfez_pos2 = (30.7119738, 40.0245555)
korfez_alt=5
korfez_scan_meter2 = 10
korfez_distance_meter2 = 2
korfez_scan_meter1 = 5
korfez_distance_meter1 = 1

korfez_missions1 = vehicle.scan_area_wpler(center_lat=korfez_pos[0], center_lon=korfez_pos[1], alt=korfez_alt, area_meter=korfez_scan_meter1, distance_meter=korfez_distance_meter1)
korfez_missions2 = vehicle.scan_area_wpler(center_lat=korfez_pos2[0], center_lon=korfez_pos2[1], alt=korfez_alt, area_meter=korfez_scan_meter1, distance_meter=korfez_distance_meter1)


vehicle.set_mode(mode="GUIDED", drone_id=2)
vehicle.send_all_waypoints_git(wp_list=korfez_missions1, drone_id=2)

vehicle.set_mode(mode="GUIDED", drone_id=3)
vehicle.send_all_waypoints_git(wp_list=korfez_missions2, drone_id=3)

print(vehicle.wp.count())