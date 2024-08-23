import sys
sys.path.append("./pymavlink_custom/")
from pymavlink_custom import Vehicle
import time

vehicle = Vehicle(address="tcp:127.0.0.1:5762")
lat, lon = -35.36312501, 149.16520965
alt = 2
wp_list = [(lat, lon, alt), (lat, lon, alt)]

try:
    vehicle.arm_disarm(arm=True)
    vehicle.takeoff(alt=alt)
    takeoff_pos = vehicle.get_pos()
    wp_list.append((takeoff_pos[0], takeoff_pos[1], alt))
    wp_list.append(takeoff_pos)
    vehicle.add_mission_list(wp_list)
    vehicle.set_mode(mode="AUTO")

    wpler = vehicle.get_wp_list()
    print(wpler)
    
    start_time = time.time()
    while True:
        if time.time() - start_time > 2:
            print(vehicle.get_miss_wp())
        
        '''
        if vehicle.on_location(loc=takeoff_pos, seq=2, sapma=1):
            vehicle.set_mode(mode="LAND")
            break
        '''

except KeyboardInterrupt:
    vehicle.set_mode(mode="RTL")

finally:
    while vehicle.is_armed():
        pass
    vehicle.set_mode(mode="GUIDED")
    vehicle.clear_wp_target()
    vehicle.vehicle.close()