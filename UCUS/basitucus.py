import time
import threading
import queue
import sys
sys.path.append('../pymavlink_custom')
from pymavlink_custom import Vehicle

# THREAD FONKSİYONLARI

def get_pos(vehicle, return_queue, drone_id: int=None):
    result = vehicle.get_pos(drone_id)
    return_queue.put(result)

def takeoff(vehicle, alt: int):
    vehicle.takeoff(alt=alt)

pos_queue = queue.Queue()

# kod

vehicle = Vehicle("tcp:127.0.0.1:5762")

pos_thread = threading.Thread(target=get_pos, args=(vehicle, pos_queue))
takeoff_thread = threading.Thread(target=takeoff, args=(vehicle, 10))

vehicle.set_mode("GUIDED")

pos_thread.start()
pos_thread.join()

pos = pos_queue.get()
vehicle.add_mission(seq=0, lat=pos[0], lon=pos[1], alt=10)
vehicle.add_mission(seq=0, lat=pos[0], lon=pos[1], alt=10)

vehicle.arm_disarm(arm=True)
takeoff_thread.start()

start_time = time.time()
miss_time = time.time()
while True:
    if time.time() - start_time > 2:
        print(vehicle.get_pos())
        start_time = time.time()
    
    if time.time() - miss_time > 10:
        break

takeoff_thread.join()
print("modertl")
vehicle.set_mode("RTL")