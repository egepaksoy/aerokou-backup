import sys
sys.path.append("./pymavlink_custom")
from pymavlink_custom import Vehicle
import time

vehicle = Vehicle("COM12")

vehicle.request_message_interval("ATTITUDE", 1)
vehicle.request_message_interval("GLOBAL_POSITION_INT", 2)
vehicle.request_message_interval("MISSION_ITEM_REACHED", 3)
msg = vehicle.vehicle.recv_match()
print(msg)
vehicle.set_servo(pwm=1000)
time.sleep(2)
vehicle.set_servo(pwm=1750)
time.sleep(2)
vehicle.set_servo(pwm=0)
time.sleep(2)