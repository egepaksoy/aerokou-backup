import time
import sys
sys.path.append('./pymavlink_custom')
from pymavlink_custom import Vehicle

vehicle = Vehicle("COM7")

drone_id = 3

vehicle.set_servo(channel=9, pwm=1000, drone_id=drone_id)
time.sleep(1.5)
vehicle.set_servo(channel=9, pwm=1750, drone_id=drone_id)
time.sleep(1.5)
vehicle.set_servo(channel=9, pwm=0, drone_id=drone_id)