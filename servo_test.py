import time
import sys
sys.path.append('./pymavlink_custom')
from pymavlink_custom import Vehicle

if len(sys.argv) != 3:
    print("servo_test.py com7 drone_id")
    exit()

vehicle = Vehicle(sys.argv[1], on_flight=False)

drone_id = int(sys.argv[2])

vehicle.set_servo(channel=9, pwm=1000, drone_id=drone_id)
time.sleep(1.5)
vehicle.set_servo(channel=9, pwm=1750, drone_id=drone_id)
time.sleep(1.5)
vehicle.set_servo(channel=9, pwm=0, drone_id=drone_id)