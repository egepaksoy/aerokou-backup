import math
import sys
sys.path.append("../pymavlink_custom/")
from pymavlink_custom import calc_hipo_angle


pos = (300, 200)

screen_rat = (640, 480)

drone_pos = (40.123456, 32.123456, 10)


hipo, angle = calc_hipo_angle(screen_rat_x_y=screen_rat, x_y=pos, alt=drone_pos[2], yaw=0, alt_met=screen_rat[0])
new_pos = (drone_pos[0] + (float(hipo)*math.sin(math.radians(angle))), drone_pos[1] + (float(hipo)*math.cos(math.radians(angle))), drone_pos[2])

print(f"metre: {hipo} açı: {angle}")
print(f"pos: {new_pos}")