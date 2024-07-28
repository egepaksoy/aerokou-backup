import sys
import math
sys.path.append('../pymavlink_custom')
from pymavlink_custom import calc_hipo_angle


uzaklik , aci = calc_hipo_angle(screen_rat_x_y=(640, 480), x_y=(320, 241), alt=640, yaw=0, alt_met=640)

print(uzaklik, aci)
#       yatay                   dikey
print(uzaklik*math.sin(math.radians(aci)), uzaklik*math.cos(math.radians(aci)))
# 15,625mm bir pixel 1 metre yukseklikte