import sys
import math
sys.path.append('../pymavlink_custom')
from pymavlink_custom import calc_hipo_angle

pos = []

while True:
    with open("./konum.txt", "r") as loc_file:
        for line in loc_file:
            loc = list(map(float, line.strip().split(",")))
            if loc not in pos:
                pos.append(loc)
                print(f"Yeni konum: {loc}")
                hipo, angle = calc_hipo_angle((640, 480), (loc[0] + abs(loc[0] - loc[1]), loc[2] + abs(loc[2] - loc[3])), 5.42, 0, 640)
                x = hipo * math.cos(angle) #0.015
                y = hipo * math.sin(angle)
                print(f"{x}, {y}")