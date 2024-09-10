import time
import sys
sys.path.append("./pymavlink_custom")
from pymavlink_custom import Vehicle

if len(sys.argv) != 2:
    exit()
vehicle = Vehicle(sys.argv[0])

ALT = 5

try:
    vehicle.takeoff(alt=ALT)

    # BURAYA RTL KODU örnek:
    # while rtl_alindi == False:
    #   continue
    # if rtl_alindi == True:  
    #   vehicle.set_mode("RTL")

except Exception as e:
    print("Hata: ", e)

except KeyboardInterrupt:
    for drone_id in vehicle.drone_ids:
        print(f"{drone_id} RTL aldi")
        vehicle.set_mode("RTL", drone_id=drone_id)

finally:
    print("Kapatildi")
    vehicle.vehicle.close()