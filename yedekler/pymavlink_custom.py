from pymavlink import mavutil, mavwp
import os
import time
import math

DEG = 0.00001144032
wp = mavwp.MAVWPLoader()

def get_wp_list(vehicle, drone_id: int=1):
    vehicle.mav.mission_request_list_send(drone_id, vehicle.target_component)
    msg = vehicle.recv_match(type='MISSION_COUNT', blocking=True)
    waypoint_count = msg.count

    waypoints = []
    for i in range(waypoint_count):
        vehicle.mav.mission_request_int_send(drone_id, vehicle.target_component, i)
        msg = vehicle.recv_match(type='MISSION_ITEM_INT', blocking=True)
        waypoints.append((msg.x / 1e7, msg.y / 1e7, msg.z / 1e3))  # Latitude ve Longitude değerleri 1e7 ile ölçeklendirilmiştir

    return waypoints

def request_message_interval(vehicle, message_input: str, frequency_hz: float, drone_ids: list=[1]):
    for drone_id in drone_ids:
        message_name = "MAVLINK_MSG_ID_" + message_input
        message_id = getattr(mavutil.mavlink, message_name)
        vehicle.mav.command_long_send(
            drone_id, drone_id,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id,
            1e6 / frequency_hz,
            0,
            0, 0, 0, 0)
        print(f"Requested the message drone id {drone_id} successfully.")

def get_pos(vehicle, drone_id: int=1):
    while True:
        message = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if message and message.get_srcSystem() == drone_id:
            lat = message.lat / 1e7
            lon = message.lon / 1e7
            alt = message.relative_alt / 1e3
            return lat, lon, alt
        else:
            print("Konum bekleniyor...")
            time.sleep(1)

def get_miss_wp(vehicle, drone_id: int=1):
    while True:
        message = vehicle.recv_match(type='MISSION_ITEM_REACHED', blocking=True)
        if message and message.get_srcSystem() == drone_id:
            return int(message.seq)
        return 0

def get_yaw(vehicle, drone_id: int=1):
    while True:
        message = vehicle.recv_match(type='ATTITUDE', blocking=True)
        if message and message.get_srcSystem() == drone_id:
            yaw_deg = math.degrees(message.yaw)
            if yaw_deg < 0:
                yaw_deg += 360
            return yaw_deg
        else:
            print("Yaw acisi cekiliyor...")
            time.sleep(1)

def ack(vehicle, keyword, drone_id: int=1):
    msg = vehicle.recv_match(type=keyword, blocking=True)
    if msg and msg.get_srcSystem() == drone_id:
        print("-- Message Read " + str(msg))

def clear_wp(vehicle):
    vehicle.waypoint_clear_all_send()
    print("Waypointler silindi")

#TODO: bu fonksiyonu digeriyle birlestir
def add_mission(vehicle,  seq, lat, lon, alt, drone_id: int=1):
    frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
    wp.add(mavutil.mavlink.MAVLink_mission_item_message(
        drone_id, vehicle.target_component,
        seq, frame,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat, lon, alt))
    vehicle.waypoint_clear_all_send()
    vehicle.waypoint_count_send(wp.count())
    for i in range(wp.count()):
        msg = vehicle.recv_match(type=["MISSION_REQUEST"], blocking=True)
        if msg.get_srcSystem() == drone_id and msg:
            vehicle.mav.send(wp.wp(msg.seq))
            print("Sending waypoints {0}".format(msg.seq))

def add_mission_custom(vehicle, seq, lat, lon, alt, drone_id: int=1):
    if seq == 0:
        clear_wp(vehicle=vehicle)

    mission = mavutil.mavlink.MAVLink_mission_item_message(
        target_system=drone_id,
        target_component=vehicle.target_component,
        seq=seq,
        frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        current=0 if seq > 1 else 1,  # İlk waypoint'i current olarak ayarla
        autocontinue=1,
        param1=0, param2=0, param3=0, param4=0,
        x=lat,
        y=lon,
        z=alt
    )

    vehicle.mav.send(mission)
    ack(vehicle=vehicle, keyword="MISSION_ACK", drone_id=drone_id)
    print(f"Waypoint {seq} sent")

def scan_area(vehicle, seq, center_lat, center_lon, alt, area_meter, distance_meter, drone_id: int=1):
    met = -1 * (area_meter / 2)
    sign = 1
    i = 0
    while i <= (area_meter / distance_meter):
        add_mission(vehicle=vehicle, seq=seq + i, lat=center_lat + (met + distance_meter * i) * DEG, lon=center_lon + (met * sign) * DEG, alt=alt, drone_id=drone_id)
        sign *= -1
        i += 1
    
    return seq + i, (center_lat + (met + distance_meter * (i - 1)) * DEG, center_lon + (met * (sign * -1)) * DEG)

def arm_disarm(vehicle, arm, drone_id: int=1):
    if arm == 0 or arm == 1:
        vehicle.mav.command_long_send(drone_id, drone_id, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, arm, 0, 0, 0, 0, 0, 0)
        if arm == 1:
            print(f"{drone_id} idli drone arm edildi")
        elif arm == 0:
            print(f"{drone_id} idli drone disarm edildi")
    else:
        print(f"Gecersiz arm kodu: {arm}")
        exit()

def takeoff_custom(vehicle, alt, drone_id: int=1):
    set_mode(vehicle=vehicle, mod="AUTO", drone_id=drone_id)
    
    arm_disarm(vehicle=vehicle, arm=1, drone_id=drone_id)
    
    vehicle.mav.command_long_send(
        drone_id, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)

    current_alt = get_pos(vehicle=vehicle, drone_id=drone_id)[2]
    start_time = time.time()

    while current_alt < alt * 0.95:
        if time.time() - start_time > 1:
            current_alt = get_pos(vehicle=vehicle, drone_id=drone_id)[2]
            print(f"Anlık irtifa: {current_alt} metre")
            start_time = time.time()

    print(f"{alt} metreye yükseldi")

def takeoff(vehicle, alt, drone_id: int=1):
    set_mode(vehicle=vehicle, mod="GUIDED", drone_id=drone_id)
    
    arm_disarm(vehicle=vehicle, arm=1, drone_id=drone_id)
    
    vehicle.mav.command_long_send(
        drone_id, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)
    
    current_alt = get_pos(vehicle=vehicle, drone_id=drone_id)[2]
    start_time = time.time()
    
    while current_alt < alt * 0.95:
        if time.time() - start_time > 2:
            current_alt = get_pos(vehicle=vehicle, drone_id=drone_id)[2]
            print(f"Anlık irtifa: {current_alt} metre")
            start_time = time.time()

    print(f"{alt} metreye yükseldi")

def set_mode(vehicle, mod, drone_id: int=1):
    if mod == "RTL":
        vehicle.mav.command_long_send(drone_id, vehicle.target_component, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0)

    elif mod == "AUTO":
        vehicle.mav.command_long_send(drone_id, vehicle.target_component, mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0)

    else:
        if mod not in vehicle.mode_mapping():
            print("Mod değiştirilemedi gecersiz mod: ", mod)
            exit()
        else:
            mode = vehicle.mode_mapping()[mod]
            vehicle.mav.command_long_send(drone_id, drone_id, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode, 0, 0, 0, 0, 0)
            time.sleep(0.1)

    start_time = time.time()
    while get_mode(vehicle=vehicle, drone_id=drone_id) != mode and time.time() - start_time < 5:
        if time.time() - start_time / 2 == 0:
            print("Mod degistirme tekrar deneniyor...")
        vehicle.mav.command_long_send(drone_id, drone_id, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode, 0, 0, 0, 0, 0)
    if get_mode(vehicle=vehicle, drone_id=drone_id) != mode:
        print("Mod degistirilemedi")
        exit()

    print(f"Mod {mod} yapıldı")

def is_armed(vehicle, drone_id: int=1):
    return vehicle.sysid_state[drone_id].armed

def check_address(address):
    if "udp" not in address:
        if not os.path.exists(address):
            print("ACM dosya yolu yanlis yada yok:\n", address)
            exit()
    print("Baglanti yolu onaylandi")

def get_mode(vehicle, drone_id: int=1):
    msg = vehicle.recv_match(type="HEARTBEAT", blocking=True)
    if msg and msg.get_srcSystem() == drone_id:
        return mavutil.mode_string_v10(msg)
    return 0

def set_servo(vehicle, drone_id: int=1, channel: int=7):
    vehicle.mav.command_long_send(
        drone_id,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        channel,
        1750,
        0, 0, 0, 0, 0)
    print("Servo aciliyor...")
    time.sleep(3)
    vehicle.mav.command_long_send(
        drone_id,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        channel,
        1000,
        0, 0, 0, 0, 0)
    time.sleep(3)
    vehicle.mav.command_long_send(
        drone_id,
        drone_id,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        channel,
        0,
        0, 0, 0, 0, 0)

    print("Servo durduruldu")