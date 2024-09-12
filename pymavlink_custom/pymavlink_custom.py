from pymavlink import mavutil, mavwp
from pymavlink.dialects.v10 import ardupilotmega as mavlink
import os
import time
import math

class Vehicle():
    def __init__(self, address: str, baud: int=57600, autoreconnect: bool=True, drone_id: int=1, on_flight: bool=True):
        try:
            self.check_address(address=address)
            self.vehicle = mavutil.mavlink_connection(device=address, baud=baud, autoreconnect=autoreconnect)
            self.vehicle.wait_heartbeat()
            print("Bağlantı başarılı")
            self.drone_id = drone_id
            self.drone_ids = []
            # 1 Metre
            self.DEG = 0.00001172485

            self.get_all_drone_ids()

            if on_flight:
                drone_idler = self.get_all_drone_ids()

                if len(self.drone_ids) == 1:
                    self.drone_id = list(drone_idler)[0]
                    print(f"Ucusta tek drone var ve id'si: {self.drone_id}")
                else:
                    print("Ucustaki drone idleri: ", self.drone_ids)

                self.request_message_interval("ATTITUDE", 1, self.drone_ids)
                self.request_message_interval("GLOBAL_POSITION_INT", 2, self.drone_ids)
                self.request_message_interval("MISSION_ITEM_REACHED", 3, self.drone_ids)
                self.request_message_interval("VFR_HUD", 4, self.drone_ids)

                print("Mesajlar gonderildi")

                self.wp = mavwp.MAVWPLoader()
        except Exception as e:
            print("Baglanti saglanamadi: ", e)
            exit()

    # Baglantidaki tum drone idlerini getirir
    def get_all_drone_ids(self):
        drone_ids = set()

        start_time = time.time()

        try:
            self.vehicle.wait_heartbeat(blocking=True)
            while time.time() - start_time < 3:
                msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True)
                if msg:
                    drone_ids.add(msg.get_srcSystem())

            self.drone_ids.clear()
            for d_id in drone_ids:
                if d_id != 255:
                    self.drone_ids.append(d_id)
            
            return drone_ids
        except Exception as e:
            return e
    
    def check_connection(self):
        try:
            self.get_all_drone_ids()
            return True

        except Exception as e:
            return e

    def get_speed(self, drone_id: int=None):
        if drone_id is None:
            drone_id = self.drone_id
        
        try:
            while True:
                message = self.vehicle.recv_match(type='VFR_HUD', blocking=True)
                if message and message.get_srcSystem() == drone_id:
                    return message.airspeed
        except Exception as e:
            return e

    # Waypointleri döndürür
    def get_wp_list(self, drone_id: int=None):
        if drone_id is None:
            drone_id = self.drone_id

        try:
            self.vehicle.mav.mission_request_list_send(drone_id, self.vehicle.target_component)
            msg = self.vehicle.recv_match(type='MISSION_COUNT', blocking=True)
            waypoint_count = msg.count

            waypoints = []
            for i in range(waypoint_count):
                self.vehicle.mav.mission_request_int_send(drone_id, self.vehicle.target_component, i)
                msg = self.vehicle.recv_match(type='MISSION_ITEM_INT', blocking=True)
                waypoints.append((msg.x / 1e7, msg.y / 1e7, msg.z / 1e3))  # Latitude ve Longitude değerleri 1e7 ile ölçeklendirilmiştir

            return waypoints

        except Exception as e:
            return e

    # Cekilecek mesajları ister
    def request_message_interval(self, message_input: str, frequency_hz: float, drone_ids: list=None):
        if drone_ids is None:
            drone_ids = self.drone_ids
        
        try:
            for drone_id in drone_ids:
                message_name = "MAVLINK_MSG_ID_" + message_input
                message_id = getattr(mavutil.mavlink, message_name)
                self.vehicle.mav.command_long_send(
                    drone_id, drone_id,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                    message_id,
                    1e6 / frequency_hz,
                    0,
                    0, 0, 0, 0)
                print(f"{drone_id} id'li drona {message_input} mesajı basairyla iletildi.")
        except Exception as e:
            return e

    # Dronun konumunu döndürür
    def get_pos(self, drone_id: int=None):
        if drone_id is None:
            drone_id = self.drone_id

        try:
            while True:
                message = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
                if message and message.get_srcSystem() == drone_id:
                    lat = message.lat / 1e7
                    lon = message.lon / 1e7
                    alt = message.relative_alt / 1e3
                    return lat, lon, alt
                # print(f"Drone {drone_id} Konum bekleniyor...")
        except Exception as e:
            return e

    # Anlık waypoint'i döndürür
    def get_miss_wp(self, drone_id: int=None):
        if drone_id is None:
            drone_id = self.drone_id

        try:
            while True:
                message = self.vehicle.recv_match(type='MISSION_ITEM_REACHED', blocking=True)
                if message and message.get_srcSystem() == drone_id:
                    return int(message.seq)
                print("Waypoint mesaji bekleniyor...")
        except Exception as e:
            return e

    # Yaw acisini derece cinsinden dondurur
    def get_yaw(self, drone_id: int=None):
        if drone_id is None:
            drone_id = self.drone_id

        try:
            while True:
                message = self.vehicle.recv_match(type='ATTITUDE', blocking=True)
                if message and message.get_srcSystem() == drone_id:
                    yaw_deg = math.degrees(message.yaw)
                    if yaw_deg < 0:
                        yaw_deg += 360.0
                    return yaw_deg
                print("Yaw acisi cekiliyor...")
        except Exception as e:
            return e

    # Komut gercekleştirme mesaşlarını bekler
    def ack(self, keyword: str=None, keywords: list=None, drone_id: int=None):
        if keywords is None:
            keywords = [keyword]
        if drone_id is None:
            drone_id = self.drone_id
        
        try:
            msg = self.vehicle.recv_match(type=keywords, blocking=True)
            if msg.get_srcSystem() == drone_id and msg:
                print("-- Message Read " + str(msg))
                return True
            return False
        except Exception as e:
            return e
        
    # ID'li drone'nun waypointlerini siler
    def clear_wp_target(self, drone_id: int=None):
        if drone_id is None:
            drone_id = self.drone_id

        try:
            if self.vehicle.mavlink10():
                self.vehicle.mav.mission_clear_all_send(drone_id, self.vehicle.target_component)

            else:
                self.vehicle.mav.waypoint_clear_all_send(drone_id, self.vehicle.target_component)
            print(f"{drone_id} idli drone'nun waypointleri silindi")
        except Exception as e:
            return e

    # TODO: TEK WP EKLEYEN VERSIYONUNA BAK + WAYPOINT EKLERKEN SON EKLENENI CURRENT YAPIYOR ONU DUZELT
    # Tım waypointleri cekerek ger yeni waypoint gonderildiginde hepsini tekrar gonderir
    def add_mission(self, seq, lat, lon, alt, drone_id: int=None):
        if drone_id is None:
            drone_id = self.drone_id

        try:
            self.wp.add(mavutil.mavlink.MAVLink_mission_item_message(
                drone_id, self.vehicle.target_component,
                seq, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat, lon, alt))

            self.vehicle.waypoint_clear_all_send()
            self.vehicle.waypoint_count_send(self.wp.count())

            for i in range(self.wp.count()):
                msg = self.vehicle.recv_match(type=["MISSION_REQUEST"], blocking=True)
                if msg.get_srcSystem() == drone_id and msg:
                    self.vehicle.mav.send(self.wp.wp(msg.seq))
                    print("Sending waypoints {0}".format(msg.seq))
        except Exception as e:
            return e

    # Liste şeklinde gönderilen waypointleri ekler
    def add_mission_list(self, wp_list: list, drone_id: int=None):
        if drone_id is None:
            drone_id = self.drone_id

        try:
            self.clear_wp_target(drone_id=drone_id)
            self.vehicle.waypoint_count_send(len(wp_list))

            seq = 0
            for waypoint in wp_list:
                self.wp.add(mavutil.mavlink.MAVLink_mission_item_message(drone_id, self.vehicle.target_component, seq, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, waypoint[0], waypoint[1], waypoint[2]))
                seq += 1

            for i in range(self.wp.count()):
                msg = self.vehicle.recv_match(type=["MISSION_REQUEST"], blocking=True)
                if msg.get_srcSystem() == drone_id and msg:
                    self.vehicle.mav.send(self.wp.wp(msg.seq))
                    print("Sending waypoints {0}".format(msg.seq))
            
            self.vehicle.mav.mission_set_current_send(
                drone_id,
                self.vehicle.target_component,
                0
            )
        except Exception as e:
            return e

    # TODO: HIZ AYARLAMASINA BAK
    # Dronu guided modunda hareket ettirir
    def go_to(self, lat, lon, alt, drone_id: int=None):
        if drone_id is None:
            drone_id = self.drone_id

        try:
            self.vehicle.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, drone_id,
                            self.vehicle.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                            int(0b110111111000),
                            int(lat * 1e7), int(lon * 1e7), alt,
                            0, 0, 0, 0, 0, 0, 0, 0)
                        )
        except Exception as e:
            return e
    
    # Drone'u belirtilen alana tarama yapacak şekilde waypointler ekler
    def scan_area(self, seq, center_lat, center_lon, alt, area_meter, distance_meter, drone_id: int=None):
        if drone_id is None:
            drone_id = self.drone_id
        met = -1 * (area_meter / 2)
        sign = 1
        i = 0

        try:
            while i <= (area_meter / distance_meter):
                last_waypoint = (center_lat + (met + distance_meter * i) * self.DEG, center_lon + (met * sign) * self.DEG)
                last_seq = seq + i
                self.add_mission(seq=last_seq, lat=last_waypoint[0], lon=last_waypoint[1], alt=alt, drone_id=drone_id)
                sign *= -1
                i += 1
            
            return last_seq, last_waypoint
        except Exception as e:
            return e

    # Drone'u arm eder veya disarm eder
    def arm_disarm(self, arm, drone_id: int=None, force_arm: bool=False):
        if drone_id is None:
            drone_id = self.drone_id

        try:
            if arm == 0 or arm == 1:
                if force_arm:
                    self.vehicle.mav.command_long_send(drone_id, self.vehicle.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, arm, 0, 21196, 0, 0, 0, 0)
                else:
                    self.vehicle.mav.command_long_send(drone_id, self.vehicle.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, arm, 0, 0, 0, 0, 0, 0)
                if arm == 0:
                    print("Disarm edildi")
                if arm == 1:
                    print("ARM edildi")
            else:
                print(f"Gecersiz arm kodu: {arm}")
                exit()
        except Exception as e:
            return e

    # Drona takeoff verir
    def takeoff(self, alt, drone_id: int=None):
        if drone_id is None:
            drone_id = self.drone_id

        try:
            self.vehicle.mav.command_long_send(
                drone_id, self.vehicle.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)
            
            print("Takeoff mesaji gonderildi")
            
            current_alt = 0
            start_time = time.time()

            print(f"Takeoff alınıyor...")
            while current_alt < alt * 0.9:
                current_alt = self.get_pos(drone_id=drone_id)[2]
                if time.time() - start_time > 2:
                    print(f"Anlık irtifa: {current_alt} metre")
                    start_time = time.time()
            
            print(f"{alt} metreye yükseldi")
        except Exception as e:
            return e

    # Dronun modunu belirler
    def set_mode(self, mode: str, drone_id: int=None):
        if drone_id is None:
            drone_id = self.drone_id
        
        try:
            if mode == "RTL":
                self.vehicle.mav.command_long_send(drone_id, self.vehicle.target_component, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0)

            elif mode == "AUTO":
                self.vehicle.mav.command_long_send(drone_id, self.vehicle.target_component, mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0)

            else:
                mode_id = self.vehicle.mode_mapping()[mode]
                if mode not in self.vehicle.mode_mapping():
                    print("Mod değiştirilemedi gecersiz mod: ", mode)
                    exit()
                else:
                    self.vehicle.mav.command_long_send(drone_id, self.vehicle.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)

            time.sleep(1)
            current_mode = self.get_mode(drone_id=drone_id)
            if current_mode != mode:
                self.set_mode(mode=mode, drone_id=drone_id)

            else:
                print(f"Mod {mode} yapıldı")
        except Exception as e:
            return e

    # Dronun arm durumunu kontrol eder
    def is_armed(self, drone_id: int=None):
        if drone_id is None:
            drone_id = self.drone_id

        try:
            start_time = time.time()
            while True:
                msg = self.vehicle.recv_match(type="HEARTBEAT", blocking=True)
                if msg and msg.get_srcSystem() == drone_id:
                    if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                        return 1
                    else:
                        return 0
                if time.time() - start_time > 5:
                    print("UYARI!!! 5 saniyedir arm durumu cekilmedi!!!")
                    break
        except Exception as e:
            return e
            
    # Dronun baglanti yolunu kontrol eder
    def check_address(self, address: str):
        if "tcp" not in address:
            if not os.path.exists(address):
                print("Dosya yolu yanlis yada yok:\n", address)
                exit()
        print("Baglanti yolu onaylandi")

    # Dronun modunu elde eder
    def get_mode(self, drone_id: int=None):
        if drone_id is None:
            drone_id = self.drone_id
        
        try:
            start_time = time.time()
            while True:
                msg = self.vehicle.recv_match(type="HEARTBEAT", blocking=True)
                if msg and msg.get_srcSystem() == drone_id:
                    return mavutil.mode_string_v10(msg)
                if time.time() - start_time > 5:
                    print("UYARI!!! 5 SANiYEDiR MOD BiLGiSi ALINAMADI!!!")
                    return None
        except Exception as e:
            return e

    # Dronun serbo pwm'ini ayarlar
    def set_servo(self, drone_id: int=None, channel: int=9, pwm: int=1000):
        if drone_id is None:
            drone_id = self.drone_id

        try:
            self.vehicle.mav.command_long_send(
                drone_id,
                self.vehicle.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                channel,
                pwm,
                0, 0, 0, 0, 0)
        except Exception as e:
            return e

    def get_distance(self, loc1, loc2):
        R = 6371000
    
        lat1, lon1 = loc1
        lat2, lon2 = loc2

        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        
        a = math.sin(delta_phi / 2.0) ** 2 + \
            math.cos(phi1) * math.cos(phi2) * \
            math.sin(delta_lambda / 2.0) ** 2
        
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        return R * c

    # Konumda oldugunu kontrol etme
    def on_location(self, loc, seq: int=0, sapma: int=4, drone_id: int=None):
        if drone_id is None:
            drone_id = self.drone_id
        
        try:
            if seq != 0:
                if abs(self.get_pos(drone_id=drone_id)[0] - loc[0]) <= self.DEG/sapma and abs(self.get_pos(drone_id=drone_id)[1] - loc[1]) <= self.DEG/sapma and self.get_miss_wp(drone_id=drone_id) == seq:
                    return True
                return False
            else:
                if abs(self.get_pos(drone_id=drone_id)[0] - loc[0]) <= self.DEG/sapma and abs(self.get_pos(drone_id=drone_id)[1] - loc[1]) <= self.DEG/sapma:
                    return True
                return False
        except Exception as e:
            return e

# Kameradan goruntu hesaplama kodu
def calc_hipo_angle(screen_rat_x_y, x_y, alt, yaw, alt_met):
    screen_x = screen_rat_x_y[0]
    screen_y = screen_rat_x_y[1]
    screen_y_mid = screen_y/2
    screen_x_mid = screen_x/2
    x = x_y[0]
    y = x_y[1]

    hipo = math.sqrt(abs(screen_x_mid-x)**2 + abs(screen_y_mid-y)**2)

    angle = math.degrees(math.atan2(abs(screen_x_mid-x),abs(screen_y_mid-y)))

    x_sign = x-screen_x_mid
    y_sign = y-screen_y_mid

    if x_sign < 0 and y_sign < 0:
        angle = 270+angle
    elif x_sign < 0 and y_sign > 0:
        angle = 180 + angle
    elif x_sign > 0 and y_sign < 0:
        angle = angle
    elif x_sign > 0 and y_sign > 0:
        angle = 180 - angle
    elif x_sign == 0 and y_sign < 0:
        angle = 0
    elif x_sign == 0 and y_sign > 0:
        angle = 180
    elif x_sign < 0 and y_sign == 0:
        angle = 270
    elif x_sign > 0 and y_sign == 0:
        angle = 90
    elif x_sign == 0 and y_sign == 0:
        angle = 0

    #       metre       derece
    return hipo*alt/alt_met, (yaw + angle) % 360

#TODO: fonksiyonlara bak bilgi cekenlerde while olayı koy get_mode de oldugu gibi