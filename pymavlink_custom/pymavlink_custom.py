from pymavlink import mavutil, mavwp
import os
import time
import math

class Vehicle():
    def __init__(self, address: str, baud: int=57600, autoreconnect: bool=True, drone_id: int=2):
        self.check_address(address=address)
        self.vehicle = mavutil.mavlink_connection(address, baud=baud, autoreconnect=autoreconnect)
        self.vehicle.wait_heartbeat()
        print("Bağlantı başarılı")
        print(f"Ucustaki drone idleri: {self.get_all_drone_ids()}")

        self.request_message_interval("ATTITUDE", 1)
        self.request_message_interval("GLOBAL_POSITION_INT", 2)
        self.request_message_interval("MISSION_ITEM_REACHED", 3)

        print("Mesajlar gonderildi")

        # 1 Metre
        self.DEG = 0.00001144032
        self.wp = mavwp.MAVWPLoader()
        self.drone_ids = self.get_all_drone_ids()

    # Baglantidaki tum drone idlerini getirir
    def get_all_drone_ids(self):
        drone_ids = set()

        start_time = time.time()
        while time.time() - start_time < 1.5:
            msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True)
            if msg:
                drone_ids.add(msg.get_srcSystem())

        return drone_ids

    # Waypointleri döndürür
    def get_wp_list(self, drone_id: int=2):
        self.vehicle.mav.mission_request_list_send(drone_id, self.vehicle.target_component)
        msg = self.vehicle.recv_match(type='MISSION_COUNT', blocking=True)
        waypoint_count = msg.count

        waypoints = []
        for i in range(waypoint_count):
            self.vehicle.mav.mission_request_int_send(drone_id, self.vehicle.target_component, i)
            msg = self.vehicle.recv_match(type='MISSION_ITEM_INT', blocking=True)
            waypoints.append((msg.x / 1e7, msg.y / 1e7, msg.z / 1e3))  # Latitude ve Longitude değerleri 1e7 ile ölçeklendirilmiştir

        return waypoints

    # Cekilecek mesajları ister
    def request_message_interval(self, message_input: str, frequency_hz: float, drone_ids: list=[2]):
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

    # Dronun konumunu döndürür
    def get_pos(self, drone_id: int=2):
        while True:
            message = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if message and message.get_srcSystem() == drone_id:
                lat = message.lat / 1e7
                lon = message.lon / 1e7
                alt = message.relative_alt / 1e3
                return lat, lon, alt
            else:
                print("Konum bekleniyor...")
                time.sleep(1)

    # Anlık waypoint'i döndürür
    def get_miss_wp(self, drone_id: int=2):
        while True:
            message = self.vehicle.recv_match(type='MISSION_ITEM_REACHED', blocking=True)
            if message and message.get_srcSystem() == drone_id:
                return int(message.seq)
            return 0

    # Yaw acisini derece cinsinden dondurur
    def get_yaw(self, drone_id: int=2):
        while True:
            message = self.vehicle.recv_match(type='ATTITUDE', blocking=True)
            if message and message.get_srcSystem() == drone_id:
                yaw_deg = math.degrees(message.yaw)
                if yaw_deg < 0:
                    yaw_deg += 360
                return yaw_deg
            else:
                print("Yaw acisi cekiliyor...")
                time.sleep(1)

    # Komut gercekleştirme mesaşlarını bekler
    def ack(self, keyword, drone_id: int=2):
        msg = self.vehicle.recv_match(type=keyword, blocking=True)
        if msg and msg.get_srcSystem() == drone_id:
            print("-- Message Read " + str(msg))

    # ID'li drone'nun waypointlerini siler
    def clear_wp_target(self, drone_id: int=2):
        if self.vehicle.mav.mavlink10():
            self.vehicle.mav.mission_clear_all_send(drone_id, self.vehicle.target_component)
        else:
            self.vehicle.mav.waypoint_clear_all_send(drone_id, self.vehicle.target_component)
        print(f"{drone_id} idli drone'nun waypointleri silindi")

    # Dronun waypointlerini siler
    def clear_wp(self):
        self.vehicle.waypoint_clear_all_send()
        print("Waypointler silindi")

    # Tım waypointleri cekerek ger yeni waypoint gonderildiginde hepsini tekrar gonderir
    def eski_add_mission(self,  seq, lat, lon, alt, drone_id: int=2):
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        self.wp.add(mavutil.mavlink.MAVLink_mission_item_message(
            drone_id, self.vehicle.target_component,
            seq, frame,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat, lon, alt))
        self.vehicle.waypoint_clear_all_send()
        self.vehicle.waypoint_count_send(self.wp.count())
        for i in range(self.wp.count()):
            msg = self.vehicle.recv_match(type=["MISSION_REQUEST"], blocking=True)
            if msg.get_srcSystem() == drone_id and msg:
                self.vehicle.mav.send(self.wp.wp(msg.seq))
                print("Sending waypoints {0}".format(msg.seq))

    # Waypoint ekler
    def add_mission(self, seq, lat, lon, alt, drone_id: int=2):
        if seq == 0:
            self.clear_wp_target(drone_id=drone_id)

        mission = mavutil.mavlink.MAVLink_mission_item_message(
            target_system=drone_id,
            target_component=self.vehicle.target_component,
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

        self.vehicle.mav.send(mission)
        self.ack(keyword="MISSION_ACK", drone_id=drone_id)
        print(f"Waypoint {seq} sent")

    # Drone'u belirtilen alana tarama yapacak şekilde waypointler ekler
    def scan_area(self, seq, center_lat, center_lon, alt, area_meter, distance_meter, drone_id: int=2):
        met = -1 * (area_meter / 2)
        sign = 1
        i = 0
        while i <= (area_meter / distance_meter):
            self.add_mission(seq=seq + i, lat=center_lat + (met + distance_meter * i) * self.DEG, lon=center_lon + (met * sign) * self.DEG, alt=alt, drone_id=drone_id)
            sign *= -1
            i += 1
        
        return seq + i, (center_lat + (met + distance_meter * (i - 1)) * self.DEG, center_lon + (met * (sign * -1)) * self.DEG)
    
    # Eski tarama kodu
    def eski_scan_area(self, seq, center_lat, center_lon, alt, area_meter, distance_meter, drone_id: int=2):
        met = -1 * (area_meter / 2)
        sign = 1
        i = 0
        while i <= (area_meter / distance_meter):
            self.eski_add_mission(seq=seq + i, lat=center_lat + (met + distance_meter * i) * self.DEG, lon=center_lon + (met * sign) * self.DEG, alt=alt, drone_id=drone_id)
            sign *= -1
            i += 1
        
        return seq + i, (center_lat + (met + distance_meter * (i - 1)) * self.DEG, center_lon + (met * (sign * -1)) * self.DEG)

    # Drone'u arm eder veya disarm eder
    def arm_disarm(self, arm, drone_id: int=2):
        if arm == 0 or arm == 1:
            self.vehicle.mav.command_long_send(drone_id, drone_id, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, arm, 0, 0, 0, 0, 0, 0)
            if arm == 1:
                print(f"{drone_id} idli drone arm edildi")
            elif arm == 0:
                print(f"{drone_id} idli drone disarm edildi")
        else:
            print(f"Gecersiz arm kodu: {arm}")
            exit()

    # Drona takeoff verir
    def takeoff_mode(self, alt, mode: str, drone_id: int=2):
        self.set_mode(mode=mode, drone_id=drone_id)
        
        self.arm_disarm(arm=True, drone_id=drone_id)
        time.sleep(0.5)
        
        self.vehicle.mav.command_long_send(
            drone_id, self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)

        current_alt = self.get_pos(drone_id=drone_id)[2]
        start_time = time.time()

        while current_alt < alt * 0.95:
            if time.time() - start_time > 1:
                current_alt = self.get_pos(drone_id=drone_id)[2]
                print(f"Anlık irtifa: {current_alt} metre")
                start_time = time.time()

        print(f"{alt} metreye yükseldi")

    def takeoff(self, alt, drone_id: int=2):
        self.vehicle.mav.command_long_send(
            drone_id, self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)
        
        current_alt = self.get_pos(drone_id=drone_id)[2]
        start_time = time.time()

        print(f"Drone arm edildi ve modu: {self.get_mode(drone_id=drone_id)}\nTakeoff alınıyor...")
        while current_alt < alt * 0.95:
            if time.time() - start_time > 2:
                current_alt = self.get_pos(drone_id=drone_id)[2]
                print(f"Anlık irtifa: {current_alt} metre")
                start_time = time.time()

        print(f"{alt} metreye yükseldi")

    def set_mode(self, mode: str, drone_id: int=2):
        mode_id = self.vehicle.mode_mapping()[mode]
        if mode == "RTL":
            self.vehicle.mav.command_long_send(drone_id, self.vehicle.target_component, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0)

        elif mode == "AUTO":
            self.vehicle.mav.command_long_send(drone_id, self.vehicle.target_component, mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0)

        else:
            if mode not in self.vehicle.mode_mapping():
                print("Mod değiştirilemedi gecersiz mod: ", mode)
                exit()
            else:
                self.vehicle.mav.command_long_send(drone_id, self.vehicle.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
                time.sleep(0.1)

        start_time = time.time()
        while self.get_mode(drone_id=drone_id) != mode and time.time() - start_time < 5:
            if time.time() - start_time / 2 == 0:
                print("Mod degistirme tekrar deneniyor...")

            self.vehicle.mav.command_long_send(drone_id, self.vehicle.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)

        if self.get_mode(drone_id=drone_id) != mode:
            print("Mod degistirilemedi")
            exit()

        print(f"Mod {mode} yapıldı")

    def is_armed(self, drone_id: int=2):
        return self.vehicle.sysid_state[drone_id].armed

    def check_address(self, address):
        if "udp" not in address:
            if not os.path.exists(address):
                print("ACM dosya yolu yanlis yada yok:\n", address)
                exit()
        print("Baglanti yolu onaylandi")

    def get_mode(self, drone_id: int=2):
        msg = self.vehicle.recv_match(type="HEARTBEAT", blocking=True)
        if msg and msg.get_srcSystem() == drone_id:
            return mavutil.mode_string_v10(msg)
        return 0

    def set_servo(self, drone_id: int=2, channel: int=7):
        self.vehicle.mav.command_long_send(
            drone_id,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            channel,
            1750,
            0, 0, 0, 0, 0)
        print("Servo aciliyor...")
        time.sleep(3)
        self.vehicle.mav.command_long_send(
            drone_id,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            channel,
            1000,
            0, 0, 0, 0, 0)
        time.sleep(3)
        self.vehicle.mav.command_long_send(
            drone_id,
            drone_id,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            channel,
            0,
            0, 0, 0, 0, 0)

        print("Servo durduruldu")

    # Konumda oldugunu kontrol etme
    def on_location(self, loc, seq, sapma: int=4, drone_id: int=2, check_seq: bool=True):
        if check_seq:
            if abs(self.get_pos(drone_id=drone_id)[0] - loc[0]) <= self.DEG/sapma and abs(self.get_pos(drone_id=drone_id)[1] - loc[1]) <= self.DEG/sapma and self.get_miss_wp(drone_id=drone_id) == seq:
                return True
            return False
        else:
            if abs(self.get_pos(drone_id=drone_id)[0] - loc[0]) <= self.DEG/sapma and abs(self.get_pos(drone_id=drone_id)[1] - loc[1]) <= self.DEG/sapma:
                return True
            return False


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

import socket

# Mesaj gonderir
def send_message(hedef_ip, hedef_port, mesaj):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client_socket:
        client_socket.sendto(mesaj.encode(), (hedef_ip, hedef_port))
        response, _ = client_socket.recvfrom(1024)
        print(f"Gonderildi: {response.decode()}")

# Mesaj bekler
def start_server(hedef_ip, hedef_port):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server_socket:
        server_socket.bind((hedef_ip, hedef_port))
        print(f"{hedef_ip}:{hedef_port} adresi dinleniyor")
        while True:
            data, addr = server_socket.recvfrom(1024)
            print(f"{addr} adresinden alındı: {data.decode()}")
            server_socket.sendto(data, addr)  # İsteğe bağlı olarak geri yanıt gönderir
