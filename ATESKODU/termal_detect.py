#! RASPBERRY
import sys
import serial, time
import datetime as dt
import numpy as np
import socket
import cv2

def send_message(host, port, message):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client_socket:
        client_socket.sendto(message.encode(), (host, port))
        response, _ = client_socket.recvfrom(1024)
        print(f"Gonderildi: {response.decode()}")

def main():
    def bozuk(degerler: list):
        t_min = degerler.min()
        t_max = degerler.max()

        if t_min / 100 > 15 and t_min / 100 < 120:
            return False
        return True

    # MCU'dan sıcaklık alma işlevi (Santigrat derece x 100)
    def get_temp_array(d, maxt):
        # ortam sıcaklığı alma
        T_a = (int(d[1540]) + int(d[1541])*256)/100

        # ham piksel dizisi sıcaklığını alma
        raw_data = d[4:1540]
        T_array = np.frombuffer(raw_data, dtype=np.int16)
        #print(T_array)

        numbers = str(T_array)

        temp = list(map(int, numbers.strip('[]').split()))

        #print(temp)

        sayac = 0

        y = 0
        new_maxt = 0
        new_maxt_index  = []
        a = 1
        t = []
        temps = []
        for te in temp:
            t.append(te)
            if a == 32:
                temps.append(t)
                a = 0
                t = []
            a += 1
        print(len(temps))
        temp = temps

        while y < len(temp):
            x = 0
            while x < len(temp[y]):
                if int(temp[y][x]) > 5500:
                        new_maxt = int(temp[y][x])
                        new_maxt_index = [y,x]
                if int(temp[y][x]) > maxt:
                    maxt = int(temp[y][x])
                x += 1
            y += 1

        '''
        if new_maxt != 0 and bozuk(T_array) == False:
            with open("./konum.txt", "a+") as f:
                print("YENI EN YUKSEK SICAKLIK: ", str(maxt))
                f.write(f"{new_maxt_index[0]},{new_maxt_index[1]},{maxt}\n")
        '''

        if new_maxt != 0 and bozuk(T_array) == False:
            print("Sicaklik bulundu: ", str(maxt))
            send_message(sys.argv[1], 9000, f"{new_maxt_index[0]},{new_maxt_index[1]}")
            exit()

        '''
        with open("./miss-file.txt", "r") as miss_file:
            for line in miss_file:
                if "end-mission" in line:
                    print("Görev tamamlandı, en yüksek sıcaklık ="+ str(maxt))
                    return 1,T_array, maxt
        '''

        print ("Sayaç = " + str(sayac))
        #! T_array pikselleri oluşturuyor
        return T_a, T_array, maxt

    # görüntüdeki sıcaklıkları piksellere dönüştürme işlevi
    def td_to_image(f):
        norm = np.uint8((f/100 - Tmin)*255/(Tmax-Tmin))
        norm.shape = (24,32)
        return norm

    ########################### Ana döngü #################################
    # Renk haritası aralığı
    Tmax = 40
    Tmin = 0

    print ('Configuring Serial port')
    ser = serial.Serial ('/dev/serial0')
    ser.baudrate = 115200

    # modülün frekansını 4 Hz olarak ayarlama
    ser.write(serial.to_bytes([0xA5,0x25,0x01,0xCB]))
    time.sleep(0.1)

    # Otomatik veri toplamayı başlatma
    ser.write(serial.to_bytes([0xA5,0x35,0x02,0xDC]))
    t0 = time.time()


    try:
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter('output.mp4', fourcc, 10.0, (640, 480))
        maxt = 0
        while True:
            # veri frame'i bekleniyor
            data = ser.read(1544)
            # Veriler hazır
            Ta, temp_array, maxt = get_temp_array(data, maxt)
            if bozuk(temp_array):
                    ser.close()

                    print("Bozuldu tekrardan açılıyor...")
                    ser = serial.Serial ('/dev/serial0')
                    ser.baudrate = 115200

            if Ta == 1:
                    break


            if bozuk(temp_array) == False:
                print ("Tmax==" + str(temp_array.max()))
                print ("Tmin==" + str(temp_array.min()))
                for temp in temp_array:
                    if temp > maxt:
                        maxt = temp


            ta_img = td_to_image(temp_array)

            # Image processing
            img = cv2.applyColorMap(ta_img, cv2.COLORMAP_JET)
            img = cv2.resize(img, (640, 480), interpolation = cv2.INTER_CUBIC)
            img = cv2.flip(img, 1)

            #text = 'Tmin = {:+.1f} Tmax = {:+.1f} FPS = {:.2f}'.format(temp_array.min()/100, temp_array.max()/100, 1/(time.time() - t0))
            #cv2.putText(img, text, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
            #cv2.imshow('Output', img)
            out.write(img)
            #cv2.moveWindow('Output',960,0)
            #if 's' is pressed - saving of picture

            t0 = time.time()

    except KeyboardInterrupt:
        # döngüyü sonlandırmak için
        ser.write(serial.to_bytes([0xA5,0x35,0x01,0xDB]))
        ser.close()
        out.release()
        cv2.destroyAllWindows()
        print(' Stopped')

    # her ihtimale karşı kapatma
    ser.close()
    #cv2.destroyAllWindows()
if __name__ == "__main__":
    main()