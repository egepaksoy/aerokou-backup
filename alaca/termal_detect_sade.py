#! RASPBERRY
import serial, time
import numpy as np
import cv2

def bozuk(degerler: list):
    t_min = degerler.min()
    t_max = degerler.max()

    if t_min / 100 > 25 and t_min / 100 < 120:
        return False
    return True

# MCU'dan sıcaklık alma işlevi (Santigrat derece x 100)
def get_temp_array(d):
    # ortam sıcaklığı alma
    T_a = (int(d[1540]) + int(d[1541])*256)/100

    # ham piksel dizisi sıcaklığını alma
    raw_data = d[4:1540]
    T_array = np.frombuffer(raw_data, dtype=np.int16)

    return T_a, T_array

# görüntüdeki sıcaklıkları piksellere dönüştürme işlevi
def td_to_image(f):
    Tmax = 40
    Tmin = 0
    norm = np.uint8((f/100 - Tmin)*255/(Tmax-Tmin))
    norm.shape = (24,32)
    return norm

########################### Ana döngü #################################
# Renk haritası aralığı

fire_file = open("./fire-temp.txt", "w+")
fire_file.close()

print ('Serial port ayarlanıyor')
ser = serial.Serial ('/dev/serial0')
ser.baudrate = 115200

# modülün frekansını 4 Hz olarak ayarlama
ser.write(serial.to_bytes([0xA5,0x25,0x01,0xCB]))
time.sleep(0.1)

# Otomatik veri toplamayı başlatma
ser.write(serial.to_bytes([0xA5,0x35,0x02,0xDC]))
t0 = time.time()


try:
    while True:
        # veri frame'i bekleniyor
        data = ser.read(1544)
        # Veriler hazır
        Ta, temp_array = get_temp_array(data)

        if Ta == 1:
                break

        if bozuk(temp_array):
                ser.close()
                print("Bozuldu tekrardan açılıyor...")
                ser = serial.Serial ('/dev/serial0')
                ser.baudrate = 115200
        
        else:
            print ("Tmax==" + str(temp_array.max()))
            print ("Tmin==" + str(temp_array.min()))

            maxt = 0
            for i, temp in enumerate(temp_array):
                if temp >= 4000 and temp > maxt:
                    maxt = temp
            
            if maxt > 0:
                with open("./fire-temp.txt", "a") as fire_file:
                    print(f"Yuksek sicaklik: {maxt}")
                    i = temp_array.tolist().index(maxt)
                    print(i//24+1, i%24+1)
                    fire_file.write(f"{i//24+1},{i%24+1},{maxt}\n")


        ta_img = td_to_image(temp_array)

        img = cv2.applyColorMap(ta_img, cv2.COLORMAP_JET)
        img = cv2.resize(img, (640, 480), interpolation = cv2.INTER_CUBIC)
        img = cv2.flip(img, 1)

        text = 'Tmin = {:+.1f} Tmax = {:+.1f} FPS = {:.2f}'.format(temp_array.min()/100, temp_array.max()/100, 1/(time.time() - t0))
        cv2.putText(img, text, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
        cv2.imshow('Output', img)
        cv2.moveWindow('Output',960,0)

        t0 = time.time()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    # döngüyü sonlandırmak için
    ser.write(serial.to_bytes([0xA5,0x35,0x01,0xDB]))
    ser.close()
    print(' Stopped')

finally:
    cv2.destroyAllWindows()
    ser.close()