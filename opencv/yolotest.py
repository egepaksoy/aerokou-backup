import io
import time
import cv2
import numpy
import sys
sys.path.append("../pymavlink_custom")
from pymavlink import calc_hipo_angle
from ultralytics import YOLO
import os

model = YOLO('../korfez/models/kullanilcak.pt')
cap = cv2.VideoCapture(0)

try:
    start_time = time.time()
    while True:
        rate, frame = cap.read()
        if not rate:
            break

        if time.time() - start_time > 0.1:
            results = model(frame)
            
            for r in results:
                boxes = r.boxes
                for box in boxes:
                    conf = box.conf[0].item()

                    # Sınırlayıcı kutu koordinatlarını al
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

                    if conf >= 0.85 and abs(x1-x2) <= 400 and abs(y1-y2) <= 300:
                        # Sınıf ve güven skorunu al
                        cls = int(box.cls[0].item())
                        
                        # Sınıf adını al
                        class_name = model.names[cls]
                        
                        # Bilgileri ekrana yazdır
                        print(f"Sınıf: {class_name}, Güven: {conf:.2f}, Konum: ({x1:.0f}, {y1:.0f}, {x2:.0f}, {y2:.0f})")
                        
                        # Nesneyi çerçeve içine al ve etiketle
                        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                        cv2.putText(frame, f"{class_name} {conf:.2f}", (int(x1), int(y1 - 10)), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                        
                        print(f"Konum: {x1},{y1},{x2},{y2}")
                        print(f"Mesafe {calc_hipo_angle((640, 480), (abs(x2-x1)/2+x1, abs(y2-y1)/2+y1), 10, 0, 640)}")

                        fire_loc = [int(x1) + int((x2 - x1) / 2), int(y1) + int((y2 - y1) / 2)]

                        with open("konum.txt", "a+") as f:
                            f.write(f"{fire_loc[0]},{fire_loc[1]}\n")

            cv2.imshow("YOLOv8 Canlı Tespit", frame)
            start_time = time.time()
        

        if cv2.waitKey(1) & 0xFF == ord('q'):
           break
finally:
    cv2.destroyAllWindows()
    cap.release()
