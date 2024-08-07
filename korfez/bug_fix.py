import math
from ultralytics import YOLO
import cv2
import time

screen_rat = (640, 480)
cap = cv2.VideoCapture(0)
cap.set(3, screen_rat[0])
cap.set(4, screen_rat[1])
with open("./konum.txt", "w") as loc_file:
    loc_file.close()

model = YOLO("./models/kullanilcak.pt")

start_time = time.time()
while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.resize(frame, screen_rat, interpolation = cv2.INTER_CUBIC)
    frame = cv2.flip(frame, 1)

    if time.time() - start_time > 0.1:
        results = model(frame)
        for r in results:
            boxes = r.boxes
            for box in boxes:
                if box.conf[0] < 0.90:
                    continue
                # Sınırlayıcı kutu koordinatlarını al
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

                # Sınıf ve güven skorunu al
                cls = int(box.cls[0].item())
                conf = box.conf[0].item()

                # Sınıf adını al
                class_name = model.names[cls]

                # Bilgileri ekrana yazdır
                print(f"Sınıf: {class_name}, Güven: {conf:.2f}, Konum: ({x1:.0f}, {y1:.0f}, {x2:.0f}, {y2:.0f}")

                # Nesneyi çerçeve içine al ve etiketle
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(frame, f"{class_name} {conf:.2f}", (int(x1), int(y1 - 10)), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                
                #! TODO: aldığın konumları drona anlık göndermenin yolunu bul !!!
                with open("./konum.txt", "a") as loc_file:
                    loc_file.write(f"{x1},{x2},{y1},{y2}\n")
        start_time = time.time()


    cv2.imshow('YOLOv8 Canlı Tespit', frame)  # Görüntüyü göster

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break