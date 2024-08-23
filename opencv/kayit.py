import io
import time
import socket
import struct
from PIL import Image
import cv2
import numpy
import sys
import os

video_adı = 'kaydedilen_video.mp4'
video_cozunurlugu = (640, 480)

# Video kayıt nesnesini başlat
video_kaydi = cv2.VideoWriter(video_adı, cv2.VideoWriter_fourcc(*'mp4v'), 20.0, video_cozunurlugu)
server_socket = socket.socket()
server_socket.bind((sys.argv[1], int(sys.argv[2])))  
server_socket.listen(0)
print("Listening")
connection = server_socket.accept()[0].makefile('rb')

try:
    img = None
    start_time = time.time()
    while True:
        image_len = struct.unpack('<L', connection.read(struct.calcsize('<L')))[0]
        if not image_len:
            break
        image_stream = io.BytesIO()
        image_stream.write(connection.read(image_len))
        image_stream.seek(0)
        image = Image.open(image_stream)
        frame = cv2.cvtColor(numpy.array(image), cv2.COLOR_RGB2BGR) # goruntu isleme yapmak icin hazir hale getiriyoruz
            
        video_kaydi.write(frame)
        cv2.imshow("YOLOv8 Canlı Tespit", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
           break

finally:
    cv2.destroyAllWindows()
    connection.close()
    video_kaydi.release()
    server_socket.close()