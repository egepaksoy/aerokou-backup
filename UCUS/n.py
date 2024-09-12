import threading
import time

# Thread'lerin çalışmasını kontrol etmek için bir bayrak
running = True

# Sayı yazdıran fonksiyon
def print_numbers():
    global running
    while running:
        print("Sayı: 1")
        time.sleep(1)

# Harf yazdıran fonksiyon
def print_letters():
    global running
    while running:
        print("Harf: A")
        time.sleep(1.5)

# Thread'leri oluşturma
thread1 = threading.Thread(target=print_numbers)
thread2 = threading.Thread(target=print_letters)

# Thread'leri başlatma
thread1.start()
thread2.start()

try:
    # Ana thread bu noktada çalışmaya devam eder
    while True:
        time.sleep(0.1)  # Ana thread'i beklemeye alıyoruz.
except KeyboardInterrupt:
    # Ctrl + C'ye basıldığında bu blok çalışacak
    print("Durduruluyor...")
    running = False  # Bayrağı False yaparak thread'leri durduruyoruz.

print("Program sonlandırıldı.")