import socket
import time

ESP32_IP = '10.234.118.52' 
ESP32_PORT = 50002          

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 

def kirim_pesan_uji(angle, speed):
    """Mengirimkan perintah kontrol sederhana."""
    
    message = f"halo"
    
    try:
        sock.sendto(message.encode('utf-8'), (ESP32_IP, ESP32_PORT))
        print(f"[SENT] Message: {message.strip()}")
        return True
    except Exception as e:
        print(f"[ERROR] Gagal mengirim data: {e}")
        print("Pastikan IP dan Port sudah benar dan berada di jaringan yang sama.")
        return False

print("Memulai pengiriman pesan uji UDP...")

kirim_pesan_uji(angle=15.0, speed=25.0)
time.sleep(1)
kirim_pesan_uji(angle=-5.5, speed=10.0)
time.sleep(1)
kirim_pesan_uji(angle=0.0, speed=0.0) 

print("Pengujian selesai.")