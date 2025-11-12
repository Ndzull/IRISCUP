import asyncio
import websockets
import cv2
import time
import os
from typing import Tuple, Optional, Dict, Any

# --- IMPORT MODUL EKSTERNAL (Harus ada di PYTHONPATH) ---
from communication import CommunicationManager # Mengelola WS, UDP In/Out, Global State
from vision import VisionProcessor             # Mengelola Frame Processing, BEV
from control import RobotController            # Mengelola Logic Keputusan

# =================================================================
#                      KONFIGURASI SERVER PYTHON (HOST)
# =================================================================
# --- SERVER HOST CONFIG (Digunakan oleh websockets.serve) ---
WS_SERVER_IP = '0.0.0.0'    
WS_SERVER_PORT = 8080       
# --------------------------
# [Konfigurasi lainnya]
URL_KAMERA = 'videotest.mp4'  
UDP_LISTEN_PORT = 50001
UDP_CONTROL_IP = '192.168.4.100'
UDP_CONTROL_PORT = 50002
TARGET_FPS = 20
FRAME_WIDTH = 640         
FRAME_HEIGHT = 480
DEFAULT_SPEED = 45.0

# =================================================================
# GLOBAL STATE & INITIALIZATION
# =================================================================
COMM: Optional[CommunicationManager] = None 
VISION: Optional[VisionProcessor] = None 
CONTROL: Optional[RobotController] = None
CAP = cv2.VideoCapture(URL_KAMERA) 

if not CAP.isOpened():
    print(f"ERROR: cannot open camera/video: {URL_KAMERA}")
    raise SystemExit(1) 

# Pengaturan Global CAP (Diperlukan untuk membaca video)
CAP.set(cv2.CAP_PROP_BUFFERSIZE, 1)
CAP.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
CAP.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)


# =================================================================
# FUNGSI HANDLER WEBSOCKET (Memicu Main Loop)
# =================================================================

async def lane_detection_handler(websocket, path):
    """Handler yang dijalankan SETIAP KALI Base Station terhubung."""
    global COMM, VISION, CONTROL
    
    # 1. Reset State dan Inisialisasi Handler
    COMM.REAL_TIME_DISTANCE = 0.0
    COMM.RUNNING = False # Tunggu tombol START
    print(f"Base Station connected from {websocket.remote_address}. Awaiting START signal...")
    
    # 2. Start Task Receiver WS (untuk START/STOP)
    receiver_task = asyncio.create_task(COMM.ws_receiver_loop(websocket))

    try:
        cv2.namedWindow("IRIS Local Debug (press q to stop)", cv2.WINDOW_AUTOSIZE)
        
        # 3. Main Loop Visi
        while CAP.isOpened():
            t0 = time.time()
            ret, frame = CAP.read()
            if not ret:
                # Loop video file jika selesai
                if URL_KAMERA.lower().endswith(('.mp4', '.avi', '.mov', '.mkv')):
                    CAP.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    await asyncio.sleep(0.05)
                    continue
                else:
                    break
            
            # Paksa Resize Frame
            if frame.shape[1] != FRAME_WIDTH or frame.shape[0] != FRAME_HEIGHT:
                 frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))

            # Hitung waktu (dt) untuk odometry
            now = time.time()
            dt = max(now - COMM.LAST_TIME, 1e-6)
            
            # --- PHASE 1: VISI KOMPUTER ---
            overlay_frame, angle, lane_status, robot_position, bev_mask = VISION.process_frame_for_lane_data(frame)

            # --- PHASE 2: KONTROL DAN KEPUTUSAN ---
            target_steer, target_speed = 0.0, 0.0
            
            if COMM.RUNNING: 
                target_steer, target_speed = CONTROL.decide_command(
                    angle, 
                    lane_status, 
                    COMM.CURRENT_OBSTACLE_INFO
                )
            
            # Kirim Perintah Kontrol UDP
            COMM.send_control_command(target_steer, target_speed)

            # --- PHASE 3: UPDATE TELEMETRI (Internal) ---
            if COMM.REAL_TIME_SPEED > 0.01:
                COMM.REAL_TIME_DISTANCE += (COMM.REAL_TIME_SPEED / 100.0) * dt
            COMM.LAST_TIME = now
            
            # --- PHASE 4: BROADCAST MONITORING DATA (Payload dan Pengiriman) ---
            telemetry_data = {
                "steering_angle": round(float(target_steer), 2),
                "laneStatus": lane_status,
                "robotPosition": robot_position,
                "speed": round(COMM.REAL_TIME_SPEED, 2),
                "jarakTempuh": round(COMM.REAL_TIME_DISTANCE, 3),
                "obstacle": COMM.CURRENT_OBSTACLE_INFO,
                "running": COMM.RUNNING
            }
            
            # Prepare Images (Memanggil fungsi encode_image_to_base64 dari suatu tempat)
            raw_b64 = encode_image_to_base64(frame, quality=25) 
            proc_b64 = encode_image_to_base64(overlay_frame, quality=25) 

            # Kirim Data WS
            await COMM.broadcast_telemetry({"type": "telemetry", "data": telemetry_data})
            await COMM.broadcast_telemetry({"type": "image_raw", "data": raw_b64, "width": FRAME_WIDTH, "height": FRAME_HEIGHT})
            await COMM.broadcast_telemetry({"type": "image_processed", "data": proc_b64, "width": FRAME_WIDTH, "height": FRAME_HEIGHT})
            
            # Tampilkan Debug Lokal
            cv2.imshow("IRIS Local Debug (press q to stop)", overlay_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'): break

            # Kontrol FPS
            elapsed = time.time() - t0
            await asyncio.sleep(max(0.0, (1.0 / TARGET_FPS) - elapsed))

    except Exception as e:
        print(f"[FATAL ERROR] Handler Halted: {e}")
    finally:
        # Emergency Stop dan Cleanup
        COMM.send_control_command(0.0, 0.0) 
        CAP.set(cv2.CAP_PROP_POS_FRAMES, 0) # Reset video file
        cv2.destroyAllWindows()
        receiver_task.cancel()


# =================================================================
# FUNGSI UTAMA SERVER (ORCHESTRATOR)
# =================================================================

def encode_image_to_base64(img_bgr, quality=25) -> str:
    """TEMPORARY DEFINITION: Fungsi ini harus dipindahkan ke communication.py atau utility.py"""
    import base64 as b64
    _, buf = cv2.imencode(".jpg", img_bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return b64.b64encode(buf).decode("utf-8")

async def main_processing_loop():
    """Tugas utama yang membaca kamera, memproses visi, dan mengirim perintah kontrol."""
    global COMM, VISION, CONTROL, CAP
    
    # Inisialisasi state time
    last_time = time.time()
    
    try:
        # Inisialisasi Vision dan Control State
        # Note: CURRENT_STEER, CURRENT_POSITION, dll. harus didefinisikan 
        # sebagai variabel global atau atribut class jika digunakan di luar loop
        
        while CAP.isOpened():
            t0 = time.time()
            ret, frame = CAP.read()
            if not ret:
                # Loop video file jika selesai
                if URL_KAMERA.lower().endswith(('.mp4', '.avi', '.mov', '.mkv')):
                    CAP.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    await asyncio.sleep(0.05)
                    continue
                else:
                    break
            
            # Paksa Resize Frame
            if frame.shape[1] != FRAME_WIDTH or frame.shape[0] != FRAME_HEIGHT:
                 frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))

            # Hitung waktu (dt) untuk odometry
            now = time.time()
            dt = max(now - last_time, 1e-6)
            
            # --- PHASE 1: VISI KOMPUTER ---
            # Asumsi VisionProcessor memiliki method yang mengembalikan data dan frame overlay
            # Ganti nama variabel sesuai output VisionProcessor Anda
            overlay_frame_raw, angle, lane_status, robot_position, bev_mask = VISION.process_frame_for_lane_data(frame)

            # --- PHASE 2: UPDATE TELEMETRI (Internal State) ---
            
            # 1. Update Jarak Tempuh (menggunakan data speed dari UDP)
            if COMM.REAL_TIME_SPEED > 0.01:
                COMM.REAL_TIME_DISTANCE += (COMM.REAL_TIME_SPEED / 100.0) * dt
            
            # 2. Update Waktu Acuan
            COMM.LAST_TIME = now
            
            # --- PHASE 3: KONTROL DAN KEPUTUSAN ---
            target_steer, target_speed = 0.0, 0.0
            
            if COMM.RUNNING: # Hanya kirim perintah jika tombol START ditekan
                target_steer, target_speed = CONTROL.decide_command(
                    angle, 
                    lane_status, 
                    COMM.CURRENT_OBSTACLE_INFO
                )
            
            # Kirim Perintah Kontrol UDP ke ESP32/STM32
            COMM.send_control_command(target_steer, target_speed)

            # --- PHASE 4: BROADCAST MONITORING DATA KE BASE STATION ---
            
            # a. Prepare Telemetry JSON (Menggunakan target_steer dari control logic)
            telemetry_data = {
                "steering_angle": round(float(target_steer), 2),
                "laneStatus": lane_status,
                "robotPosition": robot_position,
                "speed": round(COMM.REAL_TIME_SPEED, 2),
                "jarakTempuh": round(COMM.REAL_TIME_DISTANCE, 3),
                "obstacle": COMM.CURRENT_OBSTACLE_INFO,
                "running": COMM.RUNNING
            }
            
            # b. Prepare Images and Broadcast
            # Pastikan COMM.broadcast_telemetry() berfungsi (di communication.py)
            await COMM.broadcast_telemetry({"type": "telemetry", "data": telemetry_data})
            
            # Broadcast Gambar (Optional, tergantung implementasi broadcast_telemetry)
            # Anda harus memindahkan logic encoding base64 ke sini jika broadcast_telemetry tidak melakukannya
            
            # --- DEBUG LOKAL ---
            # Asumsi overlay_frame_raw adalah frame yang sudah digabungkan untuk imshow
            cv2.imshow("IRIS Local Debug (press q to stop)", overlay_frame_raw)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # Kontrol FPS
            elapsed = time.time() - t0
            await asyncio.sleep(max(0.0, (1.0 / TARGET_FPS) - elapsed))
            
    except Exception as e:
        print(f"[FATAL ERROR] Main Loop Halted: {e}")
        # Hentikan robot jika loop utama crash
        COMM.send_control_command(0.0, 0.0) 
    finally:
        CAP.release()
        cv2.destroyAllWindows()
        # Perlu cleanup tambahan untuk semua task yang berjalan di luar main_loop

async def main_orchestrator():
    global COMM, VISION, CONTROL
    
    # 1. INISIALISASI KOMPONEN
    COMM = CommunicationManager(WS_SERVER_PORT, UDP_LISTEN_PORT, UDP_CONTROL_IP, UDP_CONTROL_PORT)
    # Asumsi VisionProcessor dan RobotController dibuat dengan nilai default
    VISION = VisionProcessor(FRAME_WIDTH, FRAME_HEIGHT) 
    CONTROL = RobotController() 

    # 2. Start UDP Receiver
    await COMM.start_udp_listener()

    # 3. Start WebSocket Server
    ws_server = await websockets.serve(lane_detection_handler, WS_SERVER_IP, WS_SERVER_PORT)
    print(f"[WS] WebSocket server running on ws://{WS_SERVER_IP}:{WS_SERVER_PORT}")
    
    # 4. Run WS Server dan Main Processing Loop concurrently
    await asyncio.gather(
        ws_server.wait_closed(), # Menjaga server WS tetap aktif
        main_processing_loop()   # Loop Visi dan Kontrol
    )


if __name__ == "__main__":
    try:
        asyncio.run(main_orchestrator())
    except KeyboardInterrupt:
        print("\n[System Stop] Program dihentikan oleh pengguna.")
    except Exception as e:
        print(f"Error fatal di main: {e}")