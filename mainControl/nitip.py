import asyncio
import websockets
import cv2
import numpy as np
import base64
import json
import math
import time
import os
import socket 

# =================================================================
#                      KONFIGURASI SERVER PYTHON (HOST)
# =================================================================
# --- SERVER HOST CONFIG ---
WS_SERVER_IP = '0.0.0.0'    
WS_SERVER_PORT = 8080       

# --- MODIFIKASI: GUNAKAN VIDEO TEST ---
URL_KAMERA = 'videotest.mp4'       
TARGET_FPS = 30           # Diturunkan ke 10 FPS untuk debugging video
FRAME_WIDTH = 640         
FRAME_HEIGHT = 480

# Alamat UDP Kontrol Dihapus

# Inisialisasi Kamera
CAP = cv2.VideoCapture(URL_KAMERA) 
if not CAP.isOpened():
    print(f"ERROR: cannot open camera/video: {URL_KAMERA}")
    raise SystemExit(1)

CAP.set(cv2.CAP_PROP_BUFFERSIZE, 1)
CAP.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
CAP.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Variabel Global Telemetri
REAL_TIME_SPEED = 25.0  # Kecepatan Simulasi Konstan
REAL_TIME_DISTANCE = 0.0
LAST_TIME = time.time()
IS_ROBOT_RUNNING = True # Set True agar logic speed/jarak tetap berjalan


# =================================================================
#                       FUNGSI VISI & UTILITY
# =================================================================

def calculate_angle_from_line(x1, y1, x2, y2):
    if abs(x2 - x1) < 1e-3: return 90.0
    gradien = (y2 - y1) / (x2 - x1)
    return max(-90.0, min(90.0, math.degrees(math.atan(gradien))))

def get_perspective_transform_points(lebar, tinggi):
    src = np.float32([
        [lebar * 0.40, tinggi * 0.60], [lebar * 0.60, tinggi * 0.60],  
        [lebar * 0.10, tinggi * 0.95], [lebar * 0.90, tinggi * 0.95]
    ])
    offset = lebar * 0.2
    dst = np.float32([
        [offset, 0], [lebar - offset, 0], [offset, tinggi], [lebar - offset, tinggi]      
    ])
    return src, dst

def estimate_robot_lane_position(overlay, lane_status) -> str:
    # [Logic Estimasi Posisi Robot SAMA]
    if lane_status == "Lost": return "Lost"
    h, w = overlay.shape[:2]
    bottom_half = overlay[int(h*0.7):, :]
    gray = cv2.cvtColor(bottom_half, cv2.COLOR_BGR2GRAY)
    coords = cv2.findNonZero(gray)
    if coords is None: return "Lost"
    min_x = np.min(coords[:, :, 0])
    max_x = np.max(coords[:, :, 0])
    center_of_mark = (min_x + max_x) / 2.0
    frame_center = w / 2.0
    tolerance = w * 0.10
    if center_of_mark < frame_center - tolerance:
        return "left"
    elif center_of_mark > frame_center + tolerance:
        return "right"
    else:
        return "center"

def detect_lane_and_overlay(frame):
    h, w = frame.shape[:2]
    
    # [Logic Visi HLS, BEV, Hough SAMA]
    hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
    lower_white = np.array([0, 150, 0]); upper_white = np.array([255, 255, 255])
    mask = cv2.inRange(hls, lower_white, upper_white)
    kernel = np.ones((3,3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
    
    src, dst = get_perspective_transform_points(w, h)
    M = cv2.getPerspectiveTransform(src, dst)
    bev_mask = cv2.warpPerspective(mask, M, (w, h), flags=cv2.INTER_LINEAR)
    
    edges = cv2.Canny(bev_mask, 50, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=20, minLineLength=20, maxLineGap=10)

    overlay = frame.copy()
    avg_angle = 0.0; lane_status = "Lost"

    if lines is not None and len(lines) > 0:
        lane_status = "Detected"
        angles = []
        M_inv = cv2.getPerspectiveTransform(dst, src)
        bev_line_image = np.zeros_like(frame) 
        for ln in lines:
            x1, y1, x2, y2 = ln.reshape(4)
            angles.append(calculate_angle_from_line(x1, y1, x2, y2)) 
            cv2.line(bev_line_image, (x1, y1), (x2, y2), (0,255,0), 3)
            
        avg_angle = float(np.mean(angles))
        warped_line = cv2.warpPerspective(bev_line_image, M_inv, (w, h))
        overlay = cv2.addWeighted(overlay, 1.0, warped_line, 1.0, 0)
    
    # Visualisasi Teks Lokal (menggunakan global)
    Y_OFFSET = 35 
    cv2.putText(overlay, f"Speed: {REAL_TIME_SPEED:.2f} cm/s", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.putText(overlay, f"Angle: {avg_angle:.2f} deg", (10, 20+Y_OFFSET), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(overlay, f"Jarak: {REAL_TIME_DISTANCE:.2f} m", (10, 20+2*Y_OFFSET), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(overlay, f"Lane: {lane_status}", (10, 20+3*Y_OFFSET), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    bev_mask_bgr = cv2.cvtColor(bev_mask, cv2.COLOR_GRAY2BGR)
    combined_debug_frame = np.concatenate((overlay, bev_mask_bgr), axis=1)

    return combined_debug_frame, avg_angle, lane_status, overlay


def encode_image_to_base64(img_bgr, quality=20):
    _, buf = cv2.imencode('.jpg', img_bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return base64.b64encode(buf).decode('utf-8')


# =================================================================
# FUNGSI HANDLER WEBSOCKET (DIREVISI UNTUK DEBUGGING)
# =================================================================

async def lane_detection_handler(websocket, path=None):
    """
    Handler ini dijalankan SETIAP KALI Base Station terhubung.
    Fokus hanya pada Visi dan Pengiriman Data.
    """
    global REAL_TIME_DISTANCE, LAST_TIME, REAL_TIME_SPEED
    
    # Reset state saat koneksi baru (untuk video test)
    REAL_TIME_DISTANCE = 0.0
    LAST_TIME = time.time()
    CAP.set(cv2.CAP_PROP_POS_FRAMES, 0) # Mulai video dari awal
    
    print(f"Base Station connected from {websocket.remote_address}. Starting video test...")
    ws = websocket 

    try:
        cv2.namedWindow("IRIS Local Debug (Tekan 'q' utk Stop)", cv2.WINDOW_AUTOSIZE)
        
        while CAP.isOpened():
            ret, frame = CAP.read()
            if ret:
                if frame.shape[1] != FRAME_WIDTH or frame.shape[0] != FRAME_HEIGHT:
                    frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
            if not ret:
                # Video loop
                CAP.set(cv2.CAP_PROP_POS_FRAMES, 0)
                await asyncio.sleep(0.1)
                continue

            current_time = time.time()
            dt = max(current_time - LAST_TIME, 1e-6)

            # 1. VISI KOMPUTER & TELEMETRI
            combined_debug_frame, angle, lane_status, frame_overlay_clean = detect_lane_and_overlay(frame)

            # 2. LOGIC SPEED & JARAK TEMPUH (Simulasi Berjalan)
            REAL_TIME_DISTANCE += (REAL_TIME_SPEED / 100.0) * dt # Jarak bertambah terus
            LAST_TIME = current_time

            # 3. DATA TELEMETRI JSON
            telemetry = {
                "type": "telemetry",
                "data": {
                    "speed": REAL_TIME_SPEED, 
                    "laneStatus": lane_status,
                    "steering_angle": round(float(angle), 2),
                    "jarakTempuh": round(float(REAL_TIME_DISTANCE), 3)
                }
            }

            # 4. ENCODING dan Pengiriman
            raw_b64 = encode_image_to_base64(frame)
            proc_b64 = encode_image_to_base64(frame_overlay_clean, quality=20) 

            data_raw = {"type": "image_raw", "data": raw_b64, "width": frame.shape[1], "height": frame.shape[0]}
            data_proc = {"type": "image_processed", "data": proc_b64, "width": frame.shape[1], "height": frame.shape[0]}

            # TAMPILKAN LOKAL & KIRIM
            cv2.imshow("IRIS Local Debug (Tekan 'q' utk Stop)", combined_debug_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            await ws.send(json.dumps(data_raw))
            await ws.send(json.dumps(data_proc))
            await ws.send(json.dumps(telemetry))

            loop_elapsed = time.time() - current_time
            await asyncio.sleep(max(0.0, (1.0 / TARGET_FPS) - loop_elapsed))
        
    except websockets.exceptions.ConnectionClosed:
        print("Base Station disconnected gracefully.")
    except Exception as e:
        print(f"Error fatal in handler: {e}")
    finally:
        cv2.destroyAllWindows()
        print("Handler stopped. Clean up done.")


# =================================================================
# FUNGSI UTAMA SERVER (HOST)
# =================================================================

async def main():
    # UDP Receiver dihilangkan karena fokus pada visi offline (videotest.mp4)
    # Jika Anda ingin mengaktifkan UDP, kembalikan logic udp_speed_receiver di sini

    # 2. MULAI WEBSOCKET SERVER
    async with websockets.serve(lane_detection_handler, WS_SERVER_IP, WS_SERVER_PORT):
        print(f"Python WebSocket Server running on ws://{WS_SERVER_IP}:{WS_SERVER_PORT}")
        await asyncio.Future() 

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[System Stop] Program dihentikan oleh pengguna.")
    except Exception as e:
        print(f"Error fatal di main: {e}")