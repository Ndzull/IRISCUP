import asyncio
import websockets
import cv2
import numpy as np
import base64
import json
import math
import time
import os

URL_KAMERA = 'http://10.108.152.171:8080/video'       #http://10.136.200.208:8080/video http://10.36.177.25:8080/video http://10.108.152.171:8080/video
URI_BS_WS = "ws://localhost:8080"  #base station
UDP_LISTEN_IP = '0.0.0.0' #untuk ngebaca realtime speed
UDP_LISTEN_PORT = 50001 
TARGET_FPS = 30        #set target fps (10 biar ngga ngelag tp tetep sama aja)
PIXEL_TO_METER = 0.01  #meter per pixel, 1 pixel = 1 cm (untuk optical flow)
MIN_FEATURES = 8       #min. titik optical flow yang kelacak
MAX_CORNERS = 80       #maks. titik optical flow yang dilacak

CAP = cv2.VideoCapture(URL_KAMERA) #inisialisasi kamera
if not CAP.isOpened():
    print(f"ERROR: gaiso buka dari {URL_KAMERA}")
    print("Working dir:", os.getcwd()) #ini berguna kalau file dari lokal sebenernya
    print("File exists:", os.path.exists(URL_KAMERA)) #buat mastiin file ilang sama error jaringan
    exit() #kalau gaiso buka, keluar

#ini buat atur fps & resolusi
CAP.set(cv2.CAP_PROP_FPS, TARGET_FPS)
CAP.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
CAP.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

REAL_TIME_SPEED = 0.0 #nilai awal speed
REAL_TIME_DISTANCE = 0.0 #nilai awal jarak tempuh
LAST_TIME = time.time() #waktu acuan jarakTambahan=kec=epatan.delta t
_prev_gray = None #nyimpen frame gray sebelumnya buat optical flow
_prev_pts = None #nyimpen titik2 sebelumnya buat optical flow

#hitung angle dari garis
def calculate_angle_from_line(x1, y1, x2, y2):
    if abs(x2 - x1) < 1e-3: #kalau garis vertikal (delta x < 0.001)
        return 90.0
    gradien = (y2 - y1) / (x2 - x1) #kalau ngga, hitung gradien (garis miring/horizontal)
    return max(-90, min(90, math.degrees(math.atan(gradien)))) #konversi ke derajat & batesin antara -90 sampai 90

#BEV
#nentuin titik tampilannya (trapesium)
def get_perspective_transform_points(lebar, tinggi):
    src = np.float32([ #Titik Trapesium
        [lebar * 0.40, tinggi * 0.60],  # Atas Kiri
        [lebar * 0.60, tinggi * 0.60],  # Atas Kanan
        [lebar * 0.10, tinggi * 0.95],  # Bawah Kiri
        [lebar * 0.90, tinggi * 0.95]   # Bawah Kanan
    ])
    #nentuin titik outputnya (persegi panjang)
    offset = lebar * 0.2
    dst = np.float32([
        [offset, 0], [lebar - offset, 0],  #atas kiri & kanan        
        [offset, tinggi], [lebar - offset, tinggi]  #bawah kiri & kanan 
    ])
    return src, dst

#ngambil frame mentah terus ubah jadi data terukur
def detect_lane_and_overlay(frame):
    h, w = frame.shape[:2] #ambil height & width frame
    
    hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS) #BGR ke HLS
    lower_white = np.array([0, 150, 0]) 
    upper_white = np.array([255, 255, 255])
    mask = cv2.inRange(hls, lower_white, upper_white)

    kernel = np.ones((3,3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

    src, dst = get_perspective_transform_points(w, h)
    M = cv2.getPerspectiveTransform(src, dst) #buat nampilin BEV yang lurus
    
    bev_mask = cv2.warpPerspective(mask, M, (w, h), flags=cv2.INTER_LINEAR)
    
    edges = cv2.Canny(bev_mask, 50, 150) #cari tepi
    
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=20, minLineLength=20, maxLineGap=10) #algoritma untuk deteksi garis lurus

    overlay = frame.copy() #buat salinan frame asli buat overlay (yang ada garis hijaunya)
    avg_angle = 0.0 #sudut rata-rata awal
    lane_status = "Lost" #status awal lane lost (kalau udah ketemu bakal ngubah jadi detected)

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

    
    bev_mask_bgr = cv2.cvtColor(bev_mask, cv2.COLOR_GRAY2BGR)
    
    combined_debug_frame = np.concatenate((overlay, bev_mask_bgr), axis=1)

    return combined_debug_frame, avg_angle, lane_status, mask

class SpeedReceiverProtocol(asyncio.DatagramProtocol):
    def datagram_received(self, data, addr):
        global REAL_TIME_SPEED
        try:
            message = data.decode('utf-8').strip()
            if message.startswith("S:"):
                REAL_TIME_SPEED = float(message.split(":")[1].strip())
        except Exception:
            pass

async def udp_speed_receiver(host, port):
    loop = asyncio.get_event_loop()
    try:
        transport, protocol = await loop.create_datagram_endpoint(
            lambda: SpeedReceiverProtocol(),
            local_addr=(host, port)
        )
        print(f"UDP Receiver listening on {host}:{port}")
        return transport, protocol
    except OSError as e:
        print(f"FATAL ERROR: Could not bind UDP to {host}:{port}. {e}")
        return None, None

def encode_image_to_base64(img_bgr, quality=20):
    _, buf = cv2.imencode('.jpg', img_bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return base64.b64encode(buf).decode('utf-8')

async def send_data():
    global REAL_TIME_DISTANCE, LAST_TIME, REAL_TIME_SPEED, _prev_gray, _prev_pts

    udp_transport, udp_protocol = await udp_speed_receiver(UDP_LISTEN_IP, UDP_LISTEN_PORT)

    websocket = None
    try:
        ws = await websockets.connect(URI_BS_WS)
        websocket = ws
        print(f"Connected to Base Station at {URI_BS_WS}")
    except Exception as e:
        print(f"Warning: Could not connect to WS {URI_BS_WS}: {e}")
        class DummyWS:
            async def send(self, data): pass
        websocket = DummyWS()
        print("Using DummyWS (offline mode)")

    try:
        cv2.namedWindow("Ijul ngedebug ('q' bwt Stop)", cv2.WINDOW_NORMAL)

        while CAP.isOpened():
            ret, frame = CAP.read()
            if ret:
                #maksa resize ke 640x480
                if frame.shape[1] != 640 or frame.shape[0] != 480:
                    frame = cv2.resize(frame, (640, 480))
            if not ret:
                if URL_KAMERA.lower().endswith(('.mp4', '.avi', '.mov', '.mkv')):
                    CAP.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    await asyncio.sleep(0.1)
                    continue
                else:
                    break

            current_time = time.time()
            dt = max(current_time - LAST_TIME, 1e-6)

            frame_overlay, angle, lane_status, mask = detect_lane_and_overlay(frame)

            vision_speed_m_s = 0.0
            try:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                masked_gray = cv2.bitwise_and(gray, gray, mask=mask)

                if _prev_gray is None:
                    _prev_gray = masked_gray
                    _prev_pts = cv2.goodFeaturesToTrack(masked_gray, maxCorners=MAX_CORNERS, qualityLevel=0.01, minDistance=7)
                else:
                    if _prev_pts is None or len(_prev_pts) < MIN_FEATURES:
                        _prev_pts = cv2.goodFeaturesToTrack(masked_gray, maxCorners=MAX_CORNERS, qualityLevel=0.01, minDistance=7)
                        _prev_gray = masked_gray
                    else:
                        new_pts, status, err = cv2.calcOpticalFlowPyrLK(_prev_gray, masked_gray, _prev_pts, None)
                        if new_pts is not None and status is not None:
                            good_prev = _prev_pts[status.flatten() == 1]
                            good_new = new_pts[status.flatten() == 1]
                            if len(good_prev) >= MIN_FEATURES:
                                displacements = np.linalg.norm(good_new - good_prev, axis=1)
                                median_disp = float(np.median(displacements))
                                meters_moved = median_disp * PIXEL_TO_METER
                                vision_speed_m_s = meters_moved / dt
                        _prev_pts = cv2.goodFeaturesToTrack(masked_gray, maxCorners=MAX_CORNERS, qualityLevel=0.01, minDistance=7)
                        _prev_gray = masked_gray.copy()
            except Exception:
                vision_speed_m_s = 0.0

            vision_speed_cm_s = vision_speed_m_s * 100.0
            used_speed_cm_s = REAL_TIME_SPEED if REAL_TIME_SPEED > 0.01 else vision_speed_cm_s
            REAL_TIME_DISTANCE += (used_speed_cm_s / 100.0) * dt
            LAST_TIME = current_time

            telemetry = {
                "type": "telemetry",
                "data": {
                    "speed": round(float(used_speed_cm_s), 2),
                    "laneStatus": lane_status,
                    "steering_angle": round(float(angle), 2),
                    "jarakTempuh": round(float(REAL_TIME_DISTANCE), 3)
                }
            }

            raw_b64 = encode_image_to_base64(frame)
            proc_b64 = encode_image_to_base64(frame_overlay)

            data_raw = {"type": "image_raw", "data": raw_b64, "width": frame.shape[1], "height": frame.shape[0]}
            data_proc = {"type": "image_processed", "data": proc_b64, "width": frame.shape[1], "height": frame.shape[0]}

            info_y = 30
            cv2.putText(frame_overlay, f"Speed: {telemetry['data']['speed']:.2f} cm/s", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
            cv2.putText(frame_overlay, f"Angle: {telemetry['data']['steering_angle']:.2f} deg", (10, 20+info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            cv2.putText(frame_overlay, f"Jarak: {telemetry['data']['jarakTempuh']:.2f} m", (10, 20+2*info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.putText(frame_overlay, f"Lane: {telemetry['data']['laneStatus']}", (10, 20+3*info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

            cv2.imshow("Ijul ngedebug ('q' bwt Stop)", frame_overlay)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            try:
                await websocket.send(json.dumps(data_raw))
                await websocket.send(json.dumps(data_proc))
                await websocket.send(json.dumps(telemetry))
            except Exception as e:
                print("Warning: failed to send WS data:", e)

            loop_elapsed = time.time() - current_time
            await asyncio.sleep(max(0.0, (1.0 / TARGET_FPS) - loop_elapsed))

    except Exception as e:
        print("FATAL ERROR in main loop:", e)
    finally:
        CAP.release()
        cv2.destroyAllWindows()
        if udp_transport:
            udp_transport.close()
        try:
            if websocket and hasattr(websocket, "close"):
                await websocket.close()
        except Exception:
            pass


if __name__ == "__main__":
    try:
        asyncio.run(send_data())
    except KeyboardInterrupt:
        print("Stop sek kata ijul")
    except Exception as e:
        print(f"Error fatal di main: {e}")