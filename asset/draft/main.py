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

# =================== KONFIGURASI =================== #
URL_KAMERA = 'videotest.mp4'
URI_BS_WS = "ws://localhost:8080"
UDP_LISTEN_IP = '0.0.0.0'
UDP_LISTEN_PORT = 50001
ESP32_IP = "192.168.4.1"       # Ganti sesuai alamat ESP32
ESP32_PORT = 60000             # Port UDP ESP32
TARGET_FPS = 30
PIXEL_TO_METER = 0.01
MIN_FEATURES = 8
MAX_CORNERS = 80
# =================================================== #

CAP = cv2.VideoCapture(URL_KAMERA)
if not CAP.isOpened():
    print(f"ERROR: Gagal buka kamera dari {URL_KAMERA}")
    exit()

CAP.set(cv2.CAP_PROP_FPS, TARGET_FPS)
CAP.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
CAP.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

REAL_TIME_SPEED = 0.0
REAL_TIME_DISTANCE = 0.0
_prev_gray = None
_prev_pts = None
ROBOT_RUNNING = False
OBSTACLE_DATA = {"status": "none", "distance": 999, "position": "center"}
ESP32_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# =================== LANE & BEV FUNGSIONAL =================== #
def calculate_angle_from_line(x1, y1, x2, y2):
    if abs(x2 - x1) < 1e-3:
        return 90.0
    gradien = (y2 - y1) / (x2 - x1)
    return max(-90, min(90, math.degrees(math.atan(gradien))))

def get_perspective_transform_points(lebar, tinggi):
    src = np.float32([
        [lebar * 0.40, tinggi * 0.60],
        [lebar * 0.60, tinggi * 0.60],
        [lebar * 0.10, tinggi * 0.95],
        [lebar * 0.90, tinggi * 0.95]
    ])
    offset = lebar * 0.2
    dst = np.float32([
        [offset, 0], [lebar - offset, 0],
        [offset, tinggi], [lebar - offset, tinggi]
    ])
    return src, dst

def detect_lane_and_overlay(frame):
    h, w = frame.shape[:2]
    hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
    lower_white = np.array([0, 150, 0])
    upper_white = np.array([255, 255, 255])
    mask = cv2.inRange(hls, lower_white, upper_white)

    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    src, dst = get_perspective_transform_points(w, h)
    M = cv2.getPerspectiveTransform(src, dst)
    bev_mask = cv2.warpPerspective(mask, M, (w, h))
    edges = cv2.Canny(bev_mask, 50, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 20, minLineLength=20, maxLineGap=10)

    overlay = frame.copy()
    avg_angle, lane_status = 0.0, "Lost"
    robot_pos = "Center"

    if lines is not None and len(lines) > 0:
        lane_status = "Detected"
        angles, x_positions = [], []
        M_inv = cv2.getPerspectiveTransform(dst, src)
        bev_line_image = np.zeros_like(frame)

        for ln in lines:
            x1, y1, x2, y2 = ln.reshape(4)
            angles.append(calculate_angle_from_line(x1, y1, x2, y2))
            x_positions.extend([x1, x2])
            cv2.line(bev_line_image, (x1, y1), (x2, y2), (0,255,0), 3)

        avg_angle = float(np.mean(angles))
        warped_line = cv2.warpPerspective(bev_line_image, M_inv, (w, h))
        overlay = cv2.addWeighted(overlay, 1.0, warped_line, 1.0, 0)

        # Tentukan posisi robot (kiri/kanan)
        mean_x = np.mean(x_positions)
        if mean_x < w/2 - 40:
            robot_pos = "Left Lane"
        elif mean_x > w/2 + 40:
            robot_pos = "Right Lane"
        else:
            robot_pos = "Center Lane"

    bev_mask_bgr = cv2.cvtColor(bev_mask, cv2.COLOR_GRAY2BGR)
    combined_debug_frame = np.concatenate((overlay, bev_mask_bgr), axis=1)
    return combined_debug_frame, avg_angle, lane_status, robot_pos, mask
# ============================================================= #

# =================== OBSTACLE DATA =================== #
class UDPReceiverProtocol(asyncio.DatagramProtocol):
    def datagram_received(self, data, addr):
        global OBSTACLE_DATA
        try:
            msg = data.decode().strip()
            if msg.startswith("OBS:"):
                parts = msg.split(":")
                OBSTACLE_DATA["status"] = "detected" if float(parts[1]) < 100 else "none"
                OBSTACLE_DATA["distance"] = float(parts[1])
                OBSTACLE_DATA["position"] = parts[2] if len(parts) > 2 else "center"
        except Exception:
            pass

async def udp_listen_obstacle(host, port):
    loop = asyncio.get_event_loop()
    await loop.create_datagram_endpoint(lambda: UDPReceiverProtocol(), local_addr=(host, port))
# ==================================================== #

# =================== ENCODE GAMBAR =================== #
def encode_image_to_base64(img_bgr, quality=25):
    _, buf = cv2.imencode('.jpg', img_bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return base64.b64encode(buf).decode('utf-8')
# ===================================================== #

# =================== KONTROL ROBOT =================== #
def send_udp_command(cmd):
    try:
        ESP32_SOCKET.sendto(cmd.encode(), (ESP32_IP, ESP32_PORT))
    except Exception as e:
        print("Error sending UDP command:", e)
# ==================================================== #

# =================== LOOP UTAMA =================== #
async def send_data():
    global _prev_gray, _prev_pts, ROBOT_RUNNING
    asyncio.create_task(udp_listen_obstacle(UDP_LISTEN_IP, UDP_LISTEN_PORT))

    try:
        ws = await websockets.connect(URI_BS_WS)
        print(f"Connected to Base Station: {URI_BS_WS}")
    except Exception:
        print("Warning: WS gagal, offline mode aktif")
        class DummyWS:
            async def send(self, data): pass
            async def recv(self): return json.dumps({})
        ws = DummyWS()

    # async def listen_bs():
    #     nonlocal ROBOT_RUNNING
    #     async for msg in ws:
    #         try:
    #             data = json.loads(msg)
    #             if data.get("command") == "start":
    #                 ROBOT_RUNNING = True
    #                 send_udp_command("CMD:FORWARD:0")
    #             elif data.get("command") == "stop":
    #                 ROBOT_RUNNING = False
    #                 send_udp_command("CMD:STOP")
    #         except Exception:
    #             pass

    #asyncio.create_task(listen_bs())

    cv2.namedWindow("Ijul Debug ('q' = Stop)", cv2.WINDOW_NORMAL)
    last_time = time.time()

    while CAP.isOpened():
        ret, frame = CAP.read()
        if not ret:
            break
        frame = cv2.resize(frame, (640, 480))
        now = time.time()
        dt = now - last_time
        last_time = now

        frame_overlay, angle, lane_status, robot_pos, mask = detect_lane_and_overlay(frame)

        # Deteksi obstacle (visual)
        obstacle_status = OBSTACLE_DATA["status"]
        obstacle_distance = OBSTACLE_DATA["distance"]
        obstacle_pos = OBSTACLE_DATA["position"]

        # Kontrol otomatis
        if ROBOT_RUNNING:
            if lane_status == "Lost":
                send_udp_command("CMD:STOP")
                ROBOT_RUNNING = False
            elif obstacle_status == "detected":
                if obstacle_pos == "left":
                    send_udp_command("CMD:TURN_RIGHT")
                elif obstacle_pos == "right":
                    send_udp_command("CMD:TURN_LEFT")
                else:
                    send_udp_command("CMD:STOP")
            else:
                send_udp_command(f"CMD:FORWARD:{angle:.2f}")

        # Tampilkan overlay info
        cv2.putText(frame_overlay, f"Steering: {angle:.2f}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
        cv2.putText(frame_overlay, f"Lane: {lane_status}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
        cv2.putText(frame_overlay, f"Pos: {robot_pos}", (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
        cv2.putText(frame_overlay, f"Obs: {obstacle_status} {obstacle_distance:.1f}cm {obstacle_pos}", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,128,255), 2)
        cv2.imshow("Ijul Debug ('q' = Stop)", frame_overlay)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # Kirim ke Base Station
        telemetry = {
            "type": "telemetry",
            "data": {
                "steering_angle": round(angle, 2),
                "laneStatus": lane_status,
                "robotPosition": robot_pos,
                "obstacle": OBSTACLE_DATA
            }
        }

        try:
            await ws.send(json.dumps(telemetry))
        except:
            pass

        await asyncio.sleep(max(0.0, (1.0/TARGET_FPS) - (time.time()-now)))

    CAP.release()
    cv2.destroyAllWindows()
# ===================================================== #

if __name__ == "__main__":
    try:
        asyncio.run(send_data())
    except KeyboardInterrupt:
        print("Stop oleh user (Ctrl+C)")
