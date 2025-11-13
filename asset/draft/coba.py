#!/usr/bin/env python3
import os
# Force Qt platform if headless (do this before importing cv2 if needed)
if not os.environ.get("DISPLAY"):
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
else:
    os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

import asyncio
import websockets
import cv2
import numpy as np
import base64
import json
import math
import time
import socket
from typing import Tuple, Optional

# ===================== KONFIGURASI ======================
URL_KAMERA = "videotest.mp4"  # atau gunakan 0 untuk webcam
WS_SERVER_IP = "0.0.0.0"
WS_SERVER_PORT = 8080

UDP_LISTEN_IP = "0.0.0.0"
UDP_LISTEN_PORT = 50001

ESP32_IP = "10.234.118.52"
ESP32_PORT = 50002

TARGET_FPS = 30
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

USE_GUI = bool(os.environ.get("DISPLAY"))

# =========================================================
CAP = cv2.VideoCapture(URL_KAMERA)
if not CAP.isOpened():
    print(f"[ERROR] Tidak bisa buka video/kamera dari: {URL_KAMERA}")
    raise SystemExit(1)

# Try to set resolution & fps; we will resize in code if driver ignores settings
CAP.set(cv2.CAP_PROP_FPS, TARGET_FPS)
CAP.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
CAP.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

actual_width = int(CAP.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_height = int(CAP.get(cv2.CAP_PROP_FRAME_HEIGHT))
if actual_width != FRAME_WIDTH or actual_height != FRAME_HEIGHT:
    print(f"[WARN] Failed to set resolution to {FRAME_WIDTH}x{FRAME_HEIGHT}. Actual: {actual_width}x{actual_height}. Akan di-resize manual.")

# =========================================================
REAL_TIME_SPEED = 0.0
REAL_TIME_DISTANCE = 0.0
LAST_TIME = time.time()
RUNNING = False
CURRENT_STEER = 0.0
CURRENT_POSITION = "unknown"
OBSTACLE = {"detected": False, "position": None, "distance_cm": None}

CLIENTS = set()  # set of websocket connections

# UDP socket for sending commands to ESP32 (one socket reused)
udp_cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
udp_cmd_sock.settimeout(0.2)

# =========================================================
def encode_image_to_base64(img_bgr, quality=30) -> str:
    _, buf = cv2.imencode(".jpg", img_bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return base64.b64encode(buf).decode("utf-8")

def send_udp_command_to_esp32(cmd_str: str):
    try:
        udp_cmd_sock.sendto(cmd_str.encode("utf-8"), (ESP32_IP, ESP32_PORT))
        print(f"[UDP CMD] -> {ESP32_IP}:{ESP32_PORT}  {cmd_str}")
    except Exception as e:
        print(f"[WARN] UDP send failed: {e}")

def get_perspective_transform_points(w: int, h: int) -> Tuple[np.ndarray, np.ndarray]:
    src = np.float32([
        [w * 0.40, h * 0.60],
        [w * 0.60, h * 0.60],
        [w * 0.10, h * 0.95],
        [w * 0.90, h * 0.95]
    ])
    offset = w * 0.2
    dst = np.float32([
        [offset, 0], [w - offset, 0],
        [offset, h], [w - offset, h]
    ])
    return src, dst

def calculate_angle_from_line(x1, y1, x2, y2) -> float:
    if abs(x2 - x1) < 1e-3:
        return 90.0
    grad = (y2 - y1) / (x2 - x1)
    angle = math.degrees(math.atan(grad))
    return max(-90.0, min(90.0, angle))

def detect_lane_bev_and_angle(frame_bgr):
    h, w = frame_bgr.shape[:2]
    hls = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HLS)
    mask = cv2.inRange(hls, np.array([0, 150, 0]), np.array([255, 255, 255]))

    kernel = np.ones((3,3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

    src, dst = get_perspective_transform_points(w, h)
    M = cv2.getPerspectiveTransform(src, dst)
    bev_mask = cv2.warpPerspective(mask, M, (w, h))

    edges = cv2.Canny(bev_mask, 50, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 20, minLineLength=20, maxLineGap=10)

    overlay = frame_bgr.copy()
    avg_angle = 0.0
    lane_status = "Lost"

    if lines is not None and len(lines) > 0:
        lane_status = "Detected"
        angles = []
        bev_line_img = np.zeros_like(frame_bgr)
        for ln in lines:
            x1, y1, x2, y2 = ln.reshape(4)
            angles.append(calculate_angle_from_line(x1, y1, x2, y2))
            cv2.line(bev_line_img, (x1, y1), (x2, y2), (0,255,0), 3)
        avg_angle = float(np.mean(angles))
        M_inv = cv2.getPerspectiveTransform(dst, src)
        warped = cv2.warpPerspective(bev_line_img, M_inv, (w,h))
        overlay = cv2.addWeighted(overlay, 1.0, warped, 1.0, 0)

    bev_bgr = cv2.cvtColor(bev_mask, cv2.COLOR_GRAY2BGR)
    combined = np.concatenate((overlay, bev_bgr), axis=1)
    return combined, avg_angle, lane_status, overlay, bev_mask

def estimate_robot_lane_position(overlay_frame) -> str:
    h, w = overlay_frame.shape[:2]
    gray = cv2.cvtColor(overlay_frame, cv2.COLOR_BGR2GRAY)
    _, th = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    lower = th[int(h*0.5):, :]
    cols = np.where(np.sum(lower, axis=0) > 0)[0]
    if len(cols) < 10:
        return "unknown"
    leftmost = cols.min()
    rightmost = cols.max()
    lane_mid = (leftmost + rightmost) / 2.0
    frame_center = w / 2.0
    if lane_mid < frame_center - w * 0.05:
        return "left"
    elif lane_mid > frame_center + w * 0.05:
        return "right"
    return "center"

def detect_obstacle_by_vision(bev_mask):
    h, w = bev_mask.shape[:2]
    inv = cv2.bitwise_not(bev_mask)
    roi = inv[int(h*0.6):, :]
    contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return False, None
    c = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < 200:
        return False, None
    M = cv2.moments(c)
    if M["m00"] == 0:
        return False, None
    cx = int(M["m10"]/M["m00"])
    return True, "left" if cx < w/2 else "right"

# =========================================================
class SensorUDPProtocol(asyncio.DatagramProtocol):
    def datagram_received(self, data, addr):
        global REAL_TIME_SPEED, OBSTACLE
        try:
            txt = data.decode("utf-8", errors="ignore").strip()
            # Debug print reduced to avoid too much spam; uncomment if needed
            # print(f"[UDP SENSOR] from {addr}: {txt}")
            if txt.startswith("S:"):
                try:
                    REAL_TIME_SPEED = float(txt.split(":")[1])
                except:
                    pass
            elif txt.startswith("OBS:"):
                parts = txt.split(":")
                if len(parts) >= 3:
                    pos = parts[1].upper()
                    try:
                        dist = float(parts[2])
                    except:
                        dist = None
                    OBSTACLE["detected"] = True
                    OBSTACLE["position"] = "left" if pos == "L" else ("right" if pos == "R" else "center")
                    OBSTACLE["distance_cm"] = dist
            else:
                # accept JSON style sensor packets too
                try:
                    j = json.loads(txt)
                    if isinstance(j, dict):
                        if "speed" in j:
                            REAL_TIME_SPEED = float(j.get("speed", REAL_TIME_SPEED))
                        if "obstacle_detected" in j:
                            OBSTACLE["detected"] = bool(j.get("obstacle_detected"))
                        if "obstacle_position" in j and j.get("obstacle_position"):
                            OBSTACLE["position"] = j.get("obstacle_position")
                        if "distance_obstacle" in j:
                            try:
                                OBSTACLE["distance_cm"] = float(j.get("distance_obstacle"))
                            except:
                                OBSTACLE["distance_cm"] = None
                except Exception:
                    pass
        except Exception as e:
            print(f"[UDP SENSOR] parse error: {e}")

async def udp_sensor_listener(host: str = UDP_LISTEN_IP, port: int = UDP_LISTEN_PORT):
    loop = asyncio.get_event_loop()
    try:
        transport, protocol = await loop.create_datagram_endpoint(
            lambda: SensorUDPProtocol(),
            local_addr=(host, port)
        )
        print(f"[UDP SENSOR] listening on {host}:{port}")
        return transport
    except Exception as e:
        print(f"[UDP SENSOR] cannot bind {host}:{port}: {e}")
        return None

# =========================================================
async def broadcast_json(data: dict):
    if not CLIENTS:
        return
    msg = json.dumps(data)
    to_remove = []
    for ws in list(CLIENTS):
        try:
            await ws.send(msg)
        except Exception:
            to_remove.append(ws)
    for ws in to_remove:
        CLIENTS.discard(ws)

# Flexible WebSocket receiver: accepts both "command/cmd" and "control/command"
async def ws_recv_loop(ws):
    global RUNNING, REAL_TIME_DISTANCE, LAST_TIME
    async for raw in ws:
        try:
            print(f"[WS RAW] {raw}")
            try:
                msg = json.loads(raw)
            except:
                print("[WS] invalid json")
                continue

            if msg.get("type") in ["command", "control"]:
                # accept either 'cmd' or 'command'
                cmd = msg.get("cmd") if msg.get("cmd") is not None else msg.get("command", "")
                if not isinstance(cmd, str):
                    cmd = str(cmd)
                cmd = cmd.lower().strip()

                print(f"[WS CMD] {cmd}")
                if cmd == "start":
                    RUNNING = True
                    print("[WS ACTION] RUNNING=True")
                    send_udp_command_to_esp32("CMD:START")
                elif cmd == "stop":
                    RUNNING = False
                    print("[WS ACTION] RUNNING=False")
                    send_udp_command_to_esp32("CMD:STOP")
                elif cmd == "reset_distance":
                    REAL_TIME_DISTANCE = 0.0
                    LAST_TIME = time.time()
                    print("[WS ACTION] Distance reset")
                else:
                    print(f"[WS] unknown control command: {cmd}")

            elif msg.get("type") == "telemetry_request":
                # optional: respond with current telemetry snapshot
                telemetry = {
                    "type": "telemetry",
                    "data": {
                        "steering_angle": round(float(CURRENT_STEER), 2),
                        "laneStatus": CURRENT_POSITION,
                        "speed": round(float(REAL_TIME_SPEED), 2),
                        "obstacle": OBSTACLE,
                        "running": RUNNING
                    }
                }
                try:
                    await ws.send(json.dumps(telemetry))
                except:
                    pass
            else:
                print(f"[WS] unknown message type: {msg.get('type')}")
        except Exception as e:
            print(f"[WS recv error] {e}")

async def ws_sender_periodic(ws):
    """Periodically send telemetry and images to this websocket"""
    img_interval = 1.0 / max(1, int(TARGET_FPS))
    last_img = 0.0
    try:
        while True:
            telemetry = {
                "type": "telemetry",
                "data": {
                    "steering_angle": round(float(CURRENT_STEER), 2),
                    "laneStatus": CURRENT_POSITION,
                    "speed": round(float(REAL_TIME_SPEED), 2),
                    "obstacle": {
                        "detected": OBSTACLE.get("detected", False),
                        "position": OBSTACLE.get("position"),
                        "distance": OBSTACLE.get("distance_cm")
                    },
                    "running": RUNNING
                }
            }

            try:
                await ws.send(json.dumps(telemetry))
            except Exception:
                # connection might be closed
                return

            now = time.time()
            if now - last_img > img_interval:
                # read a frame in a thread to avoid blocking event loop
                ret, frame = await asyncio.to_thread(CAP.read)
                if ret and frame is not None:
                    if frame.shape[1] != FRAME_WIDTH or frame.shape[0] != FRAME_HEIGHT:
                        frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
                    combined, _, _, _, _ = detect_lane_bev_and_angle(frame)
                    raw_b64 = encode_image_to_base64(frame, quality=25)
                    proc_b64 = encode_image_to_base64(combined, quality=25)
                    try:
                        await ws.send(json.dumps({"type":"image_raw","data":raw_b64,"width":frame.shape[1],"height":frame.shape[0]}))
                        await ws.send(json.dumps({"type":"image_processed","data":proc_b64,"width":combined.shape[1],"height":combined.shape[0]}))
                    except Exception:
                        return
                last_img = now
            await asyncio.sleep(0.2)
    except websockets.exceptions.ConnectionClosed:
        return
    except Exception as e:
        print(f"[WS sender error] {e}")
        return

# Handler supports both websockets versions (path optional)
async def ws_handler(ws, path=None):
    print(f"[WS CONNECT] {ws.remote_address}, path={getattr(ws, 'path', path)}")
    CLIENTS.add(ws)
    sender = asyncio.create_task(ws_sender_periodic(ws))
    receiver = asyncio.create_task(ws_recv_loop(ws))
    try:
        await asyncio.wait([sender, receiver], return_when=asyncio.FIRST_COMPLETED)
    finally:
        CLIENTS.discard(ws)
        sender.cancel()
        receiver.cancel()
        print(f"[WS DISCONNECT] {ws.remote_address}")

def decide_and_send_control(angle, lane_pos, obstacle):
    if not RUNNING:
        send_udp_command_to_esp32("CMD:STOP")
        return

    if obstacle.get("detected") and obstacle.get("distance_cm", 999) < 20:
        send_udp_command_to_esp32("CMD:STOP")
        return

    steer = max(-45.0, min(45.0, angle))
    speed = 45.0 if abs(steer) < 10 else 35.0 if abs(steer) < 20 else 25.0
    send_udp_command_to_esp32(f"STEER:{steer:.2f}")
    send_udp_command_to_esp32(f"MOVE:{speed:.2f}")

async def main_loop():
    global CURRENT_STEER, CURRENT_POSITION, OBSTACLE, REAL_TIME_DISTANCE, LAST_TIME, REAL_TIME_SPEED

    udp_transport = await udp_sensor_listener()
    if udp_transport is None:
        print("[WARN] UDP sensor listener not available")

    print("[MAIN LOOP] started")
    try:
        while True:
            t0 = time.time()
            # Read frame in thread
            ret, frame = await asyncio.to_thread(CAP.read)
            if not ret or frame is None:
                # if file video, rewind; else small sleep
                if URL_KAMERA.lower().endswith(('.mp4', '.avi', '.mov', '.mkv')):
                    CAP.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    await asyncio.sleep(0.05)
                    continue
                else:
                    await asyncio.sleep(0.05)
                    continue

            if frame.shape[1] != FRAME_WIDTH or frame.shape[0] != FRAME_HEIGHT:
                frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))

            combined, angle, lane_status, overlay, bev_mask = detect_lane_bev_and_angle(frame)
            CURRENT_STEER = angle
            CURRENT_POSITION = estimate_robot_lane_position(overlay)

            vis_detect, vis_pos = detect_obstacle_by_vision(bev_mask)
            if not OBSTACLE["detected"] and vis_detect:
                OBSTACLE["detected"] = True
                OBSTACLE["position"] = vis_pos
                OBSTACLE["distance_cm"] = None

            decide_and_send_control(angle, CURRENT_POSITION if lane_status == "Detected" else "unknown", OBSTACLE)

            now = time.time()
            dt = max(now - LAST_TIME, 1e-6)
            REAL_TIME_DISTANCE += (REAL_TIME_SPEED / 100.0) * dt
            LAST_TIME = now

            if USE_GUI:
                y = 20
                cv2.putText(combined, f"Speed: {REAL_TIME_SPEED:.2f} cm/s", (10,y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2); y+=25
                cv2.putText(combined, f"Angle: {angle:.2f} deg", (10,y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2); y+=25
                cv2.putText(combined, f"Pos: {CURRENT_POSITION}", (10,y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2); y+=25
                if OBSTACLE["detected"]:
                    cv2.putText(combined, f"Obs: {OBSTACLE['position']} {OBSTACLE['distance_cm']}", (10,y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
                cv2.imshow("IJUL DEBUG", combined)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            # broadcast telemetry to all clients
            telemetry = {
                "type":"telemetry",
                "data":{
                    "steering_angle": round(angle,2),
                    "laneStatus": CURRENT_POSITION,
                    "speed": round(REAL_TIME_SPEED,2),
                    "obstacle": OBSTACLE,
                    "running": RUNNING
                }
            }
            await broadcast_json(telemetry)

            elapsed = time.time() - t0
            await asyncio.sleep(max(0.0, (1.0 / TARGET_FPS) - elapsed))
    finally:
        if udp_transport:
            udp_transport.close()

# ================= ENTRYPOINT =================
async def main():
    server = await websockets.serve(ws_handler, WS_SERVER_IP, WS_SERVER_PORT, ping_interval=30, ping_timeout=10, max_size=8*1024*1024)
    print(f"[WS] WebSocket server running on ws://{WS_SERVER_IP}:{WS_SERVER_PORT}")
    # Wait until a client connects and sends "start"
    while True:
        if CLIENTS:
            # at least one client connected; now wait for RUNNING to become True
            print("[WAIT] Client connected, waiting for START command...")
            while not RUNNING:
                await asyncio.sleep(0.5)
            # when RUNNING True start main loop (this call blocks)
            await main_loop()
            # when main_loop exits, reset RUNNING (so operator must re-start)
            print("[MAIN] main_loop exited, returning to wait state")
            await asyncio.sleep(0.5)
        else:
            print("[WAIT] Menunggu client WebSocket connect...")
            await asyncio.sleep(1)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[EXIT] Server stopped by user.")
    finally:
        try:
            CAP.release()
        except:
            pass
        if USE_GUI:
            try:
                cv2.destroyAllWindows()
            except:
                pass
