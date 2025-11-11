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
from typing import Tuple, Optional

URL_KAMERA = "videotest.mp4" #http://10.7.101.181:8080/video
WS_SERVER_IP = "0.0.0.0"
WS_SERVER_PORT = 8080

UDP_LISTEN_IP = "0.0.0.0"
UDP_LISTEN_PORT = 50001

ESP32_IP = "192.168.4.1" #ganti sesuai ip esp
ESP32_PORT = 50002

TARGET_FPS = 30
PIXEL_TO_METER = 0.01
MIN_FEATURES = 8
MAX_CORNERS = 200
FRAME_WIDTH = 640
FRAME_HEIGHT= 480

CAP = cv2.VideoCapture(URL_KAMERA)
if not CAP.isOpened():
    print(f"ERROR: gaiso buka dari: {URL_KAMERA}")
    print("Working dir:", os.getcwd())
    print("File exists:", os.path.exists(URL_KAMERA))
    raise SystemExit(1)

# ret_test, test_frame = CAP.read()
# if ret_test:
#     FRAME_HEIGHT, FRAME_WIDTH = test_frame.shape[:2]
#     CAP.set(cv2.CAP_PROP_POS_FRAMES, 0)
# else:
#     FRAME_WIDTH, FRAME_HEIGHT = 640, 480
CAP.set(cv2.CAP_PROP_FPS, TARGET_FPS)
CAP.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
CAP.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

# CAP.set(cv2.CAP_PROP_FPS, TARGET_FPS)
# CAP.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
# CAP.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
# CAP.set(cv2.CAP_PROP_BUFFERSIZE, 1)

REAL_TIME_SPEED = 0.0
REAL_TIME_DISTANCE = 0.0
LAST_TIME = time.time()
# _prev_gray = None #nyimpen frame gray sebelumnya buat optical flow
# _prev_pts = None #nyimpen titik2 sebelumnya buat optical flow
#optical flow kurang kepake karena deteksi speed dari sensor
RUNNING = False
CURRENT_STEER = 0.0
CURRENT_POSITION = "unknown"
OBSTACLE = {"detected": False, "position": None, "distance_cm": None}

CLIENTS = set()
udp_cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def encode_image_to_base64(img_bgr, quality=30) -> str:
    _, buf = cv2.imencode(".jpg", img_bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return base64.b64encode(buf).decode("utf-8")

def send_udp_command_to_esp32(cmd_str: str):
    try:
        udp_cmd_sock.sendto(cmd_str.encode("utf-8"), (ESP32_IP, ESP32_PORT))
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
    lower_white = np.array([0, 150, 0])
    upper_white = np.array([255, 255, 255])
    mask = cv2.inRange(hls, lower_white, upper_white)

    kernel = np.ones((3,3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

    src, dst = get_perspective_transform_points(w, h)
    M = cv2.getPerspectiveTransform(src, dst)
    bev_mask = cv2.warpPerspective(mask, M, (w, h), flags=cv2.INTER_LINEAR)

    edges = cv2.Canny(bev_mask, 50, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=20, minLineLength=20, maxLineGap=10)

    overlay = frame_bgr.copy()
    avg_angle = 0.0
    lane_status = "Lost"
    if lines is not None and len(lines) > 0:
        lane_status = "Detected"
        angles = []
        bev_line_img = np.zeros_like(frame_bgr)
        for ln in lines:
            x1,y1,x2,y2 = ln.reshape(4)
            angles.append(calculate_angle_from_line(x1,y1,x2,y2))
            cv2.line(bev_line_img, (x1,y1), (x2,y2), (0,255,0), 3)
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
    frame_center = w/2.0
    if lane_mid < frame_center - w*0.05:
        return "left"
    elif lane_mid > frame_center + w*0.05:
        return "right"
    else:
        return "center"

def detect_obstacle_by_vision(bev_mask) -> Tuple[bool, Optional[str]]:
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
    if cx < w/2:
        return True, "left"
    else:
        return True, "right"


class SensorUDPProtocol(asyncio.DatagramProtocol):
    def datagram_received(self, data, addr):
        global REAL_TIME_SPEED, OBSTACLE
        txt = None
        try:
            txt = data.decode("utf-8").strip()
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
            print(f"[UDP SENSOR] parse error ({txt}): {e}")

async def udp_sensor_listener(host: str, port: int):
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

async def ws_recv_loop(ws):
    global RUNNING, REAL_TIME_DISTANCE, LAST_TIME
    try:
        async for raw in ws:
            try:
                msg = json.loads(raw)
            except:
                continue
            if msg.get("type") == "command":
                cmd = msg.get("cmd", "").lower()
                print(f"[WS CMD] {cmd}")
                if cmd == "start":
                    RUNNING = True
                    send_udp_command_to_esp32("CMD:START")
                elif cmd == "stop":
                    RUNNING = False
                    send_udp_command_to_esp32("CMD:STOP")
                elif cmd == "reset_distance":
                    REAL_TIME_DISTANCE = 0.0
                    LAST_TIME = time.time()
            # other messages may be handled here
    except websockets.exceptions.ConnectionClosed:
        pass
    except Exception as e:
        print(f"[WS recv error] {e}")

async def ws_sender_periodic(ws):
    img_interval = 1.0 / max(1, int(TARGET_FPS/2))
    last_img = 0.0
    try:
        while True:
            telemetry = {
                "type": "telemetry",
                "data": {
                    "steering_angle": round(float(CURRENT_STEER), 2),
                    "laneStatus": CURRENT_POSITION,
                    "speed": round(float(REAL_TIME_SPEED), 2),
                    # "jarakTempuh": round(float(REAL_TIME_DISTANCE),3),  # commented out by default
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
                return

            now = time.time()
            if now - last_img > img_interval:
                ret, frame = CAP.read()
                if ret:
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

async def ws_handler(ws, path=None):
    print(f"[WS] client connected: {ws.remote_address}, path={path}")
    CLIENTS.add(ws)
    sender = asyncio.create_task(ws_sender_periodic(ws))
    receiver = asyncio.create_task(ws_recv_loop(ws))
    try:
        await asyncio.wait([sender, receiver], return_when=asyncio.FIRST_COMPLETED)
    finally:
        CLIENTS.discard(ws)
        sender.cancel()
        receiver.cancel()
        print(f"[WS] client disconnected: {ws.remote_address}")

def decide_and_send_control(steering_angle: float, lane_pos: str, obstacle_info: dict):
    if not RUNNING:
        send_udp_command_to_esp32("CMD:STOP")
        return

    if lane_pos not in ("left", "right", "center"):
        send_udp_command_to_esp32("CMD:STOP")
        return

    if obstacle_info.get("detected"):
        pos = obstacle_info.get("position")
        dist = obstacle_info.get("distance_cm")
        if dist is not None and dist < 20:
            send_udp_command_to_esp32("CMD:STOP")
            return
        if pos == "right":
            send_udp_command_to_esp32(f"STEER:{-30.0:.2f}")
            send_udp_command_to_esp32("MOVE:20")
            return
        if pos == "left":
            send_udp_command_to_esp32(f"STEER:{30.0:.2f}")
            send_udp_command_to_esp32("MOVE:20")
            return
        send_udp_command_to_esp32("CMD:STOP")
        return

    steer = max(-45.0, min(45.0, steering_angle))
    if abs(steer) > 20:
        speed_cmd = 25.0
    elif abs(steer) > 10:
        speed_cmd = 35.0
    else:
        speed_cmd = 45.0
    send_udp_command_to_esp32(f"STEER:{steer:.2f}")
    send_udp_command_to_esp32(f"MOVE:{speed_cmd:.2f}")

async def main_loop():
    global CURRENT_STEER, CURRENT_POSITION, OBSTACLE, REAL_TIME_DISTANCE, LAST_TIME

    udp_transport = await udp_sensor_listener(UDP_LISTEN_IP, UDP_LISTEN_PORT)
    if udp_transport is None:
        print("[WARN] UDP sensor listener not available")

    try:
        while True:
            t0 = time.time()
            ret, frame = CAP.read()
            if not ret:
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

            y = 20
            cv2.putText(combined, f"Speed: {REAL_TIME_SPEED:.2f} cm/s", (10,y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2); y+=25
            cv2.putText(combined, f"Angle: {angle:.2f} deg", (10,y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2); y+=25
            cv2.putText(combined, f"Pos: {CURRENT_POSITION}", (10,y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2); y+=25
            if OBSTACLE["detected"]:
                cv2.putText(combined, f"Obs: {OBSTACLE['position']} {OBSTACLE['distance_cm']}", (10,y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

            cv2.imshow("Ijul ngedebug ('q' bwt Stop)", combined)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            telemetry = {
                "type":"telemetry",
                "data":{
                    "steering_angle": round(angle,2),
                    "laneStatus": CURRENT_POSITION,
                    "speed": round(REAL_TIME_SPEED,2),
                    # "jarakTempuh": round(REAL_TIME_DISTANCE,3),
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
 
async def main():
    server = await websockets.serve(ws_handler, WS_SERVER_IP, WS_SERVER_PORT)
    print(f"[WS] WebSocket server running on ws://{WS_SERVER_IP}:{WS_SERVER_PORT}")
    await main_loop()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Stop sek kata ijul")
    finally:
        CAP.release()
        cv2.destroyAllWindows()
