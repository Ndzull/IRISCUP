import asyncio
import websockets
import cv2
import time
import base64
from typing import Optional

from communication import CommunicationManager   # Websocket, UDP In/Out
from vision import VisionProcessor               # Frame Processing, BEV (masih dipake untuk deteksi sudut)
from control import RobotController              # Decision making

#Host (piton)
WS_SERVER_IP = '0.0.0.0'
WS_SERVER_PORT = 8080
URL_KAMERA = 'videotest.mp4'
UDP_LISTEN_PORT = 50001
UDP_CONTROL_IP = '10.234.118.52' #ip esp32
UDP_CONTROL_PORT = 50002
TARGET_FPS = 20
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
DEFAULT_SPEED = 45.0

COMM: Optional[CommunicationManager] = None
VISION: Optional[VisionProcessor] = None
CONTROL: Optional[RobotController] = None
CAP = cv2.VideoCapture(URL_KAMERA)

if not CAP.isOpened():
    print(f"ERROR: gaiso buka dari: {URL_KAMERA}")
    raise SystemExit(1)

CAP.set(cv2.CAP_PROP_BUFFERSIZE, 1)
CAP.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
CAP.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

def encode_image_to_base64(img_bgr, quality=25) -> str:
    """Konversi gambar BGR ke Base64 JPG."""
    _, buf = cv2.imencode(".jpg", img_bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return base64.b64encode(buf).decode("utf-8")

#Handler Websocket
async def lane_detection_handler(websocket, path=None):
    """Dijalanin setiap Base Station terhubung."""
    global COMM, VISION, CONTROL

    COMM.REAL_TIME_DISTANCE = 0.0
    COMM.RUNNING = False
    COMM.LAST_TIME = time.time()

    print(f"[WS] Base Station connected from {websocket.remote_address}. Awaiting START...")

    receiver_task = asyncio.create_task(COMM.ws_receiver_loop(websocket))

    try:
        cv2.namedWindow("Ijul ngedebug HIBECI (q bwt stop)", cv2.WINDOW_AUTOSIZE)

        while CAP.isOpened():
            t0 = time.time()
            ret, frame = CAP.read()
            if not ret:
                #ulangi video kalau selesai
                if URL_KAMERA.lower().endswith(('.mp4', '.avi', '.mov', '.mkv')):
                    CAP.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    await asyncio.sleep(0.05)
                    continue
                else:
                    break

            if frame.shape[1] != FRAME_WIDTH or frame.shape[0] != FRAME_HEIGHT:
                frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))

            now = time.time()
            dt = max(now - COMM.LAST_TIME, 1e-6)

            #vision
            overlay_frame, angle, lane_status, robot_position, bev_mask = VISION.process_frame_for_lane_data(frame)

            #control
            target_steer, target_speed = 0.0, 0.0
            if COMM.RUNNING:
                target_steer, target_speed = CONTROL.decide_command(
                    angle, lane_status, COMM.CURRENT_OBSTACLE_INFO
                )

            #kirim ke esp32 (A,M)
            COMM.send_control_command(target_steer, target_speed)

            # --- ODOMETRI ---
            if COMM.REAL_TIME_SPEED > 0.01:
                COMM.REAL_TIME_DISTANCE += (COMM.REAL_TIME_SPEED / 100.0) * dt
            COMM.LAST_TIME = now

            # --- KIRIM DATA KE BASE STATION ---
            telemetry_data = {
                "type": "telemetry",
                "data": {
                    "steering_angle": round(float(target_steer), 2),
                    "laneStatus": lane_status,
                    "robotPosition": robot_position,
                    "speed": round(COMM.REAL_TIME_SPEED, 2),
                    "jarakTempuh": round(COMM.REAL_TIME_DISTANCE, 3),
                    "obstacle": COMM.CURRENT_OBSTACLE_INFO,
                    "running": COMM.RUNNING,
                },
            }
            #await COMM.broadcast_telemetry({"type": "telemetry", "data": telemetry_data})
            await COMM.broadcast_telemetry(telemetry_data)

            # --- KIRIM GAMBAR ---
            raw_b64 = encode_image_to_base64(frame, quality=25)
            proc_b64 = encode_image_to_base64(overlay_frame, quality=25)
            # await COMM.broadcast_telemetry({"type": "image_raw", "data": raw_b64})
            # await COMM.broadcast_telemetry({"type": "image_processed", "data": proc_b64})

            raw_payload = {"type": "image_raw", "data": raw_b64, "width": FRAME_WIDTH, "height": FRAME_HEIGHT}
            proc_payload = {"type": "image_processed", "data": proc_b64, "width": FRAME_WIDTH, "height": FRAME_HEIGHT}
            # await COMM.broadcast_telemetry(telemetry_data, raw_payload, proc_payload)
            await COMM.broadcast_telemetry(telemetry_data)
            await COMM.broadcast_telemetry(raw_payload)
            await COMM.broadcast_telemetry(proc_payload)


            # --- DEBUG LOKAL ---
            cv2.imshow("IRIS Local Debug (press q to stop)", overlay_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # kontrol FPS
            elapsed = time.time() - t0
            await asyncio.sleep(max(0.0, (1.0 / TARGET_FPS) - elapsed))

    except Exception as e:
        print(f"[FATAL ERROR] Handler Halted: {e}")
    finally:
        COMM.send_control_command(0.0, 0.0)
        CAP.set(cv2.CAP_PROP_POS_FRAMES, 0)
        cv2.destroyAllWindows()
        receiver_task.cancel()


#main function
async def main_orchestrator():
    global COMM, VISION, CONTROL

    # Inisialisasi komponen utama
    COMM = CommunicationManager(WS_SERVER_PORT, UDP_LISTEN_PORT, UDP_CONTROL_IP, UDP_CONTROL_PORT)
    VISION = VisionProcessor(FRAME_WIDTH, FRAME_HEIGHT)
    CONTROL = RobotController()
    COMM.LAST_TIME = time.time()

    # Mulai listener UDP sensor
    await COMM.start_udp_listener()

    # Jalankan WebSocket server
    ws_server = await websockets.serve(lane_detection_handler, WS_SERVER_IP, WS_SERVER_PORT)
    print(f"[WS] WebSocket server running on ws://{WS_SERVER_IP}:{WS_SERVER_PORT}")

    print("[SYSTEM] Semua subsistem siap. Menunggu koneksi Base Station...")
    await ws_server.wait_closed()


# =================================================================
# ENTRY POINT
# =================================================================
if __name__ == "__main__":
    try:
        asyncio.run(main_orchestrator())
    except KeyboardInterrupt:
        print("\n[System Stop] Dihentikan oleh pengguna.")
    except Exception as e:
        print(f"Error fatal di main: {e}")
