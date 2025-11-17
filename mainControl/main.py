import asyncio
import cv2
import time
import base64
from typing import Optional
from communication import CommunicationManager
from vision import VisionProcessor
from control import RobotController
from control import PID
import websockets

WS_SERVER_IP = "0.0.0.0"
WS_SERVER_PORT = 8080
URL_KAMERA = "videotest2.mp4" #http://10.7.101.58:8080/video http://10.234.118.128:8080/video http://10.234.118.128:8080/video
UDP_LISTEN_PORT = 50002
UDP_CONTROL_IP = "10.234.118.48" #ip esp32
UDP_CONTROL_PORT = 50002
TARGET_FPS = 30
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

#buat ngerjalanin partisi filenya
COMM: Optional[CommunicationManager] = None
VISION: Optional[VisionProcessor] = None
CONTROL: Optional[RobotController] = None
CAP = cv2.VideoCapture(URL_KAMERA)

if not CAP.isOpened():
    print(f"[ERROR] gaiso buka dari: {URL_KAMERA}")
    raise SystemExit(1)

CAP.set(cv2.CAP_PROP_BUFFERSIZE, 1)
CAP.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
CAP.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)


def encode_image_to_base64(image):
    """Konversi frame OpenCV (BGR) ke string base64"""
    if image is None:
        print("[ERROR] encode_image_to_base64: image kosong!")
        return ""
    try:
        success, buffer = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not success:
            print("[ERROR] encode_image_to_base64: gagal encode.")
            return ""
        return base64.b64encode(buffer).decode("utf-8")
    except Exception as e:
        print(f"[ERROR] encode_image_to_base64 gagal: {e}")
        return ""


async def lane_detection_handler(ws):
    '''Connect dan ngirim data ke websocket secara realtime'''
    global COMM, VISION, CONTROL

    COMM.REAL_TIME_DISTANCE = 0.0
    COMM.RUNNING = False
    COMM.LAST_TIME = time.time()

    print(f"[WS] Base Station connected: {ws.remote_address}")

    recv_task = asyncio.create_task(COMM.ws_receiver_loop(ws))
    COMM.CLIENTS.add(ws)

    try:
        while CAP.isOpened():
            t0 = time.time()
            ret, frame = CAP.read()
            if not ret:
                CAP.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue

            frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
            overlay, angle, lane_status, robot_position, bev = VISION.process_frame_for_lane_data(frame)

            target_steer, target_speed = 0.0, 0.0
            if COMM.RUNNING:
                target_steer, target_speed = CONTROL.decide_command(angle, lane_status, COMM.CURRENT_OBSTACLE_INFO)
                COMM.send_control_command(target_steer, target_speed)

            now = time.time()
            dt = now - COMM.LAST_TIME
            COMM.REAL_TIME_DISTANCE += (COMM.REAL_TIME_SPEED / 100.0) * dt
            COMM.LAST_TIME = now

            dist = COMM.CURRENT_OBSTACLE_INFO["distance_cm"]

            if dist is not None and dist < 50:
                final_obstacle_detected = True
                final_obstacle_distance = dist
                final_obstacle_position = robot_position  # samakan posisi obstacle dgn posisi robot (vision)
            else:
                final_obstacle_detected = False
                final_obstacle_distance = dist
                final_obstacle_position = None
                
            speed_actual = CONTROL.pwm_to_speed(target_speed)
            COMM.REAL_TIME_SPEED = speed_actual
            #data ke bs
            telemetry = {
                "type": "telemetry",
                "data": {
                    "steering_angle": round(target_steer, 2), #vision BEV
                    "laneStatus": lane_status, #vision
                    "robotPosition": robot_position, #vision 
                    "speed": round(COMM.REAL_TIME_SPEED, 2), #estimasi perhitungan dari kecepatan PWM motor DC
                    "jarakTempuh": round(COMM.REAL_TIME_DISTANCE, 3), #ngga ditampilin di bs sih sih
                    "obstacleDetected": final_obstacle_detected, #kalau ada obs distance dari STM32 = detected
                    "obstacleDistance": final_obstacle_distance, #dari STM32
                    "obstaclePosition": final_obstacle_position, #proses jika detected = samakan dengan robot Position
                },
            }

            raw = {
                    "type": "image_raw",
                    "data": encode_image_to_base64(frame),
                    "width": FRAME_WIDTH,
                    "height": FRAME_HEIGHT
                }

            proc = {
                    "type": "image_processed",
                    "data": encode_image_to_base64(overlay),
                    "width": FRAME_WIDTH,
                    "height": FRAME_HEIGHT
                }

            await COMM.broadcast_telemetry(telemetry, raw, proc)

            cv2.imshow("Ijul ngedebug HIBECI", overlay)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

            elapsed = time.time() - t0
            await asyncio.sleep(max(0.0, (1.0 / TARGET_FPS) - elapsed))

    except Exception as e:
        print(f"[ERROR] Loop gagal: {e}")
    finally:
        recv_task.cancel()
        COMM.CLIENTS.discard(ws)
        CAP.set(cv2.CAP_PROP_POS_FRAMES, 0)
        cv2.destroyAllWindows()
        COMM.send_control_command(0.0, 0.0)
        print("[SYSTEM] WS handler ditutup.")


async def main_orchestrator():
    '''ngirim data dan ngejalanin subprogram'''
    global COMM, VISION, CONTROL
    COMM = CommunicationManager(WS_SERVER_PORT, UDP_LISTEN_PORT, UDP_CONTROL_IP, UDP_CONTROL_PORT)
    VISION = VisionProcessor(FRAME_WIDTH, FRAME_HEIGHT)
    pid = PID(Kp=0.5, Ki=0.0, Kd=0.25)
    CONTROL = RobotController(pid)
    COMM.LAST_TIME = time.time()

    await COMM.start_udp_listener()
    ws_server = await websockets.serve(lane_detection_handler, WS_SERVER_IP, WS_SERVER_PORT)
    print(f"[SYSTEM] Semua subsistem siap di ws://{WS_SERVER_IP}:{WS_SERVER_PORT}")
    await ws_server.wait_closed()

if __name__ == "__main__":
    try:
        asyncio.run(main_orchestrator())
    except KeyboardInterrupt:
        print("\n[STOP] Stop sek kata ijul")
    except Exception as e:
        print(f"Error coy nde kene: {e}")
