import asyncio
import websockets
import socket
import json
from typing import Any, Dict, Set

class SensorUDPProtocol(asyncio.DatagramProtocol):
    """Menerima data sensor dari ESP32 (UDP)"""
    def __init__(self, manager):
        self.manager = manager

    def datagram_received(self, data, addr):
        try:
            txt = data.decode("utf-8").strip()
            print(f"[UDP IN] Dari {addr}: {txt}") 
            dist = float(txt)

            # Update obstacle info
            self.manager.CURRENT_OBSTACLE_INFO = {
                "detected": dist < 50,    # di      anggap detected jika <50 cm
                "distance_cm": dist,
                "position": None          # nanti diinfo di main.py
            }
            # # format kecepatan
            # if txt.startswith("S:"):
            #     self.manager.REAL_TIME_SPEED = float(txt.split(":")[1])
            # # Format obstacle
            # elif txt.startswith("OBS:"):
            #     parts = txt.split(":")
            #     if len(parts) >= 3:
            #         side = parts[1].strip().upper()
            #         dist = float(parts[2].strip())
            #         self.manager.CURRENT_OBSTACLE_INFO = {
            #             "detected": True,
            #             "position": "left" if side == "L" else "right",
            #             "distance_cm": dist
            #         }
            # # format JSON (opsional)
            # else:
            #     j = json.loads(txt)
            #     if isinstance(j, dict):
            #         if "speed" in j:
            #             self.manager.REAL_TIME_SPEED = float(j["speed"])
            #         if "distance_obstacle" in j:
            #             self.manager.CURRENT_OBSTACLE_INFO["distance_cm"] = float(j["distance_obstacle"])
            #         if "obstacle_detected" in j:
            #             self.manager.CURRENT_OBSTACLE_INFO["detected"] = bool(j["obstacle_detected"])
            #         if "obstacle_position" in j:
            #             self.manager.CURRENT_OBSTACLE_INFO["position"] = j["obstacle_position"]
        except Exception as e:
            print(f"[UDP ERROR] {e}")


class CommunicationManager:
    """komunikasi UDP (ESP32) dan WebSocket (Base Station)"""

    def __init__(self, ws_port: int, udp_sensor_port: int, udp_control_ip: str, udp_control_port: int):
        self.WS_PORT = ws_port
        self.UDP_SENSOR_PORT = udp_sensor_port
        self.UDP_CONTROL_IP = udp_control_ip
        self.UDP_CONTROL_PORT = udp_control_port

        self.REAL_TIME_SPEED = 0.0
        self.REAL_TIME_DISTANCE = 0.0
        self.RUNNING = False
        self.CURRENT_OBSTACLE_INFO: Dict[str, Any] = {"detected": False, "position": None, "distance_cm": None}

        self.CLIENTS: Set[websockets.WebSocketServerProtocol] = set()

        # Socket UDP untuk kirim kontrol ke ESP32
        self.udp_cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_cmd_sock.setblocking(False)

    async def start_udp_listener(self):
        """Nyalain listener UDP dari ESP32."""
        loop = asyncio.get_event_loop()
        transport, _ = await loop.create_datagram_endpoint(
            lambda: SensorUDPProtocol(self),
            local_addr=("0.0.0.0", self.UDP_SENSOR_PORT)
        )
        print(f"[UDP] Listening on 0.0.0.0:{self.UDP_SENSOR_PORT}")
        return transport

    def send_control_command(self, steer: float, motor: float):
        """Kirim info sudut dan kecepatan motor dc ke ESP32"""
        #cmd = f"A:{round(steer,2)},M:{round(motor,2)}\n"
        cmd = f"{round(steer,2)},{round(motor,2)}\n"
        try:
            self.udp_cmd_sock.sendto(cmd.encode(), (self.UDP_CONTROL_IP, self.UDP_CONTROL_PORT))
            print(f"[UDP OUT] {cmd.strip()}")
            #await asyncio.sleep(0.5) #yaopo njir ngaruh ke FPS setelah di start
        except Exception as e:
            print(f"[UDP ERROR] {e}")

    #websocket
    async def ws_receiver_loop(self, ws):
        """START/STOP/RESET dari Base Station"""
        try:
            async for msg in ws:
                j = json.loads(msg)
                if j.get("type") == "control":
                    cmd = j.get("command", "").lower()
                    if cmd == "start":
                        self.RUNNING = True
                        print("[CMD] START diterima")
                    elif cmd == "stop":
                        self.RUNNING = False
                        print("[CMD] STOP diterima")
                        self.send_control_command(0.0, 0.0)
                    elif cmd == "reset_distance":
                        self.REAL_TIME_DISTANCE = 0.0
                        print("[CMD] RESET jarak tempuh")
        except websockets.exceptions.ConnectionClosed:
            print("[WS] Base Station terputus")
        except Exception as e:
            print(f"[WS ERROR] {e}")

    async def ws_handler(self, ws, path=None):
        """Handler koneksi Websocket"""
        print(f"[WS] Client connected: {ws.remote_address}")
        self.CLIENTS.add(ws)
        recv_task = asyncio.create_task(self.ws_receiver_loop(ws))
        try:
            await recv_task
        finally:
            self.CLIENTS.discard(ws)
            recv_task.cancel()
            print(f"[WS] Client disconnected: {ws.remote_address}")

    async def broadcast_telemetry(self, *payloads):
        """Kirim data telemetry ke Websocket (cuman payload dict yang valdi)"""
        if not self.CLIENTS:
            return
        for payload in payloads:
            try:
                if not isinstance(payload, dict):
                    print("[WARN] broadcast_telemetry: payload bukan dict, skip")
                    continue
                msg = json.dumps(payload)
                await asyncio.gather(*(client.send(msg) for client in self.CLIENTS))
            except Exception as e:
                print(f"[WS SEND ERROR] {e}")

    async def start_websocket_server(self):
        """nyalain server"""
        server = await websockets.serve(self.ws_handler, "0.0.0.0", self.WS_PORT)
        print(f"[WS] WebSocket server aktif di ws://0.0.0.0:{self.WS_PORT}")
        return server
