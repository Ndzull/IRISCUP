import asyncio
import websockets
import socket
import json
from typing import Any, Dict, Set, Optional


class SensorUDPProtocol(asyncio.DatagramProtocol):
    """Nerima data sensor (speed, obstacle) dari ESP32 via UDP."""
    def __init__(self, manager):
        self.manager = manager

    def datagram_received(self, data, addr):
        try:
            txt = data.decode("utf-8").strip()

            # Format kecepatan: S:0.50
            if txt.startswith("S:"):
                self.manager.REAL_TIME_SPEED = float(txt.split(":")[1])

            # Format obstacle: OBS:L:30.5 atau OBS:R:25.0
            elif txt.startswith("OBS:"):
                parts = txt.split(":")
                if len(parts) >= 3:
                    pos_code = parts[1].strip().upper()
                    dist_raw = parts[2].strip()

                    # cek apakah angka valid
                    try:
                        dist = float(dist_raw)
                    except ValueError:
                        dist = None

                    if dist is not None:
                        self.manager.CURRENT_OBSTACLE_INFO = {
                            "detected": True,
                            "position": "left" if pos_code == "L" else "right",
                            "distance_cm": dist,
                        }
                    elif dist == 0.0:
                        self.manager.CURRENT_OBSTACLE_INFO = {
                            "detected": False,
                            "position": None,
                            "distance_cm": None,
                        }

            # Format alternatif JSON
            else:
                j = json.loads(txt)
                if isinstance(j, dict):
                    if "speed" in j:
                        self.manager.REAL_TIME_SPEED = float(j["speed"])
                    if "obstacle_detected" in j:
                        self.manager.CURRENT_OBSTACLE_INFO["detected"] = bool(j["obstacle_detected"])
                    if "obstacle_position" in j:
                        self.manager.CURRENT_OBSTACLE_INFO["position"] = j["obstacle_position"]
                    if "distance_obstacle" in j:
                        try:
                            self.manager.CURRENT_OBSTACLE_INFO["distance_cm"] = float(j["distance_obstacle"])
                        except:
                            self.manager.CURRENT_OBSTACLE_INFO["distance_cm"] = None

        except Exception as e:
            print(f"[UDP ERROR] {e}")


class CommunicationManager:
    """Mengatur UDP sensor (ESP32 → PC), UDP kontrol (PC → ESP32), dan WebSocket (ke Base Station)."""

    def __init__(self, ws_port: int, udp_sensor_port: int, udp_control_ip: str, udp_control_port: int):
        # --- konfigurasi jaringan
        self.WS_PORT = ws_port
        self.UDP_SENSOR_PORT = udp_sensor_port
        self.UDP_CONTROL_IP = udp_control_ip
        self.UDP_CONTROL_PORT = udp_control_port

        # --- status telemetry
        self.REAL_TIME_SPEED = 0.0
        self.REAL_TIME_DISTANCE = 0.0
        self.RUNNING = False
        self.CURRENT_OBSTACLE_INFO: Dict[str, Any] = {"detected": False, "position": None, "distance_cm": None}

        # --- websocket client set
        self.CLIENTS: Set[websockets.WebSocketServerProtocol] = set()

        # --- socket UDP untuk kirim kontrol ke ESP32
        self.udp_cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_cmd_sock.setblocking(False)

    # -------------------------------------------------------------
    # UDP: Kirim perintah ke ESP32
    # -------------------------------------------------------------
    def send_control_command(self, steer_angle: float, motor_constant: float):
        """Kirim perintah kontrol motor ke ESP32 via UDP."""
        cmd_str = f"A:{round(steer_angle, 2)},M:{round(motor_constant, 2)}\n"
        try:
            self.udp_cmd_sock.sendto(cmd_str.encode("utf-8"), (self.UDP_CONTROL_IP, self.UDP_CONTROL_PORT))
            print(f"[UDP CMD OUT] {cmd_str.strip()}")
        except Exception as e:
            print(f"[WARN] UDP send failed: {e}")

    # -------------------------------------------------------------
    # UDP: Listener sensor dari ESP32
    # -------------------------------------------------------------
    async def udp_sensor_listener(self):
        """Menyalakan listener UDP untuk menerima data sensor dari ESP32."""
        loop = asyncio.get_event_loop()
        try:
            transport, _ = await loop.create_datagram_endpoint(
                lambda: SensorUDPProtocol(self),
                local_addr=("0.0.0.0", self.UDP_SENSOR_PORT)
            )
            print(f"[UDP] Listening on port {self.UDP_SENSOR_PORT}")
            return transport
        except Exception as e:
            print(f"[FATAL] Failed to start UDP listener: {e}")

    # alias agar kompatibel dengan kode lama
    async def start_udp_listener(self):
        """Alias untuk kompatibilitas kode lama."""
        return await self.udp_sensor_listener()

    # -------------------------------------------------------------
    # WEBSOCKET: kirim telemetry ke Base Station
    # -------------------------------------------------------------
    async def broadcast_telemetry(self, *payloads):
        """
        Kirim satu atau lebih payload (dict) ke semua WebSocket clients.
        Contoh pemakaian:
            await comm.broadcast_telemetry(telemetry_dict)
            await comm.broadcast_telemetry(telemetry_dict, raw_image_dict, proc_image_dict)
        """
        # kompatibilitas: cari atribut clients yang ada ('CLIENTS' atau 'clients')
        clients = getattr(self, "CLIENTS", None) or getattr(self, "clients", None)
        if clients is None:
            print("[BCAST WARN] No client set attribute found on CommunicationManager.")
            return

        if not clients:
            # tidak ada client terhubung — logging ringan, jangan spam
            # hanya satu baris log setiap pemanggilan
            print("[BCAST] no clients connected — skipping broadcast")
            return

        # Untuk setiap payload, kirim ke semua client
        for payload in payloads:
            if not isinstance(payload, dict):
                # skip non-dict payloads (safety)
                print("[BCAST WARN] payload bukan dict, skip")
                continue
            msg = json.dumps(payload)
            to_remove = []
            sent_count = 0
            for ws in list(clients):
                try:
                    await ws.send(msg)
                    sent_count += 1
                except Exception:
                    to_remove.append(ws)
            # Bersihkan client yang disconnect
            for ws in to_remove:
                try:
                    clients.discard(ws)
                except:
                    pass
            print(f"[BCAST] sent '{payload.get('type', '<no-type>')}' to {sent_count} clients")

    # -------------------------------------------------------------
    # WEBSOCKET: Handler command START/STOP
    # -------------------------------------------------------------
    async def ws_receiver_loop(self, ws):
        """Terima command START/STOP/RESET dari Base Station."""
        try:
            async for raw in ws:
                msg = json.loads(raw)

                if msg.get("type") in ("command", "control"):
                    cmd = msg.get("command") or msg.get("cmd", "")
                    cmd = cmd.lower().strip()

                    if cmd == "start":
                        self.RUNNING = True
                        print("[CMD] START diterima")
                        self.send_control_command(0.0, 255.0)

                    elif cmd == "stop":
                        self.RUNNING = False
                        print("[CMD] STOP diterima")
                        self.send_control_command(0.0, 0.0)

                    elif cmd == "reset_distance":
                        self.REAL_TIME_DISTANCE = 0.0
                        print("[CMD] RESET distance")
        except websockets.exceptions.ConnectionClosed:
            pass
        except Exception as e:
            print(f"[WS ERROR] {e}")

    async def ws_handler(self, ws, path=None):
        """Menangani koneksi baru dari Base Station."""
        print(f"[WS] Client connected: {ws.remote_address}")
        self.CLIENTS.add(ws)

        recv_task = asyncio.create_task(self.ws_receiver_loop(ws))
        try:
            await recv_task
        finally:
            self.CLIENTS.discard(ws)
            recv_task.cancel()
            print(f"[WS] Client disconnected: {ws.remote_address}")

    async def start_websocket_server(self):
        """Menjalankan WebSocket server (untuk Base Station)."""
        try:
            server = await websockets.serve(self.ws_handler, "0.0.0.0", self.WS_PORT)
            print(f"[WS] WebSocket server running on ws://0.0.0.0:{self.WS_PORT}")
            return server
        except Exception as e:
            print(f"[FATAL] WebSocket server gagal jalan: {e}")

    async def run(self):
        """Main loop komunikasi (gabungan UDP & WS)."""
        await self.start_udp_listener()
        await self.start_websocket_server()

        print("[COMM] Semua sistem komunikasi aktif.")
        print("[COMM] Menunggu koneksi Base Station...")

        while True:
            await self.broadcast_telemetry()

            if self.RUNNING:
                print(
                    f"[RT] Speed={self.REAL_TIME_SPEED:.2f} | "
                    f"Obstacle={self.CURRENT_OBSTACLE_INFO}"
                )
            await asyncio.sleep(0.2)
