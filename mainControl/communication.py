import asyncio
import websockets
import socket
import json
from typing import Tuple, Any, Dict, Set, Optional

# =================================================================
# CLASS: UDP SENSOR PROTOCOL (Receiver)
# =================================================================

class SensorUDPProtocol(asyncio.DatagramProtocol):
    """Menerima paket data sensor (Speed, Obstacle) dari ESP32."""
    def __init__(self, manager):
        self.manager = manager
        
    def datagram_received(self, data, addr):
        txt = None
        try:
            txt = data.decode("utf-8").strip()
            
            # --- Logic Parsing Data Masuk (Speed dan Obstacle) ---
            
            # 1. Parsing Speed: Format "S:25.5"
            if txt.startswith("S:"):
                speed = float(txt.split(":")[1].strip())
                self.manager.REAL_TIME_SPEED = speed
                
            # 2. Parsing Obstacle: Format "OBS:L:45"
            elif txt.startswith("OBS:"):
                parts = txt.split(":")
                if len(parts) >= 3:
                    pos_code = parts[1].strip().upper() # L atau R
                    dist = float(parts[2].strip()) if parts[2].strip().replace('.', '', 1).isdigit() else None
                    
                    # Hanya terima L atau R untuk posisi
                    if pos_code in ("L", "R"):
                        self.manager.CURRENT_OBSTACLE_INFO["detected"] = True
                        self.manager.CURRENT_OBSTACLE_INFO["distance"] = dist
                        self.manager.CURRENT_OBSTACLE_INFO["position"] = "left" if pos_code == "L" else "right"
                    
                    # Jika sensor mengirim 0 atau None (Clear)
                    elif dist is not None and dist == 0.0:
                         self.manager.CURRENT_OBSTACLE_INFO = {"detected": False, "position": None, "distance_cm": None}
            
        except Exception:
            # Mengabaikan paket data yang rusak
            pass 


# =================================================================
# CLASS: COMMUNICATION MANAGER (INTI JARINGAN & STATE)
# =================================================================

class CommunicationManager:
    """
    Mengelola semua koneksi jaringan (UDP Sensor, UDP Control, WebSocket Server) 
    dan menyimpan state telemetry global.
    """
    def __init__(self, ws_port: int, udp_sensor_port: int, udp_control_ip: str, udp_control_port: int):
        # Konfigurasi Jaringan
        self.WS_PORT = ws_port
        self.UDP_SENSOR_PORT = udp_sensor_port
        self.UDP_CONTROL_IP = udp_control_ip
        self.UDP_CONTROL_PORT = udp_control_port
        
        # State Global Telemetry (Dibagikan dengan Vision dan Control)
        self.REAL_TIME_SPEED = 0.0
        self.REAL_TIME_DISTANCE = 0.0
        self.RUNNING = False  # Flag START/STOP dari BS
        self.CURRENT_OBSTACLE_INFO: Dict[str, Any] = {"detected": False, "position": None, "distance": None}
        
        self.CLIENTS: Set[Any] = set() # Set klien WebSocket yang terhubung
        
        # Socket UDP untuk Mengirim Perintah Kontrol (Harus dibuat di awal)
        self.udp_cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    
    # -------------------------------------------------------------
    # FUNGSI KONTROL & BROADCAST
    # -------------------------------------------------------------

    def send_control_command(self, steer_angle: float, speed_cmd: float):
        """Mengirim perintah kontrol motor ke ESP32 via UDP."""
        # Format pengiriman: Misalnya "A:15.5,S:20.0"
        cmd_str = f"A:{round(steer_angle, 2)},S:{round(speed_cmd, 2)}\n"
        try:
            self.udp_cmd_sock.sendto(cmd_str.encode("utf-8"), (self.UDP_CONTROL_IP, self.UDP_CONTROL_PORT))
            print(f"[CMD OUT] Sent: {cmd_str.strip()}")
        except Exception as e:
            print(f"[WARN] Failed to send control UDP: {e}")
            
    async def broadcast_telemetry(self, data: dict):
        """Mengirim Telemetri JSON ke semua klien WebSocket yang terhubung."""
        if not self.CLIENTS:
            return
        
        msg = json.dumps(data)
        to_remove = []
        for ws in list(self.CLIENTS):
            try:
                await ws.send(msg)
            except websockets.exceptions.ConnectionClosed:
                to_remove.append(ws)
            except Exception:
                to_remove.append(ws)
        for ws in to_remove:
            self.CLIENTS.discard(ws)


    # -------------------------------------------------------------
    # FUNGSI SERVER & RECEIVER
    # -------------------------------------------------------------
    
    async def start_udp_listener(self):
        """Memulai UDP Receiver untuk data sensor."""
        loop = asyncio.get_event_loop()
        try:
            await loop.create_datagram_endpoint(
                lambda: SensorUDPProtocol(self), # Memberikan instance manager ke protocol
                local_addr=('0.0.0.0', self.UDP_SENSOR_PORT)
            )
            print(f"[COMM] UDP Sensor Listener active on port {self.UDP_SENSOR_PORT}")
        except Exception as e:
            print(f"[FATAL] Failed to start UDP Listener: {e}")
            raise # Melemparkan error agar main loop tahu

    async def ws_receiver_loop(self, ws):
        """Menerima perintah START/STOP/RESET dari Base Station."""
        try:
            async for raw in ws:
                msg = json.loads(raw)
                if msg.get("type") == "control":
                    cmd = msg.get("command", "").lower()
                    if cmd == "start":
                        self.RUNNING = True
                        print("[CMD] Received START signal.")
                        # Kirim kecepatan awal/default saat START
                        # Asumsi default speed 45.0
                        self.send_control_command(0.0, 45.0) 
                    elif cmd == "stop":
                        self.RUNNING = False
                        print("[CMD] Received STOP signal.")
                        self.send_control_command(0.0, 0.0) # Perintah Stop Darurat
                    elif cmd == "reset_distance":
                        self.REAL_TIME_DISTANCE = 0.0
                        print("[CMD] Distance reset.")
        except websockets.exceptions.ConnectionClosed:
            pass
        except Exception as e:
            print(f"[WS Receiver Error] {e}")

    async def ws_handler(self, ws, path):
        """Handle incoming client connection (Base Station)."""
        print(f"[WS] Client connected: {ws.remote_address}")
        self.CLIENTS.add(ws)
        
        # Jalankan task receiver untuk klien ini
        receiver = asyncio.create_task(self.ws_receiver_loop(ws)) 
        
        try:
            # Tunggu receiver berjalan (atau dibatalkan saat disconnect)
            await receiver 
        finally:
            self.CLIENTS.discard(ws)
            receiver.cancel()
            print(f"[WS] Client disconnected: {ws.remote_address}")