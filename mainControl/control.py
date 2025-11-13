from typing import Tuple, Optional, Dict, Any
import math

MAX_STEERING_ANGLE = 45.0  #sudut maks
MAX_SPEED_CMD = 255.0       # Kecepatan maks motor dc
MIN_OBSTACLE_DIST = 20.0   

# =================================================================
# CLASS: PID CONTROLLER (STANDAR ROBOTIKA)
# =================================================================
class PIDController:
    """
    Simulasi PID Controller untuk menghaluskan perintah kemudi.
    Saat ini, hanya menggunakan komponen Proporsional (P).
    """
    def __init__(self, Kp: float = 0.8, Ki: float = 0.0, Kd: float = 0.0):
        self.Kp = Kp  # Gain Proporsional (Paling penting)
        self.Ki = Ki
        self.Kd = Kd
        # Variabel error untuk Integral dan Derivative tidak digunakan dalam versi sederhana ini.

    def compute(self, error: float) -> float:
        """Menghitung output kontrol (PWM/sudut) berdasarkan error."""
        # Output = Kp * Error
        output = self.Kp * error
        return output


# =================================================================
# CLASS: ROBOT CONTROLLER (LOGIC KEPUTUSAN UTAMA)
# =================================================================
class RobotController:
    """
    Mengambil data visi dan obstacle, dan mengeluarkan perintah steer & speed.
    """
    def __init__(self):
        self.pid_steer = PIDController(Kp=1.2) # Kp sederhana untuk steering

    def decide_command(self, steer_angle_vision: float, lane_status: str, obstacle: Dict[str, Any]) -> Tuple[float, float]:
        """
        Menentukan sudut kemudi (steer) dan kecepatan (speed) akhir 
        berdasarkan prioritas: SAFETY > OBSTACLE AVOIDANCE > LINE FOLLOWING.
        """
        
        steer_command = 0.0
        speed_command = 0.0
        
        # 1. Safety & Loss of Lane
        if lane_status == "Lost":
            print("[CONTROL] Lane LOST -> Emergency Stop.")
            return 0.0, 0.0
            
        # 2. Safety & Stop Flag (Dari BS)
        # Note: Logic ini harus dicek di main.py sebelum dipanggil
        
        # 3. Obstacle Avoidance (Prioritas Tinggi)
        if obstacle.get("detected"):
            dist = obstacle.get("distance")
            pos = obstacle.get("position")

            # A. Safety Stop: Jika terlalu dekat, STOP total
            if dist is not None and dist < MIN_OBSTACLE_DIST: 
                print(f"[CONTROL] OBSTACLE {dist}cm -> HALT.")
                return 0.0, 0.0 
            
            # B. Logika Penghindaran (Stay-Left Policy)
            if pos == "left":
                # Obstacle di jalur kiri, manuver ke kanan (sudut positif)
                print("[CONTROL] Avoid LEFT: Steer HARD RIGHT.")
                steer_command = MAX_STEERING_ANGLE
                speed_command = MAX_SPEED_CMD * 0.4 # Kurangi speed saat manuver
            elif pos == "right":
                # Obstacle di jalur kanan (tidak seharusnya ada di sana), kembali ke kiri
                print("[CONTROL] Obstacle RIGHT: Steer Left.")
                steer_command = -MAX_STEERING_ANGLE * 0.5 # Steer Kiri (kembali)
                speed_command = MAX_SPEED_CMD * 0.7
            
            return steer_command, speed_command

        # 4. Normal Line Following (Jika aman)
        
        # Hitung error steering (Target 0 - Sudut Visi)
        steer_error = 0.0 - steer_angle_vision
        
        # Gunakan PID untuk menghaluskan perintah steering
        raw_steer_output = self.pid_steer.compute(steer_error)
        
        # Clamp Steering Output (Batasi sudut akhir)
        steer_command = max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, raw_steer_output))
        
        # Logika Speed: Kurangi kecepatan di tikungan tajam
        if abs(steer_command) > 25.0:
            speed_command = MAX_SPEED_CMD * 0.6 # Tikungan tajam
        elif abs(steer_command) > 10.0:
            speed_command = MAX_SPEED_CMD * 0.8 # Tikungan sedang
        else:
            speed_command = MAX_SPEED_CMD # Lurus

        return steer_command, speed_command