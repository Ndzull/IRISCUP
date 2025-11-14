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
# control.py
from typing import Tuple

class RobotController:
    def __init__(self):
        pass

    def decide_command(self, angle_deg: float, lane_status: str, obstacle_info: dict) -> Tuple[float, float]:
        """
        Return (steer_angle_deg, motor_value)
        motor_value is a simple speed constant (0..255 example)
        """
        if obstacle_info.get("detected", False):
            dist = obstacle_info.get("distance_cm")
            if dist is not None and dist < 20:
                return 0.0, 0.0  # stop
            # if obstacle at one side, small steer away
            if obstacle_info.get("position") == "left":
                return 30.0, 50.0
            if obstacle_info.get("position") == "right":
                return -30.0, 50.0

        # basic mapping: steer = -angle (if angle positive means lane tilting right)
        steer = max(-45.0, min(45.0, -angle_deg))
        # speed reduces when high steer
        abs_s = abs(steer)
        if abs_s > 30:
            speed = 150.0
        elif abs_s > 15:
            speed = 250.0
        else:
            speed = 255.0
        # clip to firmware expected range if needed
        return steer, speed
