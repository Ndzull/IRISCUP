from typing import Tuple, Optional, Dict, Any
import math
import time

MAX_STEERING_ANGLE = 45.0  #sudut maks
MAX_SPEED_CMD = 1500.0     #kecepatan maks motor dc
MIN_OBSTACLE_DIST = 20.0   

class PID:
    '''PID buat smoothing angle dan jalan robot'''
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()

    def compute(self, error):
        now = time.time()
        dt = now - self.prev_time
        self.prev_time = now

        if dt <= 0:
            dt = 1e-6

        # PID components
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        # Output
        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        return output


class RobotController:
    '''atur PWM motor dc berdasarkan sudut dan atur sudut saat ada obstacle'''
    def __init__(self, pid:PID):
        self.state = "normal"         # normal/avoid/return_left
        self.last_safe_time = time.time()
        self.pid = pid                # PID
        self.alpha = 0.25             # smoothing ygy
        self.prev_angle = 0.0

    def pwm_to_speed(self,pwm):
        a = 0.000141
        b = -0.233
        c = 101.1
        speed= a*pwm*pwm + b*pwm + c
        return speed
    
    def smooth_angle(self, angle):
        smoothed = (self.alpha * angle) + ((1 - self.alpha) * self.prev_angle)
        self.prev_angle = smoothed
        return smoothed

    def speed_curve(self, steer):
        s = abs(steer)
        if s > 40:
            return 700
        elif s > 25:
            return 900
        else:
            return 1200  
        

    def decide_command(self, angle_deg: float, lane_status: str, obstacle_info: dict):
        
        dist = obstacle_info.get("distance_cm")
        detected = obstacle_info.get("detected", False)

        #detect obs di lajur kiri
        if detected and dist is not None and dist < 40:
            self.state = "avoid"
            self.last_safe_time = time.time()
            return 45.0, 700.0        # belok kanan

        if self.state == "avoid":
            if (not detected) or (dist is not None and dist > 60):
                self.state = "return_left"
                return -45.0, 700.0   # balik kiri
            else:
                return 40.0, 800.0    # tetep belok kanan

        if self.state == "return_left":
            if abs(angle_deg) < 15:
                self.state = "normal"
            return -30.0, 800.0        # balik ke kiri pelan

        #jalan normal
        #smoothing
        angle = self.smooth_angle(angle_deg)

        #PID steer
        steer_pid = self.pid.compute(angle)

        #Limit steering
        steer = max(-45, min(45, steer_pid))

        #Speed (sesuai besar sudut)
        speed = self.speed_curve(steer)

        return steer, speed
