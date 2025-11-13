import cv2
import numpy as np
import math
from typing import Tuple, Optional, Any, Dict

# --- KONFIGURASI VISI ---
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
MAX_STEERING_ANGLE = 45.0
HOUGH_THRESHOLD = 20
HOUGH_MIN_LINE_LENGTH = 20
HOUGH_MAX_LINE_GAP = 10


# =================================================================
# CLASS: VISION PROCESSOR
# =================================================================
class VisionProcessor:
    """
    Mengelola semua fungsi visi komputer (Segmentasi HLS, BEV, Hough Transform)
    untuk menghitung sudut kemudi dan status jalur.
    """
    def __init__(self, width: int = FRAME_WIDTH, height: int = FRAME_HEIGHT):
        self.w = width
        self.h = height
        
        self.M, self.M_inv = self._get_perspective_matrices()
        
        # Constants for HLS thresholding
        self.LOWER_WHITE = np.array([0, 150, 0]) 
        self.UPPER_WHITE = np.array([255, 255, 255])


    def _get_perspective_matrices(self) -> Tuple[np.ndarray, np.ndarray]:
        """Menentukan matriks BEV dan Inverse BEV."""
        # Nilai ASUMSI, harus dikalibrasi.
        src = np.float32([
            [self.w * 0.40, self.h * 0.60], [self.w * 0.60, self.h * 0.60],
            [self.w * 0.10, self.h * 0.95], [self.w * 0.90, self.h * 0.95]
        ])
        offset = self.w * 0.2
        dst = np.float32([
            [offset, 0], [self.w - offset, 0],
            [offset, self.h], [self.w - offset, self.h]
        ])
        return cv2.getPerspectiveTransform(src, dst), cv2.getPerspectiveTransform(dst, src)


    def _calculate_angle_from_line(self, x1, y1, x2, y2) -> float:
        """Menghitung sudut garis berdasarkan gradien."""
        if abs(x2 - x1) < 1e-3: return 90.0
        grad = (y2 - y1) / (x2 - x1)
        angle = math.degrees(math.atan(grad))
        return max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, angle))


    def estimate_robot_lane_position(self, mask_frame: np.ndarray) -> str:
        """
        Menentukan posisi robot (left/right/center/unknown) berdasarkan 
        pusat marka yang terdeteksi di bagian bawah frame.
        """
        h, w = mask_frame.shape[:2]
        # Cari semua piksel putih (marka) di bagian bawah frame
        bottom_area = mask_frame[int(h*0.7):, :]
        
        # Cari koordinat X dari semua piksel marka
        coords = cv2.findNonZero(bottom_area)
        
        if coords is None:
            return "unknown" # Jika tidak ada marka terdeteksi di area dekat
            
        # Ambil min dan max X dari semua marka yang terdeteksi
        min_x = np.min(coords[:, :, 0])
        max_x = np.max(coords[:, :, 0])
        
        # Titik tengah marka yang terdeteksi
        center_of_mark = (min_x + max_x) / 2.0
        
        frame_center = w / 2.0
        tolerance = w * 0.10 # Toleransi 10% dari lebar frame
        
        if center_of_mark < frame_center - tolerance:
            return "left"
        elif center_of_mark > frame_center + tolerance:
            return "right"
        else:
            return "center"


    def process_frame_for_lane_data(self, frame_bgr: np.ndarray) -> Tuple[np.ndarray, float, str, str, np.ndarray]:
        """
        Melakukan seluruh pipeline visi komputer.
        
        Returns:
            - overlay_frame (BGR, frame dengan garis deteksi)
            - avg_angle (float)
            - lane_status (Detected/Lost)
            - robot_position (left/rightunknown)
            - bev_mask_result (grayscale)
        """
        h, w = frame_bgr.shape[:2]
        avg_angle = 0.0
        lane_status = "Lost"
        robot_position = "unknown"
        overlay = frame_bgr.copy()
        
        # 1. Segmentasi Warna dan Morfologi
        hls = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HLS)
        mask = cv2.inRange(hls, self.LOWER_WHITE, self.UPPER_WHITE)
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        # 2. Bird's Eye View (BEV)
        bev_mask = cv2.warpPerspective(mask, self.M, (w, h), flags=cv2.INTER_LINEAR)
        
        # 3. Deteksi Garis (Hough Transform pada BEV)
        edges = cv2.Canny(bev_mask, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=HOUGH_THRESHOLD, 
                                minLineLength=HOUGH_MIN_LINE_LENGTH, maxLineGap=HOUGH_MAX_LINE_GAP)

        if lines is not None and len(lines) > 0:
            lane_status = "Detected"
            angles = []
            
            # Perhitungan Sudut
            M_inv = self._get_perspective_matrices()[1] # Dapatkan M_inv
            bev_line_image = np.zeros_like(frame_bgr) 
            for ln in lines:
                x1, y1, x2, y2 = ln.reshape(4)
                angles.append(self._calculate_angle_from_line(x1, y1, x2, y2)) 
                cv2.line(bev_line_image, (x1, y1), (x2, y2), (0,255,0), 3)
                
            avg_angle = float(np.mean(angles))
            
            # Visualisasi Garis (Inverse Transform)
            warped_line = cv2.warpPerspective(bev_line_image, M_inv, (w, h))
            overlay = cv2.addWeighted(overlay, 1.0, warped_line, 1.0, 0)
        
        # 4. Estimasi Posisi Robot (Menggunakan mask BEV)
        # Posisi dihitung berdasarkan BEV mask, bukan status Hough lines
        robot_position = self.estimate_robot_lane_position(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)) 
        
        # Note: Pengembalian ini sedikit dimodifikasi dari pola lama untuk integrasi.
        return overlay, avg_angle, lane_status, robot_position, bev_mask