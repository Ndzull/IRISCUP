import cv2
import numpy as np
import math
from typing import Tuple

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
MAX_STEERING_ANGLE = 45.0

# Hough defaults (bisa disesuaikan)
HOUGH_THRESHOLD = 25
HOUGH_MIN_LINE_LENGTH = 25
HOUGH_MAX_LINE_GAP = 20


class VisionProcessor:
    """
    Vision pipeline (HLS -> BEV -> Hough -> overlay)
    - process_frame_for_lane_data(frame) -> (overlay, avg_angle, lane_status, robot_position, bev_mask_bgr)
    - combine(overlay, bev_bgr) -> overlay with BEV inset
    """

    def __init__(self, width: int = FRAME_WIDTH, height: int = FRAME_HEIGHT):
        self.w = width
        self.h = height

        # Perspective matrices
        self.M, self.M_inv = self._get_perspective_matrices()

        # HLS white detection range (tweak jika perlu)
        self.LOWER_WHITE = np.array([0, 150, 0])
        self.UPPER_WHITE = np.array([255, 255, 255])

    def _get_perspective_matrices(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Buat matriks perspektif (BEV) dan inverse-nya.
        Koordinat src/dst ini asumsi — perlu tuning kalau POV berubah.
        """
        src = np.float32([
            [self.w * 0.40, self.h * 0.60],
            [self.w * 0.60, self.h * 0.60],
            [self.w * 0.10, self.h * 0.95],
            [self.w * 0.90, self.h * 0.95]
        ])
        offset = self.w * 0.2
        dst = np.float32([
            [offset, 0],
            [self.w - offset, 0],
            [offset, self.h],
            [self.w - offset, self.h]
        ])

        M = cv2.getPerspectiveTransform(src, dst)
        M_inv = cv2.getPerspectiveTransform(dst, src)
        return M, M_inv

    def _calculate_angle_from_line(self, x1, y1, x2, y2) -> float:
        """Hitung sudut (derajat) dari segmen garis, di-clamp ke MAX_STEERING_ANGLE."""
        dx = (x2 - x1)
        if abs(dx) < 1e-6:
            return 90.0
        slope = (y2 - y1) / dx
        angle = math.degrees(math.atan(slope))
        return max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, angle))

    def estimate_robot_lane_position(self, mask_frame: np.ndarray) -> str:
        """
        Tentukan posisi robot relatif terhadap marka di bottom area BEV mask.
        Mengembalikan "left", "right", "center", atau "unknown".
        """
        h, w = mask_frame.shape[:2]
        bottom = mask_frame[int(h * 0.7):, :]

        coords = cv2.findNonZero(bottom)
        if coords is None:
            return "unknown"

        min_x = int(np.min(coords[:, :, 0]))
        max_x = int(np.max(coords[:, :, 0]))
        lane_center = (min_x + max_x) / 2.0
        frame_center = w / 2.0
        tol = w * 0.20

        if lane_center < frame_center - tol:
            # marka ke kiri frame -> robot relatif lebih ke kanan (pov kiri => interpretasi terserah aplikasi)
            return "right"
        elif lane_center > frame_center + tol:
            return "left"
        else:
            return "center"

    def combine(self, overlay: np.ndarray, bev_bgr: np.ndarray, inset_scale: float = 0.30, padding: int = 8) -> np.ndarray:
        """
        Gabungkan overlay (full frame) dengan BEV sebagai inset (pojak kiri bawah secara default).
        - inset_scale: ukuran BEV relatif terhadap lebar overlay (0..1)
        - padding: jarak dari tepi
        """
        out = overlay.copy()
        h, w = out.shape[:2]
        if bev_bgr is None or bev_bgr.size == 0:
            return out

        # resize BEV
        inset_w = int(w * inset_scale)
        inset_h = int(h * inset_scale * (bev_bgr.shape[0] / max(1, bev_bgr.shape[1])))
        bev_small = cv2.resize(bev_bgr, (inset_w, inset_h))

        x0 = padding
        y0 = h - inset_h - padding

        # If alpha blending desired, create border + semitransparent background
        # background rectangle
        rect_color = (10, 10, 10)
        alpha = 0.85
        sub = out[y0:y0 + inset_h, x0:x0 + inset_w].astype(float)
        src = bev_small.astype(float)
        blended = cv2.addWeighted(src, alpha, sub, 1 - alpha, 0).astype(np.uint8)
        out[y0:y0 + inset_h, x0:x0 + inset_w] = blended

        # optional border
        cv2.rectangle(out, (x0 - 1, y0 - 1), (x0 + inset_w, y0 + inset_h), (200, 200, 200), 1)

        return out

    def process_frame_for_lane_data(self, frame_bgr):
        """
        Pipeline utama:
         - threshold HLS untuk marka putih
         - morphological cleanup
         - warpPerspective -> BEV mask
         - Canny + HoughLinesP di BEV
         - klasifikasi garis (left/right) & rata-rata angle
         - inverse-warp garis ke overlay
         - kembalikan (overlay_with_lines, avg_angle, lane_status, robot_position, bev_mask_bgr)
        """
        h, w = frame_bgr.shape[:2]
        overlay = frame_bgr.copy()

        # 1) HLS threshold
        hls = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HLS)
        mask = cv2.inRange(hls, self.LOWER_WHITE, self.UPPER_WHITE)

        # morphological to reduce noise
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        # 2) BEV
        bev_mask = cv2.warpPerspective(mask, self.M, (w, h), flags=cv2.INTER_LINEAR)

        # 3) edges + Hough on BEV
        edges = cv2.Canny(bev_mask, 40, 120)
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180,
            threshold=HOUGH_THRESHOLD,
            minLineLength=HOUGH_MIN_LINE_LENGTH,
            maxLineGap=HOUGH_MAX_LINE_GAP
        )

        lane_status = "Lost"
        avg_angle = 0.0

        bev_lines = np.zeros((h, w, 3), dtype=np.uint8)
        left_angles = []
        right_angles = []

        if lines is not None and len(lines) > 0:
            lane_status = "Detected"
            for ln in lines:
                x1, y1, x2, y2 = ln.reshape(4)
                dx = (x2 - x1)
                # avoid divide-by-zero, compute slope safely
                slope = (y2 - y1) / (dx + 1e-6)
                angle = self._calculate_angle_from_line(x1, y1, x2, y2)

                # heuristik: slope sign determines left/right in BEV coords (tweak thresholds if needed)
                if slope > 0.3:
                    left_angles.append(angle)
                    cv2.line(bev_lines, (x1, y1), (x2, y2), (0, 255, 0), 3)
                elif slope < -0.3:
                    right_angles.append(angle)
                    cv2.line(bev_lines, (x1, y1), (x2, y2), (0, 255, 0), 3)
                else:
                    # nearly vertical / small slope -> still draw
                    cv2.line(bev_lines, (x1, y1), (x2, y2), (0, 200, 0), 2)

            all_angles = left_angles + right_angles
            if len(all_angles) > 0:
                avg_angle = float(np.mean(all_angles))

        # 4) unwarp bev_lines onto original overlay
        unwarped = cv2.warpPerspective(bev_lines, self.M_inv, (w, h), flags=cv2.INTER_LINEAR)
        overlay = cv2.addWeighted(overlay, 1.0, unwarped, 1.0, 0)

        # 5) position estimate (gunakan BEV mask)
        robot_position = self.estimate_robot_lane_position(bev_mask)

        # 6) convert bev_mask to BGR for UI and return
        bev_mask_bgr = cv2.cvtColor(bev_mask, cv2.COLOR_GRAY2BGR)

        # 7) Optional: produce combined visualization (overlay + inset BEV)
        combined_view = self.combine(overlay, bev_mask_bgr, inset_scale=0.28)

        return combined_view, avg_angle, lane_status, robot_position, bev_mask_bgr
