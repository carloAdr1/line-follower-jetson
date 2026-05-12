"""
lane_stop_detector_node.py
Detección de carril + detección simple de señal STOP por máscara roja.
Publica:
- /lane/info Vector3: x=offset_px, y=heading_deg
- /stop/detected Bool: True si detecta rojo tipo señal
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
import cv2
import numpy as np

CAM_INDEX  = 0
FRAME_W    = 640
FRAME_H    = 480
TIMER_HZ   = 10.0

WHITE_V_MIN     = 155
LANE_ROI_START  = 0.50
STRIP_START     = 0.80
OFFSET_DEAD_PX  = 25
HEADING_DEAD    = 8


class LaneDetectorNode(Node):
    def __init__(self):
        super().__init__('lane_stop_detector_node')

        self.pub = self.create_publisher(Vector3, '/lane/info', 10)
        self.stop_pub = self.create_publisher(Bool, '/stop/detected', 10)

        self.cap = cv2.VideoCapture(CAM_INDEX)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
        self.cap.set(cv2.CAP_PROP_FPS, TIMER_HZ)

        if not self.cap.isOpened():
            raise RuntimeError(f"No se pudo abrir cámara {CAM_INDEX}")

        self.create_timer(1.0 / TIMER_HZ, self.loop)
        self.get_logger().info(f"LaneDetector listo — cam {CAM_INDEX} {FRAME_W}x{FRAME_H}")

    def loop(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        h, w = frame.shape[:2]

        # -------- DETECCIÓN STOP / ROJO --------
        stop_detected = self._detect_red_stop(frame, h, w)
        stop_msg = Bool()
        stop_msg.data = bool(stop_detected)
        self.stop_pub.publish(stop_msg)

        # -------- DETECCIÓN DE CARRIL ORIGINAL --------
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        offset, heading = self._detect_lane(gray, h, w)

        msg = Vector3()
        msg.x = float(offset)
        msg.y = float(heading)
        self.pub.publish(msg)

    def _detect_red_stop(self, frame, h, w):
        """
        Detecta regiones rojas compactas tipo señal STOP.
        No modifica la lógica de carril.
        """

        # Zona donde normalmente aparecería una señal:
        # evita piso/carril en la parte baja
        roi_y0 = int(h * 0.05)
        roi_y1 = int(h * 0.70)
        roi = frame[roi_y0:roi_y1, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Rojo en HSV se parte en dos rangos
        lower_red1 = np.array([0, 80, 70], dtype=np.uint8)
        upper_red1 = np.array([10, 255, 255], dtype=np.uint8)

        lower_red2 = np.array([170, 80, 70], dtype=np.uint8)
        upper_red2 = np.array([180, 255, 255], dtype=np.uint8)

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 600:
                continue

            x, y, cw, ch = cv2.boundingRect(cnt)

            if ch == 0:
                continue

            aspect = cw / float(ch)

            # Señal STOP normalmente es compacta, no una línea larga
            if not (0.55 <= aspect <= 1.8):
                continue

            # Evita objetos demasiado chicos o demasiado enormes
            area_box = cw * ch
            frame_area = w * h
            ratio = area_box / float(frame_area)

            if 0.002 <= ratio <= 0.20:
                return True

        return False

    def _detect_lane(self, gray, h, w):
        roi_y0 = int(h * LANE_ROI_START)
        roi    = gray[roi_y0:h, :]
        rh, rw = roi.shape
        mid    = rw // 2

        _, mask = cv2.threshold(roi, WHITE_V_MIN, 255, cv2.THRESH_BINARY)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

        # Offset en franja muy baja (más estable)
        strip_y = int(rh * ((STRIP_START - LANE_ROI_START) / (1.0 - LANE_ROI_START)))
        strip   = mask[strip_y:, :]
        col_sum = strip.sum(axis=0).astype(float)

        offset = 0.0
        white_cols = np.where(col_sum > col_sum.max() * 0.3)[0] if col_sum.max() > 0 else []

        if len(white_cols) > 5:
            lc = white_cols[white_cols < mid]
            rc = white_cols[white_cols >= mid]

            lx = int(lc.mean()) if len(lc) > 3 else None
            rx = int(rc.mean()) if len(rc) > 3 else None

            if lx is not None and rx is not None:
                offset = float((lx + rx) // 2 - mid)
            elif lx is not None:
                offset = float(lx - (mid - 100))
            elif rx is not None:
                offset = float(rx - (mid + 100))

        if abs(offset) < OFFSET_DEAD_PX:
            offset = 0.0

        # Heading con Hough
        heading = 0.0
        edges = cv2.Canny(mask, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 30, minLineLength=50, maxLineGap=20)

        if lines is not None:
            angles = [
                np.degrees(np.arctan2(y2 - y1, x2 - x1))
                for x1, y1, x2, y2 in lines[:, 0]
                if abs(x2 - x1) > 5 and abs(np.degrees(np.arctan2(y2 - y1, x2 - x1))) > 5
            ]

            if angles:
                heading = float(np.median(angles))

        if abs(heading) < HEADING_DEAD:
            heading = 0.0

        return offset, heading

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
