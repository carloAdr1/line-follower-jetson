import urllib.request
import csv
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool

ESP32_IP = "http://192.168.10.1"

OFFSET_SOFT = 55
OFFSET_HARD = 90
HEADING_K = 1.5

STOP_SECONDS = 5.0


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.last_lane_time = self.get_clock().now()

        self.create_subscription(Vector3, '/lane/info', self.lane_cb, 10)
        self.create_subscription(Bool, '/stop/detected', self.stop_cb, 10)

        self.offset_px = 0.0
        self.heading_deg = 0.0

        self.last_drive = None
        self.last_steer = None
        self.last_brake = None
        self.last_blink = None

        self.stop_active = False
        self.stop_until = None
        self.stop_latched = False
        self.stop_detected = False

        self.control_v = 0.0
        self.log_data = []
        self.start_time_s = self.get_clock().now().nanoseconds / 1e9

        self.create_timer(0.1, self.loop)
        self.get_logger().info("controller_node listo: línea + STOP 5s + logger CSV")

    def http(self, cmd):
        try:
            with urllib.request.urlopen(f"{ESP32_IP}/{cmd}", timeout=0.3) as r:
                r.read()
        except Exception as e:
            self.get_logger().warn(f"[{cmd}] {e}")

    def _send(self, attr, cmd):
        if getattr(self, attr) != cmd:
            self.http(cmd)
            setattr(self, attr, cmd)

    def sd(self, c):
        self._send('last_drive', c)

    def ss(self, c):
        self._send('last_steer', c)

    def sb(self, c):
        self._send('last_brake', c)

    def sl(self, c):
        self._send('last_blink', c)

    def lane_cb(self, msg):
        self.offset_px = msg.x
        self.heading_deg = msg.y
        self.last_lane_time = self.get_clock().now()

    def stop_cb(self, msg):
        self.stop_detected = bool(msg.data)
        now_s = self.get_clock().now().nanoseconds / 1e9

        if msg.data and not self.stop_active and not self.stop_latched:
            self.stop_active = True
            self.stop_latched = True
            self.stop_until = now_s + STOP_SECONDS
            self.get_logger().info("STOP detectado: deteniendo 5 segundos")

        if not msg.data and not self.stop_active:
            self.stop_latched = False

    def loop(self):
        now_s = self.get_clock().now().nanoseconds / 1e9

        if self.stop_active:
            self.sd('x')
            self.ss('c')
            self.sb('brake_on')
            self.sl('blink_off')

            if self.stop_until is not None and now_s >= self.stop_until:
                self.stop_active = False
                self.stop_until = None
                self.sb('brake_off')
                self.get_logger().info("STOP terminado: regresando a seguimiento normal")

            self.save_sample(now_s)
            return

        elapsed = (self.get_clock().now() - self.last_lane_time).nanoseconds / 1e9

        if elapsed > 1.0:
            self.sd('x')
            self.ss('c')
            self.sb('brake_on')
            self.sl('blink_off')
            self.save_sample(now_s)
            return

        self.sb('brake_off')
        steer = self.compute_steer(self.offset_px, self.heading_deg)

        self.ss(steer)
        self.sd('w')

        if steer in ('a', 'al'):
            self.sl('blink_izq')
        elif steer in ('d', 'dl'):
            self.sl('blink_der')
        else:
            self.sl('blink_off')

        self.save_sample(now_s)

    def compute_steer(self, offset, heading):
        v = offset + heading * HEADING_K
        self.control_v = v

        if v > OFFSET_HARD:
            return 'a'
        elif v > OFFSET_SOFT:
            return 'al'
        elif v < -OFFSET_HARD:
            return 'd'
        elif v < -OFFSET_SOFT:
            return 'dl'
        else:
            return 'c'

    def save_sample(self, now_s):
        t = now_s - self.start_time_s

        self.log_data.append({
            'time_s': t,
            'offset_px': self.offset_px,
            'heading_deg': self.heading_deg,
            'control_estimated_v': self.control_v,
            'steer_cmd': self.last_steer,
            'drive_cmd': self.last_drive,
            'brake_cmd': self.last_brake,
            'blink_cmd': self.last_blink,
            'stop_detected': int(self.stop_detected),
            'stop_active': int(self.stop_active),
            'lane_error_abs_px': abs(self.offset_px),
            'heading_error_abs_deg': abs(self.heading_deg)
        })

    def save_csv(self):
        if not self.log_data:
            self.get_logger().warn("No hay datos para guardar.")
            return

        folder = "logs"
        os.makedirs(folder, exist_ok=True)

        filename = datetime.now().strftime("controller_log_%Y%m%d_%H%M%S.csv")
        path = os.path.join(folder, filename)

        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=self.log_data[0].keys())
            writer.writeheader()
            writer.writerows(self.log_data)

        self.get_logger().info(f"Log guardado en: {path}")


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_csv()

        for c in ['x', 'brake_on', 'blink_off', 'c']:
            node.http(c)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
