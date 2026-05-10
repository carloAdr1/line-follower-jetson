import urllib.request
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

ESP32_IP = "http://192.168.10.1"

OFFSET_SOFT = 55
OFFSET_HARD = 90
HEADING_K = 1.5

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.last_lane_time = self.get_clock().now()
        self.create_subscription(Vector3, '/lane/info', self.lane_cb, 10)

        self.offset_px = 0.0
        self.heading_deg = 0.0

        self.last_drive = None
        self.last_steer = None
        self.last_brake = None
        self.last_blink = None

        self.create_timer(0.1, self.loop)
        self.get_logger().info("controller_node listo: seguimiento de línea sin ultrasónico")

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

    def sd(self, c): self._send('last_drive', c)
    def ss(self, c): self._send('last_steer', c)
    def sb(self, c): self._send('last_brake', c)
    def sl(self, c): self._send('last_blink', c)

    def lane_cb(self, msg):
        self.offset_px = msg.x
        self.heading_deg = msg.y
        self.last_lane_time = self.get_clock().now()

    def loop(self):
        elapsed = (self.get_clock().now() - self.last_lane_time).nanoseconds / 1e9

        if elapsed > 1.0:
            self.sd('x')
            self.ss('c')
            self.sb('brake_on')
            self.sl('blink_off')
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

    def compute_steer(self, offset, heading):
        v = offset + heading * HEADING_K
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

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for c in ['x', 'brake_on', 'blink_off', 'c']:
            node.http(c)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
