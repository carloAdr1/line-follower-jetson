"""
controller_node.py — prueba de evasión de obstáculos
- Sigue el carril con servo proporcional (5 niveles, correcciones suaves)
- Cuando detecta obstáculo (car_distance < umbral) manda /rebase al ESP32
- El ESP32 ejecuta la maniobra completa de forma autónoma
- Sin stop ni cruce peatonal (no aplica en esta prueba)
"""
import urllib.request, urllib.error, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32

ESP32_IP = "http://192.168.10.1"

# ── Distancias (en metros) ─────────────────────────────────────────────
OBSTACLE_TRIGGER  = 0.40   # m — inicia rebase (40 cm)
REBASE_COOLDOWN   = 6.0    # s — no vuelve a intentar rebase por este tiempo

# ── Servo proporcional ─────────────────────────────────────────────────
OFFSET_DEAD  = 25    # px — zona muerta, no toca el servo
OFFSET_SOFT  = 55    # px — corrección suave (al/dl)
OFFSET_HARD  = 90    # px — corrección fuerte (a/d)
HEADING_K    = 1.5   # factor para combinar heading con offset


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.last_lane_time = self.get_clock().now()

        self.create_subscription(Vector3, '/lane/info',     self.lane_cb, 10)
        self.create_subscription(Float32, '/car_distance',  self.car_cb,  10)

        self.offset_px   = 0.0
        self.heading_deg = 0.0
        self.car_dist    = 999.0

        self.rebase_sent_at = 0.0   # timestamp del último /rebase enviado

        # Estado: LANE_FOLLOW o REBASE_WAIT
        # Durante REBASE_WAIT el controller no manda comandos de movimiento
        # porque el ESP32 tiene el control.
        self.state = "LANE_FOLLOW"

        self.last_drive = self.last_steer = None
        self.last_brake = self.last_blink = None

        self.create_timer(0.1, self.loop)
        self.get_logger().info("controller_node (evasión) listo")

    # ── HTTP ───────────────────────────────────────────────────────────
    def http(self, cmd):
        try:
            with urllib.request.urlopen(f"{ESP32_IP}/{cmd}", timeout=0.3) as r:
                r.read()
        except Exception as e:
            self.get_logger().warn(f"[{cmd}] {e}")

    def _send(self, attr, cmd):
        if getattr(self, attr) != cmd:
            self.http(cmd); setattr(self, attr, cmd)
            self.get_logger().info(f"  {attr.split('_')[1].upper()} → {cmd}")

    def sd(self, c): self._send('last_drive', c)
    def ss(self, c): self._send('last_steer', c)
    def sb(self, c): self._send('last_brake', c)
    def sl(self, c): self._send('last_blink', c)

    # ── Callbacks ──────────────────────────────────────────────────────
    def lane_cb(self, msg):
        self.offset_px = msg.x; self.heading_deg = msg.y
        self.last_lane_time = self.get_clock().now()

    def car_cb(self, msg):
        self.car_dist = msg.data if msg.data > 0 else 999.0

    # ── Loop principal ─────────────────────────────────────────────────
    def loop(self):
        now = time.time()

        if self.state == "REBASE_WAIT":
            # Esperamos que el ESP32 termine la maniobra.
            # Tiempo estimado: T_GIRO_IZQ + T_END1 + T_AVANZA + T_GIRO_DER + T_END2
            # = 0.9 + 0.3 + 5.0 + 0.9 + 0.4 = 7.5s + margen
            if now - self.rebase_sent_at > 8.5:
                self.get_logger().info("Rebase completado → LANE_FOLLOW")
                self.state = "LANE_FOLLOW"
            return   # no mandamos nada mientras el ESP32 maneja el carro

        # ── LANE_FOLLOW ──
        # Disparar rebase si hay obstáculo y no estamos en cooldown
        if (self.car_dist < OBSTACLE_TRIGGER and
                now - self.rebase_sent_at > REBASE_COOLDOWN):
            self.get_logger().info(
                f"Obstáculo a {self.car_dist:.2f}m → enviando /rebase")
            self.http("rebase")
            self.rebase_sent_at = now
            self.state = "REBASE_WAIT"
            return

        self._follow_lane()

    # ── Seguimiento de carril ──────────────────────────────────────────
    def _follow_lane(self):
        elapsed = (self.get_clock().now()-self.last_lane_time).nanoseconds/1e9
        if elapsed > 1.0:
            self.sd('x'); self.ss('c'); self.sb('brake_on'); self.sl('blink_off')
            return

        self.sb('brake_off')
        steer = self._compute_steer(self.offset_px, self.heading_deg)
        self.ss(steer); self.sd('w')

        if   steer in ('a','al'): self.sl('blink_izq')
        elif steer in ('d','dl'): self.sl('blink_der')
        else:                     self.sl('blink_off')

    def _compute_steer(self, offset, heading):
        """
        offset > 0 → carro desviado a la derecha → girar izquierda (a)
        offset < 0 → carro desviado a la izquierda → girar derecha (d)
        """
        v = offset + heading * HEADING_K
        if   v >  OFFSET_HARD: return 'a'
        elif v >  OFFSET_SOFT: return 'al'
        elif v < -OFFSET_HARD: return 'd'
        elif v < -OFFSET_SOFT: return 'dl'
        else:                  return 'c'


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for c in ['x','brake_on','blink_off','c']: node.http(c)
        node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
