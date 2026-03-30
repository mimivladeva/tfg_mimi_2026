import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav2_msgs.msg import SpeedLimit
import time


class DebugMonitor(Node):

    def __init__(self):
        super().__init__("debug_monitor")

        self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 10)
        self.create_subscription(SpeedLimit, "/speed_limit", self.speed_callback, 10)

        self.last_cmd = Twist()
        self.last_speed_limit = None

        self.prev_state = "UNKNOWN"
        self.last_change_time = time.time()

        self.timer = self.create_timer(0.05, self.loop)

        self.get_logger().info("DEBUG MONITOR iniciado (preciso sin UNKNOWN)")

    # =========================
    # CALLBACKS
    # =========================

    def cmd_callback(self, msg):
        self.last_cmd = msg

    def speed_callback(self, msg):
        self.last_speed_limit = msg.speed_limit

    # =========================
    # CLASIFICACIÓN FINAL
    # =========================

    def classify_state(self):
        lin = self.last_cmd.linear.x
        ang = self.last_cmd.angular.z

        # -------------------------
        # STOP (muy claro)
        # -------------------------
        if abs(lin) < 0.01 and abs(ang) < 0.01:
            return "STOP"

        # -------------------------
        # TURN REAL (SPIN)
        # -------------------------
        if abs(ang) > 1.5 and abs(lin) < 0.05:
            return "TURN_LEFT" if ang > 0 else "TURN_RIGHT"

        # -------------------------
        # SLOW (prioridad sobre normal)
        # -------------------------
        if self.last_speed_limit is not None:
            if abs(self.last_speed_limit - 0.10) < 0.03:
                return "SLOW"

        # -------------------------
        # NORMAL (CUBRE TODO LO DEMÁS)
        # -------------------------
        if lin > 0.05:
            return "NORMAL"

        # -------------------------
        # CORRECCIÓN EN SITIO (Nav2)
        # -------------------------
        if abs(ang) > 0.2:
            return "NORMAL"

        # fallback (muy raro)
        return "STOP"

    # =========================
    # LOOP
    # =========================

    def loop(self):
        state = self.classify_state()

        if state != self.prev_state:
            now = time.time()
            duration = now - self.last_change_time
            self.last_change_time = now

            lin = self.last_cmd.linear.x
            ang = self.last_cmd.angular.z

            self.get_logger().info(
                f"[CAMBIO] {self.prev_state} → {state} | "
                f"duración: {duration:.2f}s | "
                f"lin={lin:.3f} ang={ang:.3f} speed_limit={self.last_speed_limit}"
            )

            self.prev_state = state


def main():
    rclpy.init()
    node = DebugMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()