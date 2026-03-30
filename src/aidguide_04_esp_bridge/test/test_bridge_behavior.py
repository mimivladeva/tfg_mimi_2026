import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav2_msgs.msg import SpeedLimit
import time


class BridgeMonitor(Node):

    def __init__(self):
        super().__init__("bridge_monitor")

        self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 10)
        self.create_subscription(SpeedLimit, "/speed_limit", self.speed_cb, 10)

        self.last_cmd = None
        self.last_speed_limit = None

        self.current_event = "UNKNOWN"
        self.last_event_time = time.time()

        self.get_logger().info("Monitor iniciado (detección de eventos reales)")

        self.timer = self.create_timer(0.5, self.analyze)

    def cmd_cb(self, msg):
        self.last_cmd = msg

    def speed_cb(self, msg):
        self.last_speed_limit = msg.speed_limit

    def detect_event(self):
        if self.last_cmd is None:
            return "NO_DATA"

        lin = self.last_cmd.linear.x
        ang = self.last_cmd.angular.z

        # 🔴 ESTOP / STOP
        if abs(lin) < 0.01 and abs(ang) < 0.01:
            if self.last_speed_limit == 0.0:
                return "ESTOP"
            return "STOP"

        # 🔴 TURN
        if abs(ang) > 0.3:
            if ang > 0:
                return "TURN_LEFT"
            else:
                return "TURN_RIGHT"

        # 🔴 SPEED STATES
        if self.last_speed_limit is not None:
            if self.last_speed_limit < 0.12:
                return "SLOW"
            else:
                return "NORMAL"

        return "MOVING"

    def analyze(self):
        event = self.detect_event()

        if event != self.current_event:
            now = time.time()
            duration = now - self.last_event_time

            self.get_logger().info(
                f"🎯 EVENTO: {event} | duración anterior: {duration:.2f}s"
            )

            self.current_event = event
            self.last_event_time = now

        # Debug continuo
        if self.last_cmd:
            self.get_logger().info(
                f"[DEBUG] lin={self.last_cmd.linear.x:.3f} "
                f"ang={self.last_cmd.angular.z:.3f} "
                f"speed_limit={self.last_speed_limit}"
            )


def main():
    rclpy.init()
    node = BridgeMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()