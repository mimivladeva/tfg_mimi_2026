import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_msgs.msg import SpeedLimit

SPEED_NORMAL = 0.18
SPEED_SLOW = 0.10
TOL = 0.02  # tolerancia


class NavTester(Node):
    def __init__(self):
        super().__init__('nav_tester')

        self.last_cmd = None
        self.current_speed = None
        self.last_cmd_time = None

        self.create_subscription(String, '/esp32/event', self.event_cb, 10)
        self.create_subscription(SpeedLimit, '/speed_limit', self.speed_cb, 10)

        self.timer = self.create_timer(0.5, self.evaluate)  # cada 0.5s

        self.get_logger().info("=== TEST NAV2 INICIADO ===")
        

        self.sub_event = self.create_subscription(
            String,
            '/esp32/event',
            self.event_cb,
            10
        )

        self.sub_speed = self.create_subscription(
            SpeedLimit,
            '/speed_limit',
            self.speed_cb,
            10
        )

        self.get_logger().info("=== TEST NAV2 INICIADO ===")

    def event_cb(self, msg):
        self.last_cmd = msg.data.strip().upper()
        self.evaluate()

    def speed_cb(self, msg):
        self.current_speed = msg.speed_limit
        self.evaluate()

    def evaluate(self):
        if self.last_cmd is None or self.current_speed is None:
            return

        cmd = self.last_cmd
        speed = self.current_speed

        result = "UNKNOWN"

        if cmd == "SLOW":
            if abs(speed - SPEED_SLOW) < TOL:
                result = "PASS"
            else:
                result = "FAIL"

        elif cmd == "NORMAL":
            if abs(speed - SPEED_NORMAL) < TOL:
                result = "PASS"
            else:
                result = "FAIL"

        elif cmd == "STOP":
            if speed < 0.01:
                result = "PASS"
            else:
                result = "FAIL"

        elif cmd in ["TURN_LEFT", "TURN_RIGHT"]:
            result = "INFO (SPIN)"

        else:
            return

        self.print_result(cmd, speed, result)

    def print_result(self, cmd, speed, result):
        line = f"[TEST] CMD={cmd:10} | SPEED={speed:.3f} | RESULT={result}"
        
        if result == "PASS":
            self.get_logger().info(line)
        elif result == "FAIL":
            self.get_logger().error(line)
        else:
            self.get_logger().warn(line)


def main():
    rclpy.init()
    node = NavTester()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
