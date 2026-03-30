import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav2_msgs.msg import SpeedLimit
from action_msgs.msg import GoalStatusArray

import time


class RobotTestMonitor(Node):

    def __init__(self):
        super().__init__("robot_test_monitor")

        self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 10)
        self.create_subscription(SpeedLimit, "/speed_limit", self.speed_callback, 10)
        self.create_subscription(
            GoalStatusArray,
            "/follow_waypoints/_action/status",
            self.status_callback,
            10
        )

        self.last_cmd = Twist()
        self.last_speed_limit = None

        self.tests = {
            "STOP": False,
            "SLOW": False,
            "NORMAL": False,
            "TURN_LEFT": False,
            "TURN_RIGHT": False,
            "ESTOP": False
        }

        self.last_movement_time = time.time()

        self.nav2_status = None
        self.mission_valid = True
        self.mission_started = False

        self.nav2_finished_time = None
        self.wait_after_finish = 2.0
        self.evaluated = False

        self.timer = self.create_timer(0.1, self.loop)

        self.get_logger().info("TEST MONITOR iniciado (preciso)")

    # =========================
    # CMD CALLBACK (PRECISO)
    # =========================

    def cmd_callback(self, msg):
        self.last_cmd = msg

        lin = msg.linear.x
        ang = msg.angular.z

        # -------------------------
        # STOP real
        # -------------------------
        if abs(lin) < 0.01 and abs(ang) < 0.01:
            if not self.tests["STOP"]:
                self.tests["STOP"] = True
                self.get_logger().info("TEST STOP -> PASS")

        # -------------------------
        # TURN real (SOLO SPIN)
        # -------------------------
        if abs(lin) < 0.05 and abs(ang) > 1.5:
            if ang > 0:
                if not self.tests["TURN_LEFT"]:
                    self.tests["TURN_LEFT"] = True
                    self.get_logger().info("TEST TURN_LEFT -> PASS")
            else:
                if not self.tests["TURN_RIGHT"]:
                    self.tests["TURN_RIGHT"] = True
                    self.get_logger().info("TEST TURN_RIGHT -> PASS")

        # -------------------------
        # MOVIMIENTO (para ESTOP)
        # -------------------------
        if abs(lin) > 0.02 or abs(ang) > 0.2:
            self.last_movement_time = time.time()

    # =========================
    # SPEED CALLBACK (FIABLE)
    # =========================

    def speed_callback(self, msg):
        self.last_speed_limit = msg.speed_limit

        # SLOW
        if abs(msg.speed_limit - 0.10) < 0.02:
            if not self.tests["SLOW"]:
                self.tests["SLOW"] = True
                self.get_logger().info("TEST SLOW -> PASS")

        # NORMAL
        if abs(msg.speed_limit - 0.18) < 0.03:
            if not self.tests["NORMAL"]:
                self.tests["NORMAL"] = True
                self.get_logger().info("TEST NORMAL -> PASS")

    # =========================
    # NAV2 STATUS
    # =========================

    def status_callback(self, msg):
        if len(msg.status_list) == 0:
            return

        status = msg.status_list[-1].status
        self.nav2_status = status

        if status == 2:
            self.mission_started = True

        if status == 6:
            self.get_logger().error("TEST: Nav2 ABORT detectado")
            self.mission_valid = False

        if status in [4, 5, 6] and self.nav2_finished_time is None:
            self.nav2_finished_time = time.time()
            self.get_logger().info("TEST: Nav2 terminado")

    # =========================
    # LOOP
    # =========================

    def loop(self):

        if self.evaluated:
            return

        # ESTOP real (parado demasiado tiempo)
        if time.time() - self.last_movement_time > 2.5:
            if not self.tests["ESTOP"]:
                self.tests["ESTOP"] = True
                self.get_logger().info("TEST ESTOP -> PASS")

        if self.nav2_finished_time is None:
            return

        if time.time() - self.nav2_finished_time < self.wait_after_finish:
            return

        self.evaluate_tests()
        self.evaluated = True

    # =========================
    # RESULTADOS
    # =========================

    def evaluate_tests(self):

        self.get_logger().info("======== RESULTADOS TEST ========")

        if not self.mission_valid:
            self.get_logger().error("RESULTADO FINAL -> FAIL (NAV2 ABORTÓ)")
            return

        if not self.mission_started:
            self.get_logger().error("RESULTADO FINAL -> FAIL (MISIÓN NO INICIADA)")
            return

        all_pass = True

        for name, result in self.tests.items():
            if result:
                self.get_logger().info(f"{name} -> PASS")
            else:
                self.get_logger().error(f"{name} -> FAIL")
                all_pass = False

        if all_pass:
            self.get_logger().info("RESULTADO FINAL -> TODO PASS")
        else:
            self.get_logger().error("RESULTADO FINAL -> SUSPENSE")


def main():
    rclpy.init()

    node = RobotTestMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()