import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav2_msgs.msg import SpeedLimit


class BridgeTester(Node):

    def __init__(self):
        super().__init__("bridge_tester")

        self.last_speed = None
        self.last_turn = None

        self.create_subscription(
            SpeedLimit,
            "/speed_limit",
            self.speed_cb,
            10)

        self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_cb,
            10)

        print("\nEsperando comandos del ESP32...\n")

    def speed_cb(self, msg):

        speed = msg.speed_limit

        if speed == self.last_speed:
            return

        self.last_speed = speed

        if abs(speed - 0.01) < 0.001:
            print("TEST STOP -> PASS")

        elif abs(speed - 0.10) < 0.001:
            print("TEST SLOW -> PASS")

        elif abs(speed - 0.18) < 0.001:
            print("TEST NORMAL -> PASS")

    def cmd_cb(self, msg):

        angular = msg.angular.z

        if abs(angular) < 0.2:
            return

        direction = "LEFT" if angular > 0 else "RIGHT"

        if direction == self.last_turn:
            return

        self.last_turn = direction

        if direction == "LEFT":
            if abs(angular - 0.6) < 0.1:
                print("TEST TURN_LEFT -> PASS")
            else:
                print("TEST TURN_LEFT -> FAIL")

        if direction == "RIGHT":
            if abs(angular + 0.6) < 0.1:
                print("TEST TURN_RIGHT -> PASS")
            else:
                print("TEST TURN_RIGHT -> FAIL")


def main():

    rclpy.init()

    node = BridgeTester()

    rclpy.spin(node)


if __name__ == "__main__":
    main()