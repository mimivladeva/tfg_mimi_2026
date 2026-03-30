import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class NavToPose(Node):

    def __init__(self):
        super().__init__('nav_to_pose')

        self.client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.timer = self.create_timer(2.0, self.send_goal)
        self.sent = False

    def send_goal(self):
        if self.sent:
            return

        goal = NavigateToPose.Goal()

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = 1.0
        pose.pose.position.y = 0.0
        pose.pose.orientation.w = 0.0

        goal.pose = pose

        self.client.wait_for_server()

        self.client.send_goal_async(goal)

        self.get_logger().info("Goal enviado")
        self.sent = True


def main():
    rclpy.init()
    node = NavToPose()
    rclpy.spin(node)
    rclpy.shutdown()