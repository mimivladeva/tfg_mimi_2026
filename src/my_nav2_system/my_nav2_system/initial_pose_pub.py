import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from builtin_interfaces.msg import Time

class InitialPosePublisher(Node):

    def __init__(self):
        super().__init__('initial_pose_pub')

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        self.timer = self.create_timer(4.0, self.publish_pose)
        self.sent = False

    def publish_pose(self):
        if self.sent:
            return

        msg = PoseWithCovarianceStamped()

        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.w = 1.0

        self.pub.publish(msg)

        self.get_logger().info("Initial pose enviada")
        self.sent = True


def main():
    rclpy.init()
    node = InitialPosePublisher()
    rclpy.spin(node)
    rclpy.shutdown()