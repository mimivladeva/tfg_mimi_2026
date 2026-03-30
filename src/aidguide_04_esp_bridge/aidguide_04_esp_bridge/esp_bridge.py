import math
import serial
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from nav2_msgs.action import FollowWaypoints, Spin
from nav2_msgs.msg import SpeedLimit
from action_msgs.msg import GoalStatus

from tf_transformations import euler_from_quaternion


SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200

SPEED_NORMAL = 0.18
SPEED_SLOW = 0.10
TURN_ANGLE = 0.6
BRIDGE_DISTANCE = 0.2


def yaw_to_quat(yaw):
    return (
        0.0,
        0.0,
        math.sin(yaw * 0.5),
        math.cos(yaw * 0.5)
    )


class ESPNav2Bridge(Node):

    def __init__(self):
        super().__init__("aidguide_esp_bridge")

        # ================= SERIAL =================
        self.ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.01)

        # ================= ACTION CLIENTS =================
        self.follow_client = ActionClient(self, FollowWaypoints, "follow_waypoints")
        self.spin_client = ActionClient(self, Spin, "spin")

        # ================= PUBS =================
        self.speed_pub = self.create_publisher(SpeedLimit, "/speed_limit", 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # ================= SUBS =================
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_cb,
            10
        )

        # ================= ESTADO =================
        self.state = "WAITING_START"
        self.goal_handle = None
        self.spin_goal_handle = None

        self.current_pose = None
        self.current_yaw = 0.0
        self.have_amcl = False

        self.waypoints = self.define_waypoints()
        self.current_waypoint = 0

        self.rebuilt_mission = False
        self.restart_idx = 0

        self.estop = False
        self.mission_started = False
        self.mission_finished = False

        self.spinning = False
        self.pending_spin_yaw = None
        self.cancel_in_progress = False

        # 🔥 protección post-reanudación
        self.ignore_speed_commands_until = 0.0

        # ================= TIMERS =================
        self.timer = self.create_timer(0.02, self.loop)
        self.start_timer = self.create_timer(1.0, self.try_start_mission)

        self.get_logger().info("Bridge ESP32 iniciado")

    # ================= AMCL =================

    def amcl_cb(self, msg):
        self.current_pose = msg.pose.pose

        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw

        if not self.have_amcl:
            self.have_amcl = True
            self.get_logger().info("AMCL listo")

    # ================= WAYPOINTS =================

    def define_waypoints(self):
        pts = [(0.5, 0.0), (0.8, 0.0), (1.2, 0.0), (2.0, 0.0)]
        poses = []

        for i in range(len(pts)):
            x, y = pts[i]

            if i < len(pts) - 1:
                nx, ny = pts[i + 1]
                yaw = math.atan2(ny - y, nx - x)
            else:
                yaw = 0.0

            qx, qy, qz, qw = yaw_to_quat(yaw)

            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            poses.append(pose)

        return poses

    def make_pose(self, x, y, yaw):
        qx, qy, qz, qw = yaw_to_quat(yaw)

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        return pose

    # ================= START =================

    def try_start_mission(self):
        if self.mission_started:
            return

        if not self.have_amcl:
            self.get_logger().warn("Esperando AMCL...")
            return

        if not self.follow_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn("Esperando follow_waypoints...")
            return

        self.get_logger().info("Iniciando misión")
        self.mission_started = True
        self.start_timer.cancel()
        self.start_mission()

    def start_mission(self):
        goal = FollowWaypoints.Goal()
        goal.poses = self.waypoints

        future = self.follow_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_cb
        )
        future.add_done_callback(self.goal_response_cb)

    # ================= REANUDAR =================

    def start_mission_from_current(self):
        if self.current_pose is None:
            self.get_logger().error("Sin AMCL")
            return

        idx = self.current_waypoint

        x = self.current_pose.position.x
        y = self.current_pose.position.y
        yaw = self.current_yaw

        bridge_x = x + BRIDGE_DISTANCE * math.cos(yaw)
        bridge_y = y + BRIDGE_DISTANCE * math.sin(yaw)

        bridge = self.make_pose(bridge_x, bridge_y, yaw)

        new_poses = [bridge] + self.waypoints[idx:]

        goal = FollowWaypoints.Goal()
        goal.poses = new_poses

        self.rebuilt_mission = True
        self.restart_idx = idx

        # 🔥 protección velocidad
        self.ignore_speed_commands_until = time.time() + 2.0

        self.get_logger().info(f"Reanudando desde {idx}")

        future = self.follow_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_cb
        )
        future.add_done_callback(self.goal_response_cb)

    # ================= CALLBACKS =================

    def goal_response_cb(self, future):
        gh = future.result()

        if not gh.accepted:
            self.get_logger().error("Goal rechazado")
            return

        self.goal_handle = gh
        self.state = "NAVIGATING"

        self.get_logger().info("Navegación iniciada")

        result_future = gh.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        result = future.result()

        if result.status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Nav2 ABORT")
            self.state = "PAUSED"

        elif result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Misión completada")
            self.state = "COMPLETED"

        self.goal_handle = None

    def feedback_cb(self, feedback):
        fw_idx = feedback.feedback.current_waypoint

        if self.rebuilt_mission:
            self.current_waypoint = self.restart_idx + max(fw_idx - 1, 0)
        else:
            self.current_waypoint = fw_idx

    # ================= CANCEL =================

    def cancel_goal(self):
        if self.goal_handle is None:
            return

        self.cancel_in_progress = True
        future = self.goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done_cb)

    def cancel_done_cb(self, future):
        self.goal_handle = None
        self.cancel_in_progress = False

        if self.pending_spin_yaw is not None:
            yaw = self.pending_spin_yaw
            self.pending_spin_yaw = None
            self.send_spin(yaw)

    # ================= SPIN =================

    def spin_robot(self, yaw):
        if self.spinning:
            return

        if self.goal_handle:
            self.pending_spin_yaw = yaw
            self.spinning = True
            self.cancel_goal()
            return

        self.spinning = True
        self.send_spin(yaw)

    def send_spin(self, yaw):
        goal = Spin.Goal()
        goal.target_yaw = float(yaw)

        future = self.spin_client.send_goal_async(goal)
        future.add_done_callback(self.spin_response_cb)

    def spin_response_cb(self, future):
        gh = future.result()

        if not gh.accepted:
            self.spinning = False
            return

        result_future = gh.get_result_async()
        result_future.add_done_callback(self.spin_done_cb)

    def spin_done_cb(self, future):
        self.spinning = False

        # pequeño delay estabilidad
        time.sleep(0.2)

        self.start_mission_from_current()

    # ================= SERIAL =================

    def read_serial(self):
        try:
            line = self.ser.readline().decode().strip()
            if line.startswith("CMD:"):
                return line.replace("CMD:", "").strip()
        except:
            pass
        return None

    # ================= CONTROL =================

    def publish_zero(self):
        t = Twist()
        self.cmd_pub.publish(t)

    def set_speed(self, speed):
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.speed_limit = float(speed)
        msg.percentage = False
        self.speed_pub.publish(msg)

    # ================= LOOP =================

    def loop(self):
        cmd = self.read_serial()

        if cmd is None:
            return

        self.get_logger().info(f"CMD: {cmd} | estado={self.state}")

        if cmd == "TURN_LEFT":
            self.spin_robot(TURN_ANGLE)

        elif cmd == "TURN_RIGHT":
            self.spin_robot(-TURN_ANGLE)

        elif cmd == "SLOW":
            if time.time() < self.ignore_speed_commands_until:
                self.get_logger().warn("Ignorando SLOW (estabilización)")
                return
            self.set_speed(SPEED_SLOW)

        elif cmd == "NORMAL":
            if time.time() < self.ignore_speed_commands_until:
                self.get_logger().warn("Ignorando NORMAL (estabilización)")
                return

            self.set_speed(SPEED_NORMAL)

            if self.state == "PAUSED":
                self.start_mission_from_current()

        elif cmd == "STOP":
            self.publish_zero()
            self.cancel_goal()
            self.state = "PAUSED"


def main():
    rclpy.init()
    node = ESPNav2Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()