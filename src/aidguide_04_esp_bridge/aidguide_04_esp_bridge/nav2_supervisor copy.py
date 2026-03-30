import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import FollowWaypoints, Spin
from nav2_msgs.msg import SpeedLimit
from action_msgs.msg import GoalStatus

SPEED_NORMAL = 0.18
SPEED_SLOW = 0.10
TURN_ANGLE = 0.6


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class Nav2Supervisor(Node):
    def __init__(self):
        super().__init__('nav2_supervisor')

        self.state = 'IDLE'

        self.follow_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.spin_client = ActionClient(self, Spin, 'spin')

        self.speed_pub = self.create_publisher(SpeedLimit, '/speed_limit', 10)
        self.event_sub = self.create_subscription(String, '/esp32/event', self.event_cb, 10)

        self.goal_handle = None
        self.waypoints = self.define_waypoints()
        self.resume_index = 0
        self.current_waypoint = 0
        self.pending_resume_after_spin = False

        self.completed_logged = False
        self.expected_cancel = False  # 🔴 CLAVE

        self.get_logger().info('Nav2Supervisor iniciado')
        self.startup_timer = self.create_timer(1.0, self.try_start_mission)

    def log_state_change(self, new_state: str, reason: str):
        old = self.state
        self.state = new_state
        self.get_logger().info(f'Estado: {old} -> {new_state} | motivo: {reason}')

    def define_waypoints(self):
        pts = [(1.0, 0.0), (1.5, 0.0), (2.0, 0.0)]
        poses = []

        for i, (x, y) in enumerate(pts):
            yaw = 0.0
            if i < len(pts) - 1:
                nx, ny = pts[i + 1]
                yaw = math.atan2(ny - y, nx - x)

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation = yaw_to_quat(yaw)
            poses.append(pose)

        return poses

    def try_start_mission(self):
        if self.state != 'IDLE':
            return

        if not self.follow_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn('Esperando a FollowWaypoints...')
            return

        self.get_logger().info('Nav2 listo → iniciando misión')
        self.startup_timer.cancel()
        self.start_mission_from(self.resume_index)

    def set_speed(self, speed: float):
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.speed_limit = float(speed)
        msg.percentage = False
        self.speed_pub.publish(msg)

    def start_mission_from(self, idx: int):
        if idx >= len(self.waypoints):
            self.log_state_change('COMPLETED', 'misión finalizada')
            return

        goal = FollowWaypoints.Goal()
        goal.poses = self.waypoints[idx:]
        self.resume_index = idx

        future = self.follow_client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        future.add_done_callback(self.follow_goal_response_cb)

    def follow_goal_response_cb(self, future):
        gh = future.result()

        if not gh.accepted:
            self.log_state_change('IDLE', 'goal rechazado')
            return

        self.goal_handle = gh
        self.log_state_change('NAVIGATING', 'goal aceptado')

        result_future = gh.get_result_async()
        result_future.add_done_callback(self.follow_result_cb)

    def feedback_cb(self, feedback_msg):
        self.current_waypoint = self.resume_index + feedback_msg.feedback.current_waypoint

        if self.current_waypoint != getattr(self, "_last_wp", -1):
            self._last_wp = self.current_waypoint
            self.get_logger().info(f'Waypoint: {self.current_waypoint}')

    def follow_result_cb(self, future):
        result = future.result()
        status = result.status
        self.goal_handle = None

        self.get_logger().info(f'FollowWaypoints result status: {status}')

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.expected_cancel = False
            self.log_state_change('COMPLETED', 'misión completada')

        elif status == GoalStatus.STATUS_CANCELED:
            if self.expected_cancel:
                self.expected_cancel = False
                self.get_logger().info('Cancelación controlada OK')
                return

            self.log_state_change('IDLE', 'cancelación inesperada')

        elif status == GoalStatus.STATUS_ABORTED:
            if self.expected_cancel or self.state in ['PAUSED', 'SPINNING', 'ESTOP']:
                self.expected_cancel = False
                self.get_logger().warn('Abort ignorado (controlado)')
                return

            self.log_state_change('IDLE', 'abort real')

    def cancel_navigation(self, on_done):
        if self.goal_handle is None:
            on_done()
            return

        self.expected_cancel = True
        future = self.goal_handle.cancel_goal_async()

        def _cb(_):
            on_done()

        future.add_done_callback(_cb)

    def do_spin(self, angle):
        goal = Spin.Goal()
        goal.target_yaw = float(angle)

        future = self.spin_client.send_goal_async(goal)
        future.add_done_callback(self.spin_goal_response_cb)

    def spin_goal_response_cb(self, future):
        gh = future.result()

        if not gh.accepted:
            self.log_state_change('PAUSED', 'spin rechazado')
            return

        result_future = gh.get_result_async()
        result_future.add_done_callback(self.spin_result_cb)

    def spin_result_cb(self, future):
        result = future.result()

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            if self.pending_resume_after_spin:
                self.pending_resume_after_spin = False
                self.start_mission_from(self.current_waypoint)
            else:
                self.log_state_change('PAUSED', 'spin completado')
        else:
            self.log_state_change('PAUSED', 'spin fallido')

    def event_cb(self, msg):
        cmd = msg.data.strip().upper()

        if self.state == 'COMPLETED':
            return

        if cmd == 'ESTOP':
            def done():
                self.log_state_change('ESTOP', 'emergency stop')
            self.cancel_navigation(done)
            return

        if self.state == 'ESTOP':
            if cmd in ['RESET', 'NORMAL']:
                self.log_state_change('PAUSED', 'salida ESTOP')
            return

        if cmd == 'SLOW':
            self.set_speed(SPEED_SLOW)
            return

        if cmd == 'NORMAL':
            self.set_speed(SPEED_NORMAL)
            if self.state == 'PAUSED':
                self.start_mission_from(self.current_waypoint)
            return

        if cmd == 'STOP' and self.state == 'NAVIGATING':
            def done():
                self.log_state_change('PAUSED', 'STOP confirmado')
            self.cancel_navigation(done)
            return

        if cmd == 'TURN_LEFT':
            def done():
                self.log_state_change('SPINNING', 'turn left')
                self.do_spin(+TURN_ANGLE)

            if self.state == 'NAVIGATING':
                self.pending_resume_after_spin = True
                self.cancel_navigation(done)
            elif self.state == 'PAUSED':
                done()
            return

        if cmd == 'TURN_RIGHT':
            def done():
                self.log_state_change('SPINNING', 'turn right')
                self.do_spin(-TURN_ANGLE)

            if self.state == 'NAVIGATING':
                self.pending_resume_after_spin = True
                self.cancel_navigation(done)
            elif self.state == 'PAUSED':
                done()
            return


def main():
    rclpy.init()
    node = Nav2Supervisor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()