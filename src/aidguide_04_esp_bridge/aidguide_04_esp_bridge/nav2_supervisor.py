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
TURN_ANGLE = 0.20


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

        self.expected_cancel = False
        self.mission_started = False

        # 🔴 control arranque robusto
        self.start_pending = False
        self.start_delay_timer = None

        # 🔴 bloqueo de comandos
        self.command_busy = False
        self.active_command = None

        self.get_logger().info('Nav2Supervisor iniciado')
        self.startup_timer = self.create_timer(1.0, self.try_start_mission)
        #activar cojmandos 
        self.commands_enabled = False
        self.mission_completed = False
        self.goal_start_index = 0
        self.enable_timer = None

    def log_state_change(self, new_state: str, reason: str):
        old = self.state
        self.state = new_state
        self.get_logger().info(f'Estado: {old} -> {new_state} | motivo: {reason}')

    def define_waypoints(self):
        pts = [(0.5, 0.0), (1.0, 0.0), (1.5, 0.0), (2.0 ,0.0)]
        #pts = [(1.0, 0.0), (1.5, 0.15), (2.0, 0.1), (2.3, 0.0)]
        #pts = [(1.0, 0.0),(1.5, 0.0),(2.1, 0.15),(2.3, 0.0)]
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
        if self.mission_completed:
            return
        
        if self.state != 'IDLE':
            return

        if self.goal_handle is not None:
            return

        if self.start_pending:
            return

        if not self.follow_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn('Esperando a FollowWaypoints...')
            return

        self.get_logger().info('Nav2 listo → esperando estabilización')
        self.start_pending = True
        self.start_delay_timer = self.create_timer(2.5, self.delayed_start_mission)

    def delayed_start_mission(self):
        self.start_delay_timer.cancel()
        self.start_pending = False

        self.get_logger().info('Iniciando misión')
        self.start_mission_from(self.resume_index)

    def set_speed(self, speed: float):
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.speed_limit = float(speed)
        msg.percentage = False
        self.speed_pub.publish(msg)

    def start_mission_from(self, idx: int):
        if idx >= len(self.waypoints):
            self.mission_completed = True
            self.commands_enabled = False
            self.mission_started = False
            self.log_state_change('COMPLETED', 'misión finalizada')
            return


        self.mission_completed = False
        self.get_logger().info(f'Enviando misión desde waypoint {idx}')

        goal = FollowWaypoints.Goal()
        goal.poses = self.waypoints[idx:]

        self.goal_start_index = idx
        self.resume_index = idx
        self.current_waypoint = -1

        future = self.follow_client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        future.add_done_callback(self.follow_goal_response_cb)

    def enable_commands(self):
        if not self.commands_enabled:
            self.commands_enabled = True
            self.get_logger().info("🟢 Comandos ESP32 ACTIVADOS")
          
        if self.enable_timer is not None:
            self.enable_timer.cancel()
            self.enable_timer = None

    def follow_goal_response_cb(self, future):
        gh = future.result()

        if not gh.accepted:
            self.log_state_change('IDLE', 'goal rechazado')
            return

        self.goal_handle = gh
        self.mission_started = True
        self.log_state_change('NAVIGATING', 'goal aceptado')
        # 🔴 activar comandos con delay
        # cancelar timer anterior si existe
        if self.enable_timer is not None:
            self.enable_timer.cancel()
            self.enable_timer = None
        self.commands_enabled = False  # importante reset
        self.enable_timer = self.create_timer(2.0, self.enable_commands)

        gh.get_result_async().add_done_callback(self.follow_result_cb)

    def feedback_cb(self, feedback_msg):
        #self.current_waypoint = self.resume_index + feedback_msg.feedback.current_waypoint
        #wp = self.resume_index + feedback_msg.feedback.current_waypoint OJO
        wp = feedback_msg.feedback.current_waypoint
        global_wp = self.goal_start_index + wp
         # Solo imprime si cambia
        if wp != self.current_waypoint:
            if self.current_waypoint != -1:
                #self.resume_index = self.current_waypoint   # 🔴 CLAVE
                #self.get_logger().info(f'✅ Waypoint alcanzado: {self.current_waypoint}')
                reached_global = self.goal_start_index + self.current_waypoint
                self.resume_index = reached_global
                self.get_logger().info(f'✅ Waypoint alcanzado: {reached_global}')
                
            self.current_waypoint = wp
            #self.get_logger().info(f'➡️ Navegando hacia waypoint: {wp}')
            self.get_logger().info(f'➡️ Navegando hacia waypoint: {global_wp}')

    def follow_result_cb(self, future):
        result = future.result()
        status = result.status
        self.goal_handle = None

        self.get_logger().info(f'FollowWaypoints result status: {status}')

        self.end_command()

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.mission_completed = True
            self.commands_enabled = False
            self.mission_started = False
            self.pending_resume_after_spin = False
            self.expected_cancel = False
            self.set_speed(0.0)
            self.log_state_change('COMPLETED', 'misión completada')

        elif status == GoalStatus.STATUS_ABORTED:
            self.expected_cancel = False
            self.pending_resume_after_spin = False
            self.commands_enabled = False
            self.mission_started = False
            #self.abort_happened = True   # 🔴 IMPORTANTE
             # 🔴 IMPORTANTE: reanudar desde donde estaba
            next_wp = max(0, self.resume_index)
            self.get_logger().warn(f'⚠️ Abort → reanudando desde waypoint {next_wp}')
            self.start_mission_from(next_wp)
            #self.log_state_change('IDLE', 'abort real')

        elif status == GoalStatus.STATUS_CANCELED:
            if not self.expected_cancel:
                self.log_state_change('IDLE', 'cancelación inesperada')

    def cancel_navigation(self, on_done):
        if self.goal_handle is None:
            on_done()
            return

        self.expected_cancel = True
        future = self.goal_handle.cancel_goal_async()
        future.add_done_callback(lambda _: on_done())

    def do_spin(self, angle):
        goal = Spin.Goal()
        goal.target_yaw = float(angle)
        goal.time_allowance.sec = 5

        future = self.spin_client.send_goal_async(goal)
        future.add_done_callback(self.spin_goal_response_cb)

    def spin_goal_response_cb(self, future):
        gh = future.result()

        if not gh.accepted:
            self.log_state_change('PAUSED', 'spin rechazado')
            self.end_command()
            return

        gh.get_result_async().add_done_callback(self.spin_result_cb)

    def spin_result_cb(self, future):
        result = future.result()

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            if self.pending_resume_after_spin:
                self.pending_resume_after_spin = False

                #next_wp = self.current_waypoint
                next_wp = self.goal_start_index + self.current_waypoint
                self.set_speed(SPEED_SLOW)
                self.log_state_change('NAVIGATING', f'reanudando desde waypoint {next_wp}')
                self.start_mission_from(next_wp)

        self.end_command()

    # 🔴 gestión comandos
    def begin_command(self, name):
        if self.command_busy:
            self.get_logger().warn(f'Ignorado {name}, ocupado con {self.active_command}')
            return False

        self.command_busy = True
        self.active_command = name
        return True

    def end_command(self):
        self.command_busy = False
        self.active_command = None

    def event_cb(self, msg):
        cmd = msg.data.strip().upper()
        self.get_logger().info(f'📥 Comando recibido: {cmd}') 

        if self.mission_completed or self.state == 'COMPLETED':
            self.get_logger().warn('⛔ Ignorado: misión ya completada')
            return

        if not self.mission_started:
            self.get_logger().warn('⛔ Ignorado: misión no iniciada')
            return
        if not self.commands_enabled:
            self.get_logger().warn('⛔ Ignorado: comandos deshabilitados')
            return

        if cmd == 'ESTOP':
            self.command_busy = False

            if self.begin_command('ESTOP'):
                self.cancel_navigation(lambda: self.log_state_change('ESTOP', 'emergency'))
            return

        if self.command_busy:
            return

        if cmd == 'SLOW':
            self.set_speed(SPEED_SLOW)
            return

        if cmd == 'NORMAL':
            self.set_speed(SPEED_NORMAL)
            return

        if cmd == 'STOP':
            # if self.begin_command('STOP'):
            #     self.cancel_navigation(lambda: self.log_state_change('PAUSED', 'STOP'))
            # return
            self.set_speed(0.0)
            return

        if cmd in ['TURN_LEFT', 'TURN_RIGHT']:
            if self.state != 'NAVIGATING':
                self.get_logger().warn(f'⛔ Ignorado {cmd}: no está navegando')
                return

            if not self.begin_command(cmd):
                return

            angle = TURN_ANGLE if cmd == 'TURN_LEFT' else -TURN_ANGLE

            def done():
                self.log_state_change('SPINNING', cmd)
                self.do_spin(angle)

            self.pending_resume_after_spin = True

            if self.goal_handle:
                self.cancel_navigation(done)
            else:
                done()


def main():
    rclpy.init()
    node = Nav2Supervisor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()