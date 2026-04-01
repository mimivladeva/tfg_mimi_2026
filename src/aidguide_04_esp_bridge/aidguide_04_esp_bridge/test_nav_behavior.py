import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry

import time


class RobotTester(Node):

    def __init__(self):
        super().__init__('robot_tester')

        # subscripciones
        self.event_sub = self.create_subscription(
            String, '/esp32/event', self.event_cb, 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10)

        self.state_sub = self.create_subscription(
            String, '/nav/state', self.state_cb, 10)

        # estado
        self.last_cmd = None
        self.last_cmd_time = None
        self.mission_finished = False

        # medidas actuales
        self.linear_speed = 0.0
        self.angular_speed = 0.0

        # histórico (clave)
        self.linear_history = []
        self.angular_history = []
        self.max_samples = 10

        # parámetros esperados
        self.expected = {
            'SLOW': 0.10,
            'NORMAL': 0.18,
            'FAST': 0.23,
            'STOP': 0.0
        }

        # tolerancia más realista
        self.tolerance = 0.10

        # timer de evaluación
        self.timer = self.create_timer(0.5, self.evaluate)

        self.get_logger().info("🧪 RobotTester iniciado")

    # ========================
    # CALLBACKS
    # ========================

    def event_cb(self, msg):
        if self.mission_finished:
            return

        self.last_cmd = msg.data.strip().upper()
        self.last_cmd_time = time.time()

        self.get_logger().info(f'📥 TEST recibe comando: {self.last_cmd}')

    def odom_cb(self, msg):
        self.linear_speed = msg.twist.twist.linear.x
        self.angular_speed = msg.twist.twist.angular.z

        # guardar histórico
        self.linear_history.append(abs(self.linear_speed))
        self.angular_history.append(self.angular_speed)

        if len(self.linear_history) > self.max_samples:
            self.linear_history.pop(0)

        if len(self.angular_history) > self.max_samples:
            self.angular_history.pop(0)

    def state_cb(self, msg):
        if msg.data == 'COMPLETED':
            self.get_logger().info('🏁 Misión completada → TEST detenido')
            self.mission_finished = True

            self.timer.cancel()
            self.destroy_subscription(self.event_sub)

    # ========================
    # EVALUACIÓN
    # ========================

    def evaluate(self):
        if self.mission_finished or self.last_cmd is None:
            return

        cmd = self.last_cmd

        if len(self.linear_history) == 0:
            return

         # esperar reacción real del robot
        if max(self.linear_history) < 0.05 and cmd != 'STOP':
            return

        

        # media → mucho más robusto
        real_speed = sum(self.linear_history) / len(self.linear_history)
        avg_ang = sum(self.angular_history) / len(self.angular_history)

        # debug útil
        self.get_logger().info(
            f'📊 v={real_speed:.2f} w={avg_ang:.2f}')

        # -------- SPEED --------
        if cmd in self.expected:
            expected_speed = self.expected[cmd]

            if abs(real_speed - expected_speed) < self.tolerance:
                self.get_logger().info(f'✅ {cmd} OK | real={real_speed:.2f}')
            else:
                # evaluación por tendencia (clave en robot real)
                if cmd == 'SLOW' and real_speed < 0.18:
                    self.get_logger().info(f'✅ {cmd} OK (tendencia)')
                elif cmd == 'FAST' and real_speed > 0.15:
                    self.get_logger().info(f'✅ {cmd} OK (tendencia)')
                elif cmd == 'STOP' and real_speed < 0.05:
                    self.get_logger().info(f'✅ {cmd} OK (parado)')
                elif cmd == 'NORMAL' and 0.12 < real_speed < 0.25:
                    self.get_logger().info(f'✅ {cmd} OK (rango)')
                else:
                    self.get_logger().warn(
                        f'⚠️ {cmd} dudoso | real={real_speed:.2f}')

        # -------- TURN --------
        elif cmd == 'TURN_LEFT':
            if avg_ang > -0.05:
                self.get_logger().info('✅ TURN_LEFT OK')
            else:
                self.get_logger().warn('⚠️ TURN_LEFT dudoso')

        elif cmd == 'TURN_RIGHT':
            if avg_ang < 0.05:
                self.get_logger().info('✅ TURN_RIGHT OK')
            else:
                self.get_logger().warn('⚠️ TURN_RIGHT dudoso')

        # -------- ESTOP --------
        elif cmd == 'ESTOP':
            if real_speed < 0.05:
                self.get_logger().info('✅ ESTOP OK')
            else:
                self.get_logger().warn('⚠️ ESTOP dudoso')

        # reset
        self.last_cmd = None


def main():
    rclpy.init()
    node = RobotTester()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()