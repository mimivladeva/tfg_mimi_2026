import math
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped


def yaw_to_quat(yaw: float):
    """Convierte yaw (rad) a quaternion (x,y,z,w) asumiendo roll=pitch=0."""
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return (0.0, 0.0, qz, qw)


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


class WaypointFollowerClient(Node):
    def __init__(self):
        super().__init__('waypoint_follower_client')
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.get_logger().info('Waypoint Follower Client initialized')

    def build_waypoints_from_xy(self, pts_xy, min_sep=0.20):
        """
        Crea PoseStamped con yaw automático apuntando al siguiente punto.
        min_sep evita waypoints demasiado juntos (traj “temblorosa”).
        """
        # Filtrar puntos demasiado cercanos
        filtered = []
        for p in pts_xy:
            if not filtered or dist(p, filtered[-1]) >= min_sep:
                filtered.append(p)

        # Si al filtrar nos quedamos con 1 punto, no hay ruta
        if len(filtered) < 2:
            self.get_logger().error("Muy pocos waypoints tras filtrar. Sube min_sep o añade puntos.")
            return []

        now = self.get_clock().now().to_msg()
        waypoints = []

        for i in range(len(filtered)):
            x, y = filtered[i]

            # yaw hacia el siguiente punto (o repetir yaw del anterior si es el último)
            if i < len(filtered) - 1:
                nx, ny = filtered[i + 1]
                yaw = math.atan2(ny - y, nx - x)
            else:
                px, py = filtered[i - 1]
                yaw = math.atan2(y - py, x - px)

            qx, qy, qz, qw = yaw_to_quat(yaw)

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = now  # mismo timestamp para todos: ok en FollowWaypoints
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            waypoints.append(pose)

        return waypoints

    def define_waypoints(self):
        """
        Ruta suave (sin zigzag) en tu zona:
        - Baja en diagonal suave
        - Luego recto a la derecha
        Ajusta aquí si quieres otra forma.
        """
        pts_xy = [
    (-2.3, -2.9),
    (-2.0, -2.9),
    (-1.6, -2.9),
]
        # min_sep: 0.20–0.35 suele ir bien (más grande = menos “micro-correcciones”)
        return self.build_waypoints_from_xy(pts_xy, min_sep=0.25)

    def send_waypoints(self, waypoints):
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server no disponible')
            return

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints

        self.get_logger().info(f'Enviando {len(waypoints)} waypoints...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by server')
            return

        self.get_logger().info('Goal accepted, esperando resultado...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        current_waypoint = feedback_msg.feedback.current_waypoint
        self.get_logger().info(f'Currently at waypoint: {current_waypoint}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Missed waypoints: {len(result.missed_waypoints)}')
        self.get_logger().info(f'Error code: {result.error_code}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerClient()
    wps = node.define_waypoints()
    if wps:
        node.send_waypoints(wps)
        rclpy.spin(node)
    else:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()