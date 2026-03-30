import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

#SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_PORT = "/dev/ttyAMA0"
SERIAL_BAUD = 115200


class ESP32Reader(Node):
    def __init__(self):
        super().__init__('esp32_reader')

        try:
            self.ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.01)
        except Exception as e:
            self.get_logger().error(f'No se pudo abrir {SERIAL_PORT}: {e}')
            raise

        self.pub = self.create_publisher(String, '/esp32/event', 10)
        self.timer = self.create_timer(0.016, self.loop)
        self.last_cmd = None

        self.get_logger().info(f'ESP32Reader escuchando en {SERIAL_PORT} @ {SERIAL_BAUD}')

    def loop(self):
        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                return

            if line.startswith("CMD:"):
                cmd = line.replace("CMD:", "").strip().upper()

                if not cmd:
                    return

                # Evita spam de repetición exacta
                if cmd == self.last_cmd:
                    return

                self.last_cmd = cmd

                msg = String()
                msg.data = cmd
                self.pub.publish(msg)
                self.get_logger().info(f'Publicado evento: {cmd}')

        except Exception as e:
            self.get_logger().warn(f'Error leyendo serie: {e}')


def main():
    rclpy.init()
    node = ESP32Reader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()