import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

SERIAL_PORT = "/dev/ttyAMA0"
SERIAL_BAUD = 115200


class ESP32Reader(Node):
    def __init__(self):
        super().__init__('esp32_reader')

        try:
            self.ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1)
            self.get_logger().info(f'Conectado a {SERIAL_PORT}')
        except Exception as e:
            self.get_logger().error(f'Error abriendo puerto: {e}')
            raise

        self.pub = self.create_publisher(String, '/esp32/event', 10)

        self.timer = self.create_timer(0.05, self.read_serial)

    def read_serial(self):
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()

                if line:
                    msg = String()
                    msg.data = line
                    self.pub.publish(msg)

                    self.get_logger().info(f"RX: {line}")

        except Exception as e:
            self.get_logger().error(f'Error leyendo UART: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ESP32Reader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()