import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from panda import Panda

class PandaVoltagePublisher(Node):
    def __init__(self):
        super().__init__('panda_voltage_publisher')
        self.publisher_ = self.create_publisher(Float32, '/panda/voltage', 10)
        self.timer = self.create_timer(1.0, self.publish_voltage)  # Publish every second
        try:
            self.panda = Panda()
            self.get_logger().info(f'Connected to Panda: {self.panda.get_serial()}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Panda: {e}')
            self.panda = None

    def publish_voltage(self):
        if self.panda:
            try:
                voltage = float(self.panda.health()['voltage'])
                msg = Float32()
                msg.data = voltage/1000
                self.publisher_.publish(msg)
                self.get_logger().info(f'Published Voltage: {voltage/1000:.2f} V')
            except Exception as e:
                self.get_logger().error(f'Error reading voltage: {e}')
        else:
            self.get_logger().error("Panda device not connected!")

def main(args=None):
    rclpy.init(args=args)
    node = PandaVoltagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


