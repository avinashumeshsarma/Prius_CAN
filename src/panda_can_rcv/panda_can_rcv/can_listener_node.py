import rclpy
from rclpy.node import Node
import time
from panda_can_rcv.can_interface import CANInterface
from panda_can_rcv.message_relay import MessageRelay
from panda_msgs.msg import CANMessage

class CANListenerNode(Node):
    def __init__(self):
        super().__init__('can_listener_node')

        # Load parameters from YAML
        self.declare_parameter('mode', 'real')
        self.declare_parameter('input_topic', '/test/can_input')
        self.declare_parameter('output_topic', '/can/decoded')

        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        if self.mode == 'real':
            self.can_interface = CANInterface()
            self.publisher = self.create_publisher(CANMessage, self.output_topic, 10)
            self.timer = self.create_timer(0.000002, self.read_and_publish)
            self.get_logger().info(f"Running in REAL mode, publishing to {self.output_topic}")

        elif self.mode == 'test':
            input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
            self.relay = MessageRelay(self, input_topic, self.output_topic)
            self.get_logger().info(f"Running in TEST mode, relaying messages from {input_topic} to {self.output_topic}")

    def read_and_publish(self):
        messages = self.can_interface.read_messages()
        for msg_id, data, src in messages:
            msg = CANMessage()
            msg.id = msg_id
            msg.data = list(data)
            msg.timestamp = int(time.time())
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CANListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
