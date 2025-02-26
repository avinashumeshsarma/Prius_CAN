from panda_msgs.msg import CANMessage

class MessageRelay:
    def __init__(self, node, input_topic, output_topic):
        self.publisher = node.create_publisher(CANMessage, output_topic, 10)
        self.subscription = node.create_subscription(
            CANMessage, input_topic, self.relay_message, 10
        )

    def relay_message(self, msg):
        self.publisher.publish(msg)
