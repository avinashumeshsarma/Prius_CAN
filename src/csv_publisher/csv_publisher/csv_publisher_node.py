import rclpy
from rclpy.node import Node
import pandas as pd
from panda_msgs.msg import CANMessage  # Custom CAN message
import ament_index_python.packages
import os

class CsvPublisher(Node):
    def __init__(self):
        super().__init__('csv_publisher_node')

        # Define the publisher
        self.publisher_ = self.create_publisher(CANMessage, '/csv_can_data', 10)

        # Load the CSV file
        csv_file_path = os.path.join(
            ament_index_python.packages.get_package_share_directory('csv_publisher'),
            'config',
            '01.csv'
)

        self.csv_data = pd.read_csv(csv_file_path)
        self.index = 0

        # Publish data every second
        timer_period = 0.0001  # 10 Hz (adjust if needed)
        self.timer = self.create_timer(timer_period, self.publish_csv_data)

    def publish_csv_data(self):
        if self.index >= len(self.csv_data):
            self.get_logger().info('Finished publishing all CSV data.')
            return

        # Read the row
        row = self.csv_data.iloc[self.index]

        # Extract fields from CSV
        msg = CANMessage()
        msg.id = int(row['MessageID'], 16)  # Convert hex ID to integer

        # Convert hexadecimal message string to list of bytes
        hex_message = row['Message'].lstrip('0x').zfill(16)  # Pad to 16 hex characters
        msg.data = [int(hex_message[i:i+2], 16) for i in range(0, len(hex_message), 2)]

        msg.timestamp = int(float(row['Time']) * 1e6)  # Convert time to microseconds

        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Published ID={msg.id}, Data={msg.data}, Timestamp={msg.timestamp}'
        )

        self.index += 1


def main(args=None):
    rclpy.init(args=args)
    node = CsvPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
