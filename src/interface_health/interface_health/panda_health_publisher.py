import rclpy
from rclpy.node import Node
from panda import Panda
from panda_msgs.msg import PandaHealthStatus  # Import the custom message

class PandaHealthPublisher(Node):
    def __init__(self):
        super().__init__('panda_health_publisher')
        
        # Connect to Panda device
        try:
            self.panda = Panda()
            self.get_logger().info("Connected to Panda device")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Panda device: {e}")
            return

        # Publisher
        self.publisher_ = self.create_publisher(PandaHealthStatus, 'panda/health', 10)
        
        # Timer to periodically publish data (every 1 second)
        self.timer = self.create_timer(1.0, self.publish_health)

    def publish_health(self):
        try:
            health = self.panda.health()
            msg = PandaHealthStatus(
                uptime=health['uptime'],
                voltage=health['voltage'],
                current=health['current'],
                safety_tx_blocked=health['safety_tx_blocked'],
                safety_rx_invalid=health['safety_rx_invalid'],
                tx_buffer_overflow=health['tx_buffer_overflow'],
                rx_buffer_overflow=health['rx_buffer_overflow'],
                faults=health['faults'],
                ignition_line=health['ignition_line'],
                ignition_can=health['ignition_can'],
                controls_allowed=health['controls_allowed'],
                car_harness_status=health['car_harness_status'],
                safety_mode=health['safety_mode'],
                safety_param=health['safety_param'],
                fault_status=health['fault_status'],
                power_save_enabled=health['power_save_enabled'],
                heartbeat_lost=health['heartbeat_lost'],
                alternative_experience=health['alternative_experience'],
                interrupt_load=health['interrupt_load'],
                fan_power=health['fan_power'],
                safety_rx_checks_invalid=health['safety_rx_checks_invalid'],
                spi_checksum_error_count=health['spi_checksum_error_count'],
                fan_stall_count=health['fan_stall_count'],
                sbu1_voltage_mv=health['sbu1_voltage_mV'],
                sbu2_voltage_mv=health['sbu2_voltage_mV'],
                som_reset_triggered=health['som_reset_triggered']
            )
            self.publisher_.publish(msg)
            self.get_logger().info("Published Panda health status")
        except Exception as e:
            self.get_logger().error(f"Error reading panda health: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PandaHealthPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
