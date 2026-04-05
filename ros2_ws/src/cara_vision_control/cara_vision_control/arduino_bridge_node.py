#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from cara.serial_control import initialize_arduino, send_behavior_command, close_arduino

class CaraArduinoBridgeNode(Node):
    def __init__(self):
        super().__init__('cara_arduino_bridge_node')

        initialize_arduino()

        self.sub = self.create_subscription(
            String,
            '/cara/behavior_cmd',
            self.cmd_callback,
            10
        )

        self.get_logger().info("Arduino bridge ready. Listening on /cara/behavior_cmd")

    def cmd_callback(self, msg):
        try:
            send_behavior_command(msg.data)
            self.get_logger().info(f"Executed behavior: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Failed to execute behavior command: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CaraArduinoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        close_arduino()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()