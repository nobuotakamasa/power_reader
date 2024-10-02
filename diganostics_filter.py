import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray

class DiagnosticsFilterNode(Node):
    def __init__(self):
        super().__init__('diagnostics_filter_node')
        self.subscription = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostics_callback,
            10)
        self.subscription  # prevent unused variable warning

    def diagnostics_callback(self, msg):
        for status in msg.status:
            if 'power_consumption_monitor' in status.name:
                self.get_logger().info(f"Component: {status.name}")
                self.get_logger().info(f"Hardware ID: {status.hardware_id}")
                for value in status.values:
                    self.get_logger().info(f"{value.key}: {value.value}")

def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
