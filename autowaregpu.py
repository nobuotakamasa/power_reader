
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# Global variables for data buffer and time window
data_buffer = []
#time_window = 1.0  # 1 second
topicname = "/ecu/autowareecu/gpu"

max = 0.0

def callback(msg):
    global max
    v = msg.data
    if v > max:
        max = v
        print(max)

def main(args=None):
    rclpy.init(args=args)

    # Create a node instance
    node = Node('AutowareecuGPU')
    subscription = node.create_subscription(
        Float32,
        topicname,
        callback,
        100
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
