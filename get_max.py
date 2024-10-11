
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import sys

#topicname = "/ecu/autowareecu/gpu"
topicname = sys.argv[1]

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
    node = Node('GetMaximum')
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
