
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from pacmod3_msgs.msg import VehicleSpeedRpt
import time

# Global variables for data buffer and time window
max_buffer = []
#time_window = 1.0  # 1 second
#topicname = "/ecu/currents/values"


def callback(msg):
    global max_buffer
    modfied = False
    if len(max_buffer) < len(msg.data):
        modfied = True
        for i in range(len(msg.data)):
            max_buffer.append(msg.data[i])
    for i in range(len(msg.data)):
        if msg.data[i] > max_buffer[i]:
            max_buffer[i] = msg.data[i]
            modfied = True
    if modfied:
        print(max_buffer)

def main(args=None):
    rclpy.init(args=args)

    # Create a node instance
    node = Node('current_max')
    #topic name
    topicname = "/ecu/currents/values"

    # Create the subscription and bind the callback function
    subscription1 = node.create_subscription(
        Float32MultiArray,
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
