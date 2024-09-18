
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from pacmod3_msgs.msg import VehicleSpeedRpt
import time

# Global variables for data buffer and time window
data_buffer = []
#time_window = 1.0  # 1 second
#topicname = "/ecu/currents/values"

speed = None

def callback(msg):
    current_time = time.time()
    global data_buffer

    # Append the new data along with the current timestamp
    data_buffer.append((current_time, msg.data))

    # Remove data older than 1 second
    data_buffer = [(t, data) for t, data in data_buffer if current_time - t <= time_window]

    # Calculate the average over the last 1 second for each element in the array
    if data_buffer:
        avg_data = calculate_average()
        print(speed)
        print(f"1-second average: {avg_data}")

def calculate_average():
    num_elements = len(data_buffer[0][1])  # Length of the float32 array
    summed_data = [0.0] * num_elements

    # Sum all values for each element in the buffer
    for _, data in data_buffer:
        for i in range(num_elements):
            summed_data[i] += data[i]

    # Calculate the average by dividing by the number of samples
    avg_data = [sum_val / len(data_buffer) for sum_val in summed_data]
    return avg_data

def speed_callback(msg):
    speed = msg.vehicle_speed
    #print(f'Vehicle Speed: {msg.vehicle_speed:.2f} m/s')
    #print(f'Speed Valid: {msg.vehicle_speed_valid}')


def main(args=None):
    rclpy.init(args=args)

    # Create a node instance
    node = Node('current_subscriber')
    #interval time
    node.declare_parameter('time_window', 1.0)
    global time_window
    time_window = node.get_parameter('time_window').value
    #topic name
    node.declare_parameter('topicname', "/ecu/currents/values")
    topicname = node.get_parameter('topicname').value

    # Create the subscription and bind the callback function
    subscription1 = node.create_subscription(
        Float32MultiArray,
        topicname,
        callback,
        100
    )
    subscription2 = node.create_subscription(
            VehicleSpeedRpt,
            '/pacmod/vehicle_speed_rpt',
            speed_callback,
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
