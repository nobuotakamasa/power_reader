import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
import yaml
import sys
import signal

yamlfile = sys.argv[1]

# YAMLファイルを読み込む
with open(yamlfile, "r") as file:
    data = yaml.safe_load(file)

# データを表示
print("topic power")
print(data["sensor_input"]["voltage"])
print(data["sensor_input"]["current"])
print(data["sensor_input"]["connection"])

print("topic cpu_inner")
print(data["cpu_inner"]["value"])
print(data["cpu_inner"]["array"])

print("diagnostics")
statusname = data["diagnostics"]["statusname"]
print(statusname)

diagnostics = {}
diagnostics_count = {}
# Ctrl-Cが押されたときのハンドラ
def signal_handler(sig, frame):
    print("------------------------------------")
    print("diagnostics contents:")
    #print(diagnostics)
    for hardware_id in diagnostics:
        print(hardware_id)
        sum = {}
        for values in diagnostics[hardware_id]:
            for value in values:
                if sum.get(value.key) == None:
                    sum[value.key] = 0.0
                #print(f"{type(value.key)} {value.key}: {type(value.value)} {value.value}")
                sum[value.key] += float(value.value)
        print(sum)
        print("------------- average --------------")
        num = diagnostics_count[hardware_id]
        for key in sum:
            average = sum[key] / num
            print(f"{key}: {average}")
        print("------------------------------------")
    sys.exit(0)  # 正常終了

# SIGINT（Ctrl-C）のシグナルをキャッチするように設定
signal.signal(signal.SIGINT, signal_handler)
def diagnostics_callback(msg):
    for status in msg.status:
        if statusname in status.name:
            if diagnostics.get(status.hardware_id) == None:
                diagnostics[status.hardware_id] = []
                diagnostics_count[status.hardware_id] = 0
            diagnostics[status.hardware_id].append(status.values)
            diagnostics_count[status.hardware_id] += 1
            #print(f"Hardware ID: {status.hardware_id}")
            #for value in status.values:
            #    print(f"{value.key}: {value.value}")

voltages = []
powers = []
currents = []
def callback_voltage(msg):
    global voltages
    voltages = msg.data
    #print(msg.data)

def callback_current(msg):
    global currents
    global powers
    currents = msg.data
    print(voltages, currents)

def callback_topic_value(name, msg):
    print(name, msg.data)

def callback_topic_array(name, msg):
    print(name, msg.data)


def main(args=None):
    rclpy.init(args=args)

    # Create a node instance
    node = Node('array_average')
    subscriptions = []
    subscriptions.append(node.create_subscription(
        DiagnosticArray, '/diagnostics', diagnostics_callback,10))

    subscriptions.append(node.create_subscription(
        Float32MultiArray,
        data["sensor_input"]["voltage"],
        callback_voltage, 10))

    subscriptions.append(node.create_subscription(
        Float32MultiArray,
        data["sensor_input"]["current"],
        callback_current, 100))

    topic_value = data["cpu_inner"]["value"]
    for name in topic_value:
        subscriptions.append(node.create_subscription(
            Float32,
            name,
            lambda x: callback_topic_value(name,x), 10))

    topic_array = data["cpu_inner"]["value"]
    for name in topic_array:
        subscriptions.append(node.create_subscription(
            Float32MultiArray,
            name,
            lambda x: callback_topic_array(name,x), 10))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()