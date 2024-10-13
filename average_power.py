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
    params = yaml.safe_load(file)

sensor_connections = params["sensor_input"]["connection"]
names = params["sensor_input"]["name"]

# データを表示
print("topic power")
print(params["sensor_input"]["voltage"])
print(params["sensor_input"]["current"])
print(params["sensor_input"]["name"])
#print(sensor_connections)
#for i in range(len(sensor_connections)):
#    print(f"{names[i]} {sensor_connections[i]}")

print("topic cpu_inner")
print(params["cpu_inner"]["value"])
print(params["cpu_inner"]["array"])

print("diagnostics")
statusname = params["diagnostics"]["statusname"]
print(statusname)

diagnostics = {}
diagnostics_count = {}
def print_diagnostics():
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
        #print(sum)
        print("------------- average --------------")
        num = diagnostics_count[hardware_id]
        for key in sum:
            average = sum[key] / num
            print(f"{key}, {average}")

def print_powers():
    sum = [0.0] * len(powers[0])
    for power in powers:
        for i in range(len(power)):
            sum[i] += power[i]
    print("------------- averages --------------")
    averages = list(map(lambda x: x/powers_count, sum))
    #print(powers_count, averages, sum)
    for i in range(len(averages)):
        print(f"{names[i]}, {averages[i]}")

# Ctrl-Cが押されたときのハンドラ
def signal_handler(sig, frame):
    print("======   printing averaged results   ======")
    print("====== diagnostics contents ======")
    print_diagnostics()
    print("======     sensor inputs    ======")
    print_powers()
    print("==================================")
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

voltages = [0.0,0.0]
powers = []
powers_count = 0
#currents = []
def callback_voltage(msg):
    global voltages
    voltages = msg.data
    #print(msg.params)

def callback_current(msg):
    #global currents
    global powers_count
    currents = msg.data
    power = []
    for i in range(len(currents)):
        #print(currents[i], voltages, type(sensor_connections[i]))
        power.append(currents[i] * voltages[sensor_connections[i]])
    powers.append(power)
    powers_count += 1
    #print(power)
    #print(voltages, currents)

def callback_topic_value(name, msg):
    print(name, msg.data)

def callback_topic_array(name, msg):
    print(name, msg.data)


def main(args=None):
    rclpy.init(args=args)

    # Create a node instance
    node = Node('array_average')
    subscriptions = []
    #collecting dignostics
    subscriptions.append(node.create_subscription(
        DiagnosticArray, '/diagnostics', diagnostics_callback,10))

    #collecting voltages
    subscriptions.append(node.create_subscription(
        Float32MultiArray,
        params["sensor_input"]["voltage"],
        callback_voltage, 10))

    #collecting currents
    subscriptions.append(node.create_subscription(
        Float32MultiArray,
        params["sensor_input"]["current"],
        callback_current, 100))

    topic_value = params["cpu_inner"]["value"]
    for name in topic_value:
        subscriptions.append(node.create_subscription(
            Float32,
            name,
            lambda x: callback_topic_value(name,x), 10))

    topic_array = params["cpu_inner"]["value"]
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