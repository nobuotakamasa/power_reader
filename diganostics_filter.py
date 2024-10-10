import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray
import sys
import signal

try:
    statusname = sys.args[1]
except:
    statusname = 'power_consumption_monitor'

print(statusname)
keys = [
    "cores average power consumption (Watt)",
    "gpu average power consumption (Watt)",
    "pkg average power consumption (Watt)",
    "ram average power consumption (Watt)",
]


hwid = {}
# Ctrl-Cが押されたときのハンドラ
def signal_handler(sig, frame):
    print("Current dictionary contents:")
    print(hwid)
    sys.exit(0)  # 正常終了

# SIGINT（Ctrl-C）のシグナルをキャッチするように設定
signal.signal(signal.SIGINT, signal_handler)

class DiagnosticsFilterNode(Node):
    def __init__(self):
        super().__init__('diagnostics_filter_node')
        self.subscription = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostics_callback0,
            10)
        self.subscription  # prevent unused variable warning

    #随時表示
    def diagnostics_callback0(self, msg):
        for status in msg.status:
            if statusname in status.name:
                self.get_logger().info(f"Hardware ID: {status.hardware_id}")
                for value in status.values:
                    self.get_logger().info(f"{value.key}: {value.value}")
    #最大値を求める
    def diagnostics_callback(self, msg):
        global hwid
        for status in msg.status:
            if statusname in status.name:
                hardwareid = hwid.get(status.hardware_id)
                if hardwareid == None:
                    newdict = {}
                    for key in keys:
                        newdict[key] = 0.0
                    hwid[status.hardware_id] = newdict
                    self.get_logger().info(f"add new dict : {status.hardware_id}")
                hardwareid = hwid.get(status.hardware_id)
                #print(hardwareid)
                for value in status.values:
                    key = value.key
                    max_value = hardwareid.get(key)
                    if max_value != None:
                        v = float(value.value)
                        if max_value < v:
                            hardwareid[key] = v
                            self.get_logger().info(f"Hardware ID: {status.hardware_id}")
                            self.get_logger().info(f"{key}:{hardwareid[key]}")

def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
