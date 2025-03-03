from rclpy.node import Node
from pathlib import Path
from std_msgs.msg import Bool
import yaml
import subprocess
import signal
import time
import os
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class BagRecorderNode(Node):
    def __init__(self):
        super().__init__("bag_record_node")

        self.declare_parameter(
            "cfg_path", str(Path(__file__).parents[3] / "config/cfg.yaml")
        )

        self.declare_parameter(
            "output_dir", str("/logging/")
        )
        
        self.cfg_path = (
            self.get_parameter("cfg_path").get_parameter_value().string_value
        )

        self.output_dir = (
            self.get_parameter("output_dir").get_parameter_value().string_value
        )

        self.active = False
        self.cfg = yaml.safe_load(open(self.cfg_path))

        # TODO: check if the output directory exists.
        # Exit if it does not exist.
        os.chdir(self.output_dir)

        self.commands = ["ros2", "bag", "record", "-s", "mcap"]
        self.add_topics()

        self.process = None
        
        # Create QoS profile for reliable communication
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=10
        )
        
        # Use reliable QoS for the status publisher
        self.status_pub = self.create_publisher(
            Bool, 
            "bag_recording_status", 
            reliable_qos
        )
        
        self.toggle_status = self.create_subscription(
            Bool, "set_recording_status", self.set_status_callback, 10
        )

        self.timer = self.create_timer(0.5, self.pub_status_callback)

    def add_topics(self):
        for topic in self.cfg["topics"]:
            self.commands.append(topic)

    def pub_status_callback(self):
        msg = Bool()
        msg.data = self.active
        self.status_pub.publish(msg)

    def set_status_callback(self, msg):
        if msg.data:
            self.run()
        else:
            self.interrupt()

    def run(self):
        if not self.active:
            self.active = True
            self.process = subprocess.Popen(self.commands)
            self.cur_pid = self.process.pid
            self.get_logger().info(f"Started Recording using PID {self.cur_pid}")

    def interrupt(self):
        if self.active:
            self.active = False
            self.process.send_signal(signal.SIGINT)
            self.process.wait()
            self.get_logger().info(f"Ending Recording of PID {self.process.pid}")


def main(args=None):
    rclpy.init(args=args)
    node = BagRecorderNode()
    rclpy.spin(node)
    node.interrupt()
        
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
