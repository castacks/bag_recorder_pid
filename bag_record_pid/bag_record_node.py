
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
import datetime

class BagRecorderNode(Node):
    def __init__(self):
        super().__init__("bag_record_node")

        self.node_name = self.get_name() # Get the full node name, including namespace if any

        self.declare_parameter(
            "cfg_path", str(Path(__file__).parents[3] / "config/cfg.yaml")
        )

        self.declare_parameter(
            "output_dir", str("/logging/")
        )
        
        self.declare_parameter(
            "mcap_qos_dir", ""
        )
        
        self.cfg_path     = self.get_parameter("cfg_path").get_parameter_value().string_value
        self.output_dir   = self.get_parameter("output_dir").get_parameter_value().string_value
        self.mcap_qos_dir = self.get_parameter("mcap_qos_dir").get_parameter_value().string_value

        self.active = False
        self.cfg = yaml.safe_load(open(self.cfg_path))

        # TODO: check if the output directory exists.
        # Exit if it does not exist.
        os.chdir(self.output_dir)

        self.command_prefix = ["ros2", "bag", "record", "-s", "mcap"]
        self.commands = dict()
        self.add_topics()

        self.process = dict()
        
        # Create QoS profile for reliable communication
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=10
        )
        
        # Use reliable QoS for the status publisher
        self.status_pub = self.create_publisher(
            Bool, 
            f"{self.node_name}/bag_recording_status", 
            reliable_qos
        )
        
        self.toggle_status = self.create_subscription(
            Bool, 
            f"{self.node_name}/set_recording_status", 
            self.set_status_callback, 
            10
        )

        self.timer = self.create_timer(0.5, self.pub_status_callback)

    def add_topics(self):
        '''The configuration file looks like
        
            sections:
                gps_lidar_spot_depth_status:
                    mcap_qos: mcap_qos.yaml
                    args: 
                    - -b
                    - 4000000000 # ~4GB
                    - --max-cache-size 
                    - 1073741824 # 1GB
                    topics:
                    - /tf
                    - gq7/ekf/llh_position
                    
            The -o or --output argument should not be specified here.
            The "mcap_qos" field here will be interpreted as the filename of the MCAP QoS profile.
        '''
        namespace = self.get_namespace()
        
        for section_name, section_config in self.cfg['sections'].items():
            self.commands[section_name] = []
            
            # Populate the initial command line.
            self.commands[section_name].extend(self.command_prefix)
            
            # Add the args to the command line.
            str_args = [ str(c) for c in section_config['args'] ]
            self.commands[section_name].extend(str_args)
            
            # Set the output filename.
            self.commands[section_name].append('-o')
            self.commands[section_name].append(
                f"{section_name}_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}" )
            
            # Set the mcap qos profile.
            if section_config['mcap_qos'] != "":
                mcap_qos_path = os.path.join(self.mcap_qos_dir, str(section_config['mcap_qos']))
                self.commands[section_name].append('--storage-config-file')
                self.commands[section_name].append(mcap_qos_path)
            
            self.get_logger().warn(f'CMD for section {section_name}: {" ".join(self.commands[section_name])}')
            
            # Add the topics to the command at the end.
            self.get_logger().warn(f"Recording section {section_name} topics:")
            for topic in section_config['topics']:
                if topic.startswith('/'):
                    full_topic_name = topic
                else:
                    full_topic_name = f"{namespace}/{topic}"
                    
                self.commands[section_name].append(full_topic_name)
                self.get_logger().warn(f"{full_topic_name}")

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
            
            for section, command in self.commands.items():
                self.process[section] = dict()
                self.process[section]['process'] = subprocess.Popen(command)
                self.process[section]['pid'] = self.process[section]['process'].pid
                self.get_logger().info(f"Started Recording Section {section} with PID {self.process[section]['pid']}")

    def interrupt(self):
        if self.active:
            for section, process in self.process.items():
                process['process'].send_signal(signal.SIGINT)
                process['process'].wait()
                self.get_logger().info(f"Ending Recording of Section {section} with PID {process['pid']}")
            
            self.active = False


def main(args=None):
    rclpy.init(args=args)
    node = BagRecorderNode()
    rclpy.spin(node)
    node.interrupt()
        
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
