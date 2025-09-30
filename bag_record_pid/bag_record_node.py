
import copy

from rclpy.node import Node
from std_msgs.msg import Bool
import yaml
import subprocess
import signal
import time
import os
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from datetime import datetime
import importlib

class BagRecorderNode(Node):
    """
    Generic bagging node that can record multiple bags at once with different
    topics and args. This also can run pre-bag and post-bag code, for example,
    if collecting metadata or confirming a confirm before bagging.
    """
    def __init__(self):
        super().__init__("bag_record_node")

        self.node_name = self.get_name() # Get the full node name, including namespace if any

        self.declare_parameter("cfg_path", "")
        self.declare_parameter("output_dir", "")
        self.declare_parameter("storage_config_dir", "")
        self.declare_parameter("best_effort_qos_sub", True)
        self.declare_parameter("heartbeat_mode", False)
        
        self.cfg_path = self.get_parameter("cfg_path").get_parameter_value().string_value
        self.output_dir = self.get_parameter("output_dir").get_parameter_value().string_value
        self.storage_config_dir = self.get_parameter("storage_config_dir").get_parameter_value().string_value
        self.best_effort_qos_sub = self.get_parameter("best_effort_qos_sub").get_parameter_value().bool_value
        self.heartbeat_mode = self.get_parameter("heartbeat_mode").get_parameter_value().bool_value
        
        self.active = False

        self.cfg = yaml.safe_load(open(self.cfg_path))
        self.pre_hooks = {k: [] for k in self.cfg['bags']}  # List of processing functions per bag: fn(node)
        self.post_hooks = {k: [] for k in self.cfg['bags']}
        self.load_hooks()

        # TODO: check if the output directory exists.
        # Exit if it does not exist.
        os.chdir(self.output_dir)

        self.command_prefix = ["ros2", "bag", "record"]
        self.commands = dict()
        self.add_topics()

        self.process = dict()

        if self.heartbeat_mode:
            # If we expect listen to a heartbeat to know when to run bagging or not
            # (Like with a GUI)
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
                QoSProfile(
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    depth=10
                ) if self.best_effort_qos_sub else reliable_qos
            )
            self.timer = self.timer = self.create_timer(0.5, self.pub_status_callback)
        else:
            # Run on init
            self.run()

    def pre_logger(self):
        """
        Run any desired code before bagging
        """
        for k, bag_hooks in self.pre_hooks.items():
            for pre_fn in bag_hooks:
                pre_fn(self)

    def post_logger(self):
        """
        Run any desired code after bagging
        """
        for k, bag_hooks in self.post_hooks.items():
            for post_fn in bag_hooks:
                post_fn(self)

    def add_topics(self):
        namespace = self.get_namespace()

        for bag_name, bag_config in self.cfg['bags'].items():
            self.commands[bag_name] = dict()

            # Command lists.
            self.commands[bag_name]['prefix'] = []
            self.commands[bag_name]['suffix'] = []

            # Populate the initial command line.
            self.commands[bag_name]['prefix'] += self.command_prefix

            # Add the args to the command line.
            self.commands[bag_name]['prefix'] += [str(x) for kv in bag_config['args'].items() for x in kv]

            # Set the mcap qos profile.
            if bag_config['storage_config'] != "":
                storage_config_path = os.path.join(self.storage_config_dir, bag_config['storage_config'])
                self.commands[bag_name]['prefix'].append('--storage-config-file')
                self.commands[bag_name]['prefix'].append(storage_config_path)

            self.get_logger().warn(
                f'CMD for bag {bag_name}: '
                f'{" ".join(self.commands[bag_name]["prefix"])}' )

            # Add the topics to the command at the end.
            self.get_logger().warn(f"Searching for bag topics {bag_name} topics:")
            for topic in bag_config['topics']:
                # Env var with many topics
                if topic.startswith('$'):
                    env_key = topic[1:]
                    expanded = os.getenv(env_key, "")
                    if expanded.strip() == "":
                        self.get_logger().error(f"Env var {env_key} not set or empty")
                        continue
                    # Split if the env var holds multiple topics separated by spaces
                    for t in expanded.split(" "):
                        self.commands[bag_name]['suffix'].append(t)
                        self.get_logger().warn(f"{t}")
                    continue
                # Single topic
                elif topic.startswith('/'):
                    full_topic_name = topic
                else:
                    full_topic_name = f"{namespace}/{topic}"

                self.commands[bag_name]['suffix'].append(full_topic_name)
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

    def load_hooks(self):
        for k, cfg in self.cfg['bags'].items():
            hooks = cfg.get("hooks", {})
            pre_hooks = hooks.get('pre_logger', [])
            post_hooks = hooks.get('post_logger', [])
            for mod_name in pre_hooks:
                mod = importlib.import_module(mod_name)
                if hasattr(mod, "pre_logger"):
                    self.pre_hooks[k].append(mod.pre_logger)
                    self.get_logger().info(f"Registered hooks from {mod_name}")
                else:
                    self.get_logger().error(f"Module {mod_name} has no pre_logger()")
            for mod_name in post_hooks:
                mod = importlib.import_module(mod_name)
                if hasattr(mod, "post_logger"):
                    self.post_hooks[k].append(mod.post_logger)
                    self.get_logger().info(f"Registered hooks from {mod_name}")
                else:
                    self.get_logger().error(f"Module {mod_name} has no post_logger()")

    def run(self):
        if not self.active:
            self.active = True

            time_suffix = f"{datetime.now().strftime('%Y%m%d_%H%M%S')}"

            for bag_name, command_dict in self.commands.items():
                self.pre_logger()
                cmd = copy.deepcopy(command_dict['prefix'])

                # Set the output filename.
                output_filename = f"{bag_name}_{time_suffix}"
                cmd.append('-o')
                cmd.append(output_filename)

                # Appending an empty string will cause the ros2 bag record to consider the space as a topic
                # and introduce an error.
                if len(command_dict['suffix']) > 0:
                    cmd += command_dict['suffix']

                self.process[bag_name] = dict()
                self.process[bag_name]['process'] = subprocess.Popen(cmd, preexec_fn=os.setsid)

                self.process[bag_name]['cmd'] = cmd
                self.process[bag_name]['output_filename'] = output_filename
                self.get_logger().warn(f"Started Recording bag {bag_name} with PID {self.process[bag_name]['process'].pid} to {output_filename}")

    def terminate_proc_and_children(self, subprocess_cmd: subprocess.Popen):
        try:
            # Send SIGINT to the process group
            os.killpg(os.getpgid(subprocess_cmd.pid), signal.SIGINT)
            # Wait a short period
            subprocess_cmd.wait(timeout=5)
        except subprocess.TimeoutExpired:
            # If still alive, escalate
            try:
                os.killpg(os.getpgid(subprocess_cmd.pid), signal.SIGTERM)
                subprocess_cmd.wait(timeout=2)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(subprocess_cmd.pid), signal.SIGKILL)
                subprocess_cmd.wait()
        except Exception as e:
            if rclpy.ok():
                self.get_logger().error(f"Failed to terminate process: {e}")

    def interrupt(self):
        if self.active:
            for bag_name, process in self.process.items():
                self.terminate_proc_and_children(process["process"])
                self.get_logger().info(f"Ending Recording of bag {bag_name} with PID {process['process'].pid}")
                self.get_logger().warn(f"Output filename: {process['output_filename']}")
            self.post_logger()
            self.active = False


def main(args=None):
    rclpy.init(args=args)
    node = BagRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.interrupt()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
