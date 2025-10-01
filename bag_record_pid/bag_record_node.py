
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
    topics and args..
    Can be used directly from Python (passing args to __init__) or via ROS2 params.
    """
    def __init__(self,
                 config=None,
                 storage_config_dir=None,
                 best_effort_qos_sub=None,
                 heartbeat_mode=None):
        super().__init__("bag_record_node")

        self.node_name = self.get_name()

        # Declare parameters (still works with ros2 launch / params)
        self.declare_parameter("config", "")
        self.declare_parameter("output", "")
        self.declare_parameter("storage_config_dir", "")
        self.declare_parameter("best_effort_qos_sub", True)
        self.declare_parameter("heartbeat_mode", False)

        # If arguments were passed to __init__, they override parameters
        self.config = config or self.get_parameter("config").get_parameter_value().string_value
        self.storage_config_dir = storage_config_dir or self.get_parameter("storage_config_dir").get_parameter_value().string_value
        self.best_effort_qos_sub = (
            best_effort_qos_sub
            if best_effort_qos_sub is not None
            else self.get_parameter("best_effort_qos_sub").get_parameter_value().bool_value
        )
        self.heartbeat_mode = (
            heartbeat_mode
            if heartbeat_mode is not None
            else self.get_parameter("heartbeat_mode").get_parameter_value().bool_value
        )

        self.active = False

        self.command_prefix = ["ros2", "bag", "record"]
        self.commands = {}
        self.add_topics()

        self.process = {}

        if self.heartbeat_mode:
            # Heartbeat / toggle mode
            reliable_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                depth=10
            )
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
            self.timer = self.create_timer(0.5, self.pub_status_callback)
        else:
            # Auto-run immediately
            self.run()

    def add_topics(self):
        namespace = self.get_namespace()

        for bag_name, bag_config in self.config['bags'].items():
            self.commands[bag_name] = dict()

            # Command lists.
            self.commands[bag_name]['prefix'] = []
            self.commands[bag_name]['suffix'] = []
            self.commands[bag_name]['pseudo_suffix'] = [] # used for env vars

            # Populate the initial command line.
            self.commands[bag_name]['prefix'] += self.command_prefix

            # Add the args to the command line.
            self.commands[bag_name]['prefix'] += ["--{}={}".format(k, v) for k,v in bag_config['args'].items() if v is not None and k != 'config']

            # Add the topics to the command at the end.
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
                    self.commands[bag_name]['pseudo_suffix'].append(topic)
                    continue
                # Single topic
                elif topic.startswith('/'):
                    full_topic_name = topic
                else:
                    full_topic_name = f"{namespace}/{topic}"

                self.commands[bag_name]['suffix'].append(full_topic_name)

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


            for bag_name, command_dict in self.commands.items():
                cmd = copy.deepcopy(command_dict['prefix'])
                psuedo_cmd = copy.deepcopy(command_dict['prefix'])

                # Appending an empty string will cause the ros2 bag record to consider the space as a topic
                # and introduce an error.
                if len(command_dict['suffix']) > 0:
                    cmd += command_dict['suffix']
                if len(command_dict['pseudo_suffix']) > 0:
                    psuedo_cmd += command_dict['pseudo_suffix']

                if psuedo_cmd != command_dict['prefix']:
                    self.get_logger().info(
                        f'CMD for bag {bag_name}: '
                        f'{" ".join(psuedo_cmd)}')
                else:
                    self.get_logger().info(
                        f'CMD for bag {bag_name}: '
                        f'{" ".join(cmd)}')

                self.process[bag_name] = dict()
                self.process[bag_name]['process'] = subprocess.Popen(
                    cmd,
                    preexec_fn=os.setsid,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True
                )
                self.process[bag_name]['cmd'] = cmd
                self.process[bag_name]['output'] = self.config['bags'][bag_name]['args']['output']

                # Give it a short moment to either start or die
                time.sleep(0.5)

                ret = self.process[bag_name]['process'].poll()
                if ret is not None and ret != 0:
                    stderr_out = self.process[bag_name]['process'].stderr.read()
                    self.get_logger().error(f"Bagging process for {bag_name} failed immediately with code {ret}: {stderr_out}")
                    self.interrupt() # kill node immediately
                    raise RuntimeError(f"Bag {bag_name} failed. {ret}: {stderr_out}")

                self.get_logger().info(f"Started Recording bag {bag_name} with PID {self.process[bag_name]['process'].pid} to {self.config['bags'][bag_name]['args']['output']}")

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
                self.get_logger().info(f"Output filename: {process['output']}")
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
