import numpy as np
import rclpy
from rclpy.node import Node
from collections import defaultdict
from importlib import import_module
import threading
import yaml
import time
import os
from rich.console import Console
from rich.table import Table
from rich.live import Live
from ament_index_python import get_package_share_directory

def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)

class TopicHealthMonitor(Node):
    def __init__(self):
        super().__init__('topic_health_monitor')

        self.node_name = self.get_name() # Get the full node name, including namespace if any

        self.declare_parameter("cfg_path", "")
        
        self.cfg_path = self.get_parameter("cfg_path").get_parameter_value().string_value

        if self.cfg_path == '':
            self.cfg_path = get_share_file('bag_record_pid', 'config/topic_rates.yaml')

        self.cfg = yaml.safe_load(open(self.cfg_path))
        topics_expected_rates = self.cfg['topic_rates']
        self.topics_expected_rates = topics_expected_rates
        self.topic_names = [t["topic"] for t in topics_expected_rates]
        self.msg_counts = np.zeros(len(topics_expected_rates), dtype=np.int32)
        self.current_rates = np.zeros(len(topics_expected_rates), dtype=np.float32)
        self.last_stamps = np.full(len(topics_expected_rates), np.nan, dtype=np.float64)

        self.time_tolerance = self.cfg['tolerance']

        for idx, tinfo in enumerate(topics_expected_rates):
            msg_type = self._import_type(tinfo["type"])
            self.create_subscription(
                msg_type,
                tinfo["topic"],
                lambda msg, i=idx: self.msg_callback(msg, i),
                10
            )

        self.twindow = self.cfg['time_window']
        self.timer = self.create_timer(self.twindow, self.tick)
        self.lock = threading.Lock()
    
    def _import_type(self, type_str):
        pkg, _, name = type_str.rpartition(".")
        module = import_module(pkg)
        return getattr(module, name)

    def msg_callback(self, msg, idx):
        with self.lock:
            self.msg_counts[idx] += 1
            # Works if message has header.stamp
            try:
                stamp = msg.header.stamp
                self.last_stamps[idx] = stamp.sec + stamp.nanosec * 1e-9
            except AttributeError:
                self.last_stamps[idx] = np.nan

    def tick(self):
        with self.lock:
            self.current_rates[:] = self.msg_counts/self.twindow
            self.msg_counts[:] = 0

    def get_rates(self):
        with self.lock:
            return self.current_rates.copy()

    def get_out_of_sync(self):
        with self.lock:
            now = self.get_clock().now().nanoseconds * 1e-9
            diffs = np.abs(now - self.last_stamps)
            mask = diffs > self.time_tolerance
            return mask, diffs


def make_table(monitor: TopicHealthMonitor):
    rates = monitor.get_rates()
    mask, diffs = monitor.get_out_of_sync()

    table = Table(title="📡 Topic Health Monitor", expand=True)
    table.add_column("Topic", style="bold cyan")
    # table.add_column("Expected [Hz]", justify="right")
    table.add_column("Actual [Hz]", justify="right")
    table.add_column("Time Diff [s]", justify="right")

    for i, tinfo in enumerate(monitor.topics_expected_rates):
        expected = tinfo["expected_rate"]
        actual = rates[i]
        diff = diffs[i]
        tol = expected*.05
        rate_color = "green" if actual+tol >= expected else "red"
        time_color = "green" if not mask[i] else "red"
        table.add_row(
            monitor.topic_names[i],
            # f"{expected:.1f}",
            f"[{rate_color}]{actual:.1f}[/{rate_color}]",
            f"[{time_color}]{diff:.3f}[/{time_color}]"
        )
    return table


def main(args=None):
    rclpy.init(args=args)

    monitor = TopicHealthMonitor()

    ros_thread = threading.Thread(target=rclpy.spin, args=(monitor,), daemon=True)
    ros_thread.start()

    console = Console()
    with Live(make_table(monitor), console=console, refresh_per_second=2) as live:
        try:
            while rclpy.ok():
                time.sleep(0.5)
                live.update(make_table(monitor))
        except KeyboardInterrupt:
            pass

    monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
