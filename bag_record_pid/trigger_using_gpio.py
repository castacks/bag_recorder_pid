#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import Jetson.GPIO as GPIO

class ButtonEventPublisher(Node):
    def __init__(self):
        super().__init__('button_event_publisher')

        # ----- Parameters -----
        self.declare_parameter('pin', 37)                   # BOARD numbering
        self.declare_parameter('pull', 'down')              # 'down' or 'up'
        self.declare_parameter('debounce_ms', 200)          # debounce in ms
        self.declare_parameter('topic', 'button_state')     # topic name
        self.declare_parameter('active_low', False)         # invert logic if wired to GND

        self.pin = int(self.get_parameter('pin').get_parameter_value().integer_value)
        pull_str = self.get_parameter('pull').get_parameter_value().string_value.lower()
        self.debounce_ms = int(self.get_parameter('debounce_ms').get_parameter_value().integer_value)
        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.active_low = bool(self.get_parameter('active_low').get_parameter_value().bool_value)

        # ----- Publisher -----
        self.pub = self.create_publisher(Bool, topic, 10)

        # ----- GPIO Setup -----
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)

        if pull_str == 'up':
            pull_cfg = GPIO.PUD_UP
        elif pull_str == 'down':
            pull_cfg = GPIO.PUD_DOWN
        else:
            raise ValueError("Parameter 'pull' must be 'up' or 'down'.")

        GPIO.setup(self.pin, GPIO.IN, pull_up_down=pull_cfg)

        # Initialize last_state and publish initial state (optional)
        raw = GPIO.input(self.pin)
        self.last_state = self._logical_state(raw)
        self._publish(self.last_state, log=True)

        # Register interrupt on both edges
        GPIO.add_event_detect(
            self.pin,
            GPIO.BOTH,
            callback=self._edge_callback,
            bouncetime=self.debounce_ms
        )

        self.get_logger().info(
            f"Listening on BOARD pin {self.pin} with pull-{pull_str}, "
            f"{'active-low' if self.active_low else 'active-high'}, "
            f"debounce={self.debounce_ms} ms. Publishing on '{topic}'."
        )

    def _logical_state(self, raw_level: int) -> bool:
        """Map raw GPIO level to logical pressed/not-pressed."""
        return (not raw_level) if self.active_low else bool(raw_level)

    def _edge_callback(self, channel):
        # Read current raw level and convert to logical button state
        raw = GPIO.input(self.pin)
        state = self._logical_state(raw)

        # Only publish on change (extra guard in case both-edge fires twice)
        if state != self.last_state:
            self.last_state = state
            self._publish(state, log=True)

    def _publish(self, state: bool, log: bool = False):
        msg = Bool()
        msg.data = state
        # Publishing from a GPIO callback thread is fine in rclpy for simple msgs
        self.pub.publish(msg)
        if log:
            self.get_logger().info(f'Button state changed → {state}')

    def destroy_node(self):
        try:
            GPIO.remove_event_detect(self.pin)
        except Exception:
            pass
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ButtonEventPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
