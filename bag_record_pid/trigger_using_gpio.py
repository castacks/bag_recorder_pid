#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import Jetson.GPIO as GPIO

class ButtonStatePublisher(Node):
    def __init__(self):
        super().__init__('button_state_publisher')

        # ----- Parameters -----
        self.declare_parameter('pin', 37)                    # BOARD numbering
        self.declare_parameter('pull', 'down')               # 'down' or 'up'
        self.declare_parameter('active_low', False)          # invert logic if wired to GND
        self.declare_parameter('publish_rate_hz', 1.0)      # state publish rate
        self.declare_parameter("topic", "button_state")

        self.pin = int(self.get_parameter('pin').get_parameter_value().integer_value)
        pull_str = self.get_parameter('pull').get_parameter_value().string_value.lower()
        self.active_low = bool(self.get_parameter('active_low').get_parameter_value().bool_value)
        rate_hz = float(self.get_parameter('publish_rate_hz').get_parameter_value().double_value)
        rate_hz = max(1.0, rate_hz)  # clamp to sane minimum

        # ----- Publisher -----
        topic = self.get_parameter("topic").get_parameter_value().string_value
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

        # Initial state (for logging; we still publish every tick)
        raw = GPIO.input(self.pin)
        self.last_state = self._logical_state(raw)

        # ----- Timer for state-based publishing -----
        period = 1.0 / rate_hz
        self.timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"Publishing button state at {rate_hz:.1f} Hz from BOARD pin {self.pin} "
            f"(pull-{pull_str}, {'active-low' if self.active_low else 'active-high'}) "
            f"on topic '{topic}'."
        )

    def _logical_state(self, raw_level: int) -> bool:
        """Map raw GPIO level to logical pressed/not-pressed."""
        return (not raw_level) if self.active_low else bool(raw_level)

    def _tick(self):
        # Read raw GPIO each cycle and publish the logical state
        raw = GPIO.input(self.pin)
        state = self._logical_state(raw)

        msg = Bool()
        msg.data = state
        self.pub.publish(msg)

        # Log only on change to keep logs readable
        if state != self.last_state:
            self.last_state = state
            self.get_logger().info(f'Button state → {state}')

    def destroy_node(self):
        try:
            # No event detection used, nothing to remove
            pass
        finally:
            GPIO.cleanup()
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ButtonStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
