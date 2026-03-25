#!/usr/bin/env python3
"""
ROS2 Node: GQ7 PPS Time Sync
=============================

Listens to the same GPIO PPS pin as the Teensy trigger node.
When the GQ7 reports time_sync=True on /gq7/mip/system/time_sync_status,
each rising PPS edge publishes the current system time to /gq7/ext/time
(sensor_msgs/msg/TimeReference), which is consumed by the microstrain
inertial driver to align GPS time.

Per edge, three timestamps are logged for comparison:
  sw_rt  — CLOCK_REALTIME sampled immediately on edge detection
  hte_rt — HTE hardware timestamp + realtime offset (what gets published)
  ptp    — PTP hardware clock edge timestamp from Teensy serial (LASTPPS cmd)
"""

import re
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import TimeReference
from microstrain_inertial_msgs.msg import MipSystemTimeSyncStatus
import gpiod
from gpiod.line import Direction, Edge, Clock
import datetime
import serial
import threading
import time
import os


class GQ7PpsTimeSyncNode(Node):

    _LASTPPS_RE = re.compile(r'LASTPPS edge=(\d+)\.(\d+)')

    def __init__(self):
        super().__init__('gq7_pps_time_sync_node')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('gpio_chip',        '/dev/gpiochip1')
        self.declare_parameter('gpio_line',        8)
        self.declare_parameter('debounce_time_us', 0)
        self.declare_parameter('frame_id',         'gq7_pps')
        self.declare_parameter('serial_port',      '/dev/teensy_trigger')
        self.declare_parameter('baud_rate',        2000000)

        p = self.get_parameter
        self.gpio_chip        = p('gpio_chip').value
        self.gpio_line        = p('gpio_line').value
        self.debounce_time_us = p('debounce_time_us').value
        self.frame_id         = p('frame_id').value
        self.serial_port      = p('serial_port').value
        self.baud_rate        = p('baud_rate').value

        # ── State ─────────────────────────────────────────────────────────────
        self._time_sync_ok = False
        self._sync_lock    = threading.Lock()
        self._gpio_request = None
        self._stop_event   = threading.Event()
        self._serial_conn       = None
        self._edge_count        = 0
        self._serial_last_fail  = 0.0   # monotonic time of last failure

        # ── Serial ────────────────────────────────────────────────────────────
        self._init_serial()

        # ── ROS interfaces ────────────────────────────────────────────────────
        self._time_pub = self.create_publisher(
            TimeReference, '/gq7/ext/time', 10)

        self._sync_sub = self.create_subscription(
            MipSystemTimeSyncStatus,
            '/gq7/mip/system/time_sync_status',
            self._sync_status_cb,
            10)

        # ── GPIO ──────────────────────────────────────────────────────────────
        if not self._init_gpio():
            self.get_logger().error('GPIO init failed — node will not publish')
            return

        self._gpio_thread = threading.Thread(
            target=self._gpio_loop, daemon=True)
        self._gpio_thread.start()

        self.get_logger().info(
            f'Ready | gpio={self.gpio_chip}:{self.gpio_line} | '
            f'serial={self.serial_port if self._serial_conn else "UNAVAILABLE"} | '
            f'waiting for time_sync on /gq7/mip/system/time_sync_status')

    # ─────────────────────────────────────────────────────────────────────────
    # Serial init
    # ─────────────────────────────────────────────────────────────────────────

    def _init_serial(self) -> None:
        try:
            self._serial_conn = serial.Serial(
                self.serial_port, self.baud_rate, timeout=0.1)
            self.get_logger().info(
                f'Serial opened: {self.serial_port} @ {self.baud_rate} baud')
        except serial.SerialException as e:
            self.get_logger().warn(
                f'Serial unavailable ({self.serial_port}): {e} — ptp column will be absent')
            self._serial_conn = None

    # ─────────────────────────────────────────────────────────────────────────
    # PTP LASTPPS query
    # ─────────────────────────────────────────────────────────────────────────

    _SERIAL_RECONNECT_COOLDOWN = 5.0  # seconds between reconnect attempts

    def _query_ptp_lastpps(self) -> int | None:
        """Send LASTPPS over serial, return edge timestamp as nanoseconds or None."""
        # Try to reconnect if previously failed and cooldown elapsed
        if self._serial_conn is None:
            now = time.monotonic()
            if now - self._serial_last_fail >= self._SERIAL_RECONNECT_COOLDOWN:
                self._init_serial()
            if self._serial_conn is None:
                return None
        try:
            self._serial_conn.reset_input_buffer()
            self._serial_conn.write(b'LASTPPS\n')
            self._serial_conn.flush()
            line = self._serial_conn.readline().decode('ascii', errors='replace').strip()
            m = self._LASTPPS_RE.search(line)
            if not m:
                self.get_logger().warn(f'LASTPPS parse failed, got: {line!r}')
                return None
            return int(m.group(1)) * 1_000_000_000 + int(m.group(2))
        except serial.SerialException as e:
            self.get_logger().warn(f'LASTPPS serial error: {e} — will retry in {self._SERIAL_RECONNECT_COOLDOWN:.0f}s')
            try:
                self._serial_conn.close()
            except Exception:
                pass
            self._serial_conn = None
            self._serial_last_fail = time.monotonic()
            return None
        except Exception as e:
            self.get_logger().warn(f'LASTPPS unexpected error: {e}')
            return None

    # ─────────────────────────────────────────────────────────────────────────
    # Subscription callback
    # ─────────────────────────────────────────────────────────────────────────

    def _sync_status_cb(self, msg: MipSystemTimeSyncStatus):
        with self._sync_lock:
            prev = self._time_sync_ok
            self._time_sync_ok = msg.time_sync
        if msg.time_sync != prev:
            self.get_logger().info(
                f'GQ7 time_sync changed → {msg.time_sync}')

    # ─────────────────────────────────────────────────────────────────────────
    # GPIO init
    # ─────────────────────────────────────────────────────────────────────────

    def _init_gpio(self) -> bool:
        try:
            settings = gpiod.LineSettings()
            settings.direction       = Direction.INPUT
            settings.edge_detection  = Edge.RISING
            settings.debounce_period = datetime.timedelta(
                microseconds=self.debounce_time_us)

            for clock_mode, label in [(Clock.HTE, 'HTE'), (Clock.MONOTONIC, 'MONOTONIC')]:
                try:
                    settings.event_clock = clock_mode
                    self._gpio_request = gpiod.request_lines(
                        path=self.gpio_chip,
                        consumer='gq7-pps-sync',
                        config={self.gpio_line: settings})
                    self.get_logger().info(f'GPIO clock mode: {label}')
                    return True
                except (OSError, AttributeError) as e:
                    self.get_logger().warn(f'{label} unavailable: {e}')

            self.get_logger().error('All GPIO clock modes failed')
            return False
        except Exception as e:
            self.get_logger().error(f'GPIO init error: {e}')
            return False

    # ─────────────────────────────────────────────────────────────────────────
    # GPIO loop
    # ─────────────────────────────────────────────────────────────────────────

    def _gpio_loop(self):
        try:
            param = os.sched_param(os.sched_get_priority_max(os.SCHED_FIFO))
            os.sched_setscheduler(0, os.SCHED_FIFO, param)
        except PermissionError:
            self.get_logger().warn('GPIO thread: realtime priority unavailable')

        self.get_logger().info('GPIO monitor thread started')

        while not self._stop_event.is_set():
            req = self._gpio_request
            if req is None:
                break

            if not req.wait_edge_events(timeout=datetime.timedelta(milliseconds=200)):
                continue

            req = self._gpio_request
            if req is None:
                break

            try:
                events = req.read_edge_events()
            except ValueError:
                continue

            with self._sync_lock:
                synced = self._time_sync_ok

            if not synced:
                continue

            # Query PTP timestamp from Teensy serial
            ptp_ns = self._query_ptp_lastpps()

            self._edge_count += 1

            if ptp_ns is None:
                self.get_logger().warn(f'PPS edge #{self._edge_count} | ptp=unavailable')
                continue

            ptp_sec, ptp_nsec = divmod(ptp_ns, 1_000_000_000)
            self.get_logger().info(
                f'PPS edge #{self._edge_count} | ptp={ptp_sec}.{ptp_nsec:09d}s')

            ros_time = Time(nanoseconds=ptp_ns)
            msg = TimeReference()
            msg.header.stamp    = ros_time.to_msg()
            msg.header.frame_id = self.frame_id
            msg.time_ref        = ros_time.to_msg()
            msg.source          = 'gq7_pps'
            self._time_pub.publish(msg)

        self.get_logger().info('GPIO monitor thread exiting')

    # ─────────────────────────────────────────────────────────────────────────
    # Cleanup
    # ─────────────────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._stop_event.set()
        req = self._gpio_request
        self._gpio_request = None
        if req is not None:
            try:
                req.release()
            except Exception:
                pass
        if self._serial_conn is not None:
            try:
                self._serial_conn.close()
            except Exception:
                pass
            self._serial_conn = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GQ7PpsTimeSyncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
