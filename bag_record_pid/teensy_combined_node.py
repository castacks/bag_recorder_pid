#!/usr/bin/env python3
"""
ROS2 Node: Teensy Combined — Wave Trigger + GQ7 PPS Time Sync

GPIO pins:
  trigger_gpio_chip/line  — 30 Hz wave trigger edges (always publishing from bootup)
  pps_gpio_chip/line      — 1 Hz GPS PPS edges

State machine:
  IDLE           — GPIO running, timestamps publishing, no active session
  SESSION_ACTIVE — SYNC_START sent; returns to IDLE on stop_trigger

Services:
  /teensy/start_trigger  — send SYNC_START, transition IDLE → SESSION_ACTIVE
  /teensy/stop_trigger   — send SYNC_STOP,  transition SESSION_ACTIVE → IDLE

Topics published:
  /teensy/timestamps  (std_msgs/Header)     — every rising trigger edge
  /gq7/ext/time       (sensor_msgs/TimeRef) — GPS PPS timestamp (when GQ7 synced)

Topics subscribed:
  /gq7/mip/system/time_sync_status — gates PPS publishing on GQ7 sync status
"""

import re
import sys
import os
import time
import threading
import datetime
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.time import Time
import serial
import gpiod
from gpiod.line import Direction, Edge, Clock

from std_msgs.msg import Header
from sensor_msgs.msg import TimeReference
from std_srvs.srv import Trigger
from microstrain_inertial_msgs.msg import MipSystemTimeSyncStatus

MARKER_LEN = 13  # bits in SYNC_START / SYNC_STOP pattern


class NodeState(Enum):
    IDLE           = 'IDLE'
    SESSION_ACTIVE = 'SESSION_ACTIVE'


class TeensyCombinedNode(Node):

    _LASTPPS_RE = re.compile(r"(\d+)\.(\d+)")

    def __init__(self):
        super().__init__('teensy_combined_node')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('serial_port',          '/dev/teensy_trigger')
        self.declare_parameter('baud_rate',            921600)
        self.declare_parameter('trigger_gpio_chip',    '/dev/gpiochip1')
        self.declare_parameter('trigger_gpio_line',    8)
        self.declare_parameter('trigger_frame_id',     'teensy_trigger')
        self.declare_parameter('debounce_time_us',     0)
        self.declare_parameter('pps_gpio_chip',        '/dev/gpiochip0')
        self.declare_parameter('pps_gpio_line',        96)
        self.declare_parameter('pps_frame_id',         'gq7_pps')
        self.declare_parameter('pps_debounce_time_us', 0)
        self.declare_parameter('start_drain_ms',       50.0)

        p = self.get_parameter
        self.serial_port          = p('serial_port').value
        self.baud_rate            = p('baud_rate').value
        self.trigger_gpio_chip    = p('trigger_gpio_chip').value
        self.trigger_gpio_line    = p('trigger_gpio_line').value
        self.trigger_frame_id     = p('trigger_frame_id').value
        self.debounce_time_us     = p('debounce_time_us').value
        self.pps_gpio_chip        = p('pps_gpio_chip').value
        self.pps_gpio_line        = p('pps_gpio_line').value
        self.pps_frame_id         = p('pps_frame_id').value
        self.pps_debounce_time_us = p('pps_debounce_time_us').value
        self.start_drain_ms       = p('start_drain_ms').value

        # ── Serial ────────────────────────────────────────────────────────────
        self._serial_lock = threading.Lock()
        self.serial_conn  = None
        self._init_serial()

        # ── GQ7 PPS state ─────────────────────────────────────────────────────
        self._time_sync_ok   = False
        self._sync_lock      = threading.Lock()
        self._pps_edge_count = 0

        # ── State machine + session timing ────────────────────────────────────
        self._state              = NodeState.IDLE
        self.trigger_gpio_req    = None
        self.gpio_thread         = None
        self.gpio_ready_event    = threading.Event()
        self.time_offset         = None   # HTE → CLOCK_REALTIME offset (ns)
        self._start_cmd_mono_ns  = None
        self._stop_cmd_mono_ns   = None
        self._first_edge_mono_ns = None
        self._last_edge_mono_ns  = None

        # ── ROS interfaces ────────────────────────────────────────────────────
        self.timestamp_pub = self.create_publisher(Header, '/teensy/timestamps', 10)
        self.start_srv = self.create_service(
            Trigger, '/teensy/start_trigger', self.start_trigger_callback)
        self.stop_srv = self.create_service(
            Trigger, '/teensy/stop_trigger',  self.stop_trigger_callback)

        self._time_pub = self.create_publisher(TimeReference, '/gq7/ext/time', 10)
        self._sync_sub = self.create_subscription(
            MipSystemTimeSyncStatus,
            '/gq7/mip/system/time_sync_status',
            self._sync_status_cb,
            10)

        # ── PPS GPIO (always-on) ──────────────────────────────────────────────
        self._pps_gpio_req = None
        self._pps_stop_evt = threading.Event()
        if self._init_pps_gpio():
            self._pps_thread = threading.Thread(
                target=self._pps_gpio_loop, daemon=True)
            self._pps_thread.start()
        else:
            self.get_logger().error('PPS GPIO init failed — /gq7/ext/time will not publish')
            self._pps_thread = None

        # ── Trigger GPIO (always-on) ──────────────────────────────────────────
        self.gpio_ready_event.clear()
        if self._init_trigger_gpio():
            self.gpio_thread = threading.Thread(
                target=self._gpio_monitor_thread, daemon=True)
            self.gpio_thread.start()
            if not self.gpio_ready_event.wait(timeout=0.5):
                self.get_logger().warn('Trigger GPIO thread not ready at bootup')
        else:
            self.get_logger().error('Trigger GPIO init failed — timestamps will not publish')
            self.gpio_thread = None

        self.get_logger().info(
            f'Ready [{self._state.value}] | serial={self.serial_port} | '
            f'trigger={self.trigger_gpio_chip}:{self.trigger_gpio_line} | '
            f'pps={self.pps_gpio_chip}:{self.pps_gpio_line}')

    # ─────────────────────────────────────────────────────────────────────────
    # Serial
    # ─────────────────────────────────────────────────────────────────────────

    def _init_serial(self) -> None:
        try:
            self.serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            time.sleep(2)
            with self._serial_lock:
                while self.serial_conn.in_waiting:
                    self.get_logger().info(
                        f'Teensy: {self.serial_conn.readline().decode().strip()}')
            self.get_logger().info('Serial ready')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial open failed: {e}')
            sys.exit(1)

    def _send_command(self, cmd: str) -> str | None:
        with self._serial_lock:
            if self.serial_conn is None:
                return None
            try:
                self.serial_conn.write(f'{cmd}\n'.encode())
                self.serial_conn.flush()
                time.sleep(0.2)
                lines = []
                while self.serial_conn.in_waiting:
                    line = self.serial_conn.readline().decode().strip()
                    if line:
                        lines.append(line)
                if lines:
                    for line in lines:
                        self.get_logger().info(f'Teensy: {line}')
                    return lines[-1]
                self.get_logger().warn(f'No response for {cmd}')
                return None
            except Exception as e:
                self.get_logger().error(f'Serial error sending {cmd}: {e}')
                return None

    def _query_ptp_lastpps(self) -> int | None:
        with self._serial_lock:
            if self.serial_conn is None:
                return None
            try:
                self.serial_conn.reset_input_buffer()
                self.serial_conn.write(b'LASTPPS\n')
                self.serial_conn.flush()
                line = self.serial_conn.readline().decode('ascii', errors='replace').strip()
                m = self._LASTPPS_RE.search(line)
                if not m:
                    self.get_logger().warn(f'LASTPPS parse failed: {line!r}')
                    return None
                return int(m.group(1)) * 1_000_000_000 + int(m.group(2))
            except Exception as e:
                self.get_logger().warn(f'LASTPPS error: {e}')
                return None

    # ─────────────────────────────────────────────────────────────────────────
    # GQ7 sync status
    # ─────────────────────────────────────────────────────────────────────────

    def _sync_status_cb(self, msg: MipSystemTimeSyncStatus):
        with self._sync_lock:
            prev = self._time_sync_ok
            self._time_sync_ok = msg.time_sync
        if msg.time_sync != prev:
            self.get_logger().info(f'GQ7 time_sync changed → {msg.time_sync}')
            if prev and not msg.time_sync:
                self.get_logger().error('GQ7 lost trigger — check trigger cables')

    # ─────────────────────────────────────────────────────────────────────────
    # PPS GPIO (always-on)
    # ─────────────────────────────────────────────────────────────────────────

    def _init_pps_gpio(self) -> bool:
        try:
            settings = gpiod.LineSettings()
            settings.direction       = Direction.INPUT
            settings.edge_detection  = Edge.RISING
            settings.debounce_period = datetime.timedelta(
                microseconds=self.pps_debounce_time_us)
            for clock_mode, label in [(Clock.HTE, 'HTE'), (Clock.MONOTONIC, 'MONOTONIC')]:
                try:
                    settings.event_clock = clock_mode
                    self._pps_gpio_req = gpiod.request_lines(
                        path=self.pps_gpio_chip,
                        consumer='teensy-combined-pps',
                        config={self.pps_gpio_line: settings})
                    self.get_logger().info(f'PPS GPIO clock: {label}')
                    return True
                except (OSError, AttributeError) as e:
                    self.get_logger().warn(f'PPS {label} unavailable: {e}')
            self.get_logger().error('PPS GPIO: all clock modes failed')
            return False
        except Exception as e:
            self.get_logger().error(f'PPS GPIO init error: {e}')
            return False

    def _pps_gpio_loop(self):
        try:
            param = os.sched_param(os.sched_get_priority_max(os.SCHED_FIFO))
            os.sched_setscheduler(0, os.SCHED_FIFO, param)
        except PermissionError:
            self.get_logger().warn('PPS GPIO thread: realtime priority unavailable')

        self.get_logger().info('PPS GPIO thread started')
        while not self._pps_stop_evt.is_set():
            req = self._pps_gpio_req
            if req is None:
                break
            if not req.wait_edge_events(timeout=datetime.timedelta(milliseconds=200)):
                continue
            req = self._pps_gpio_req
            if req is None:
                break
            try:
                req.read_edge_events()
            except ValueError:
                continue

            with self._sync_lock:
                synced = self._time_sync_ok
            if not synced:
                continue

            ptp_ns = self._query_ptp_lastpps()
            self._pps_edge_count += 1
            if ptp_ns is None:
                self.get_logger().warn(
                    f'PPS edge #{self._pps_edge_count} | ptp=unavailable')
                continue

            ros_time = Time(nanoseconds=ptp_ns)
            msg = TimeReference()
            msg.header.stamp    = ros_time.to_msg()
            msg.header.frame_id = self.pps_frame_id
            msg.time_ref        = ros_time.to_msg()
            msg.source          = 'gq7_pps'
            if self._time_pub.get_subscription_count() > 0:
                self._time_pub.publish(msg)

        self.get_logger().info('PPS GPIO thread exiting')

    # ─────────────────────────────────────────────────────────────────────────
    # Trigger GPIO (always-on)
    # ─────────────────────────────────────────────────────────────────────────

    def _init_trigger_gpio(self) -> bool:
        try:
            settings = gpiod.LineSettings()
            settings.direction       = Direction.INPUT
            settings.edge_detection  = Edge.RISING
            settings.debounce_period = datetime.timedelta(
                microseconds=self.debounce_time_us)
            for clock_mode, label in [(Clock.HTE, 'HTE'), (Clock.MONOTONIC, 'MONOTONIC')]:
                try:
                    settings.event_clock = clock_mode
                    self.trigger_gpio_req = gpiod.request_lines(
                        path=self.trigger_gpio_chip,
                        consumer='teensy-combined-trigger',
                        config={self.trigger_gpio_line: settings})
                    self.get_logger().info(f'Trigger GPIO clock: {label}')
                    return True
                except (OSError, AttributeError) as e:
                    self.get_logger().warn(f'Trigger {label} unavailable: {e}')
            self.get_logger().error('Trigger GPIO: all clock modes failed')
            return False
        except Exception as e:
            self.get_logger().error(f'Trigger GPIO init error: {e}')
            return False

    def _gpio_monitor_thread(self):
        try:
            param = os.sched_param(os.sched_get_priority_max(os.SCHED_FIFO))
            os.sched_setscheduler(0, os.SCHED_FIFO, param)
        except PermissionError:
            self.get_logger().warn('Trigger GPIO thread: realtime priority unavailable')

        self.get_logger().info('Trigger GPIO thread started')
        self.gpio_ready_event.set()

        timeout_count = 0

        try:
            while True:
                req = self.trigger_gpio_req
                if req is None:
                    break
                if not req.wait_edge_events(timeout=datetime.timedelta(milliseconds=100)):
                    timeout_count += 1
                    if timeout_count == 50:  # ~5 s silence
                        self.get_logger().warn('No GPIO triggers for 5 s — check wiring')
                    continue

                req = self.trigger_gpio_req
                if req is None:
                    break
                try:
                    events = req.read_edge_events()
                except ValueError:
                    continue
                timeout_count = 0

                for event in events:
                    hte_ts_ns = event.timestamp_ns

                    if self.time_offset is None:
                        self.time_offset = (
                            time.clock_gettime_ns(time.CLOCK_REALTIME) - hte_ts_ns)
                        self.get_logger().info('Trigger GPIO: time offset established')

                    ros_time = Time(nanoseconds=hte_ts_ns + self.time_offset)
                    hdr = Header()
                    hdr.stamp    = ros_time.to_msg()
                    hdr.frame_id = self.trigger_frame_id
                    if self.timestamp_pub.get_subscription_count() > 0:
                        self.timestamp_pub.publish(hdr)

                    mono_ns = time.monotonic_ns()
                    if self._first_edge_mono_ns is None:
                        self._first_edge_mono_ns = mono_ns
                    self._last_edge_mono_ns = mono_ns

        except Exception as e:
            self.get_logger().error(f'Trigger GPIO thread error: {e}')
        finally:
            self.get_logger().info('Trigger GPIO thread stopped')

    def _drain_pending_trigger_events(self, max_wait_ms: float):
        if not self.trigger_gpio_req:
            return
        end_t = time.time() + max_wait_ms / 1000.0
        while time.time() < end_t:
            if self.trigger_gpio_req.wait_edge_events(
                    timeout=datetime.timedelta(milliseconds=1)):
                try:
                    self.trigger_gpio_req.read_edge_events()
                except ValueError:
                    pass
            else:
                break

    # ─────────────────────────────────────────────────────────────────────────
    # Session helpers
    # ─────────────────────────────────────────────────────────────────────────

    def _reset_session_state(self):
        self.time_offset          = None
        self._start_cmd_mono_ns   = None
        self._stop_cmd_mono_ns    = None
        self._first_edge_mono_ns  = None
        self._last_edge_mono_ns   = None

    def _log_edge_timing(self):
        if self._start_cmd_mono_ns and self._first_edge_mono_ns:
            self.get_logger().info(
                f'start→first_edge: '
                f'{(self._first_edge_mono_ns - self._start_cmd_mono_ns) / 1e6:.2f} ms')
        if self._stop_cmd_mono_ns and self._last_edge_mono_ns:
            self.get_logger().info(
                f'last_edge vs stop_cmd: '
                f'{(self._last_edge_mono_ns - self._stop_cmd_mono_ns) / 1e6:.2f} ms')

    # ─────────────────────────────────────────────────────────────────────────
    # Service callbacks
    # ─────────────────────────────────────────────────────────────────────────

    def start_trigger_callback(self, request, response):
        if self._state == NodeState.SESSION_ACTIVE:
            response.success = False
            response.message = 'Already running'
            return response

        if self.trigger_gpio_req is None:
            response.success = False
            response.message = 'Trigger GPIO not initialized'
            return response

        self._reset_session_state()
        self._drain_pending_trigger_events(self.start_drain_ms)

        self._start_cmd_mono_ns = time.monotonic_ns()
        time.sleep(0.5)  # let recording initialize before marker fires
        resp = self._send_command('SYNC_START')

        if not resp or 'OK' not in resp:
            response.success = False
            response.message = f'SYNC_START failed: {resp}'
            return response

        self._state = NodeState.SESSION_ACTIVE
        self.get_logger().info(f'✓ Session started [{self._state.value}]')
        response.success = True
        response.message = 'SYNC_START sent'
        return response

    def stop_trigger_callback(self, request, response):
        if self._state != NodeState.SESSION_ACTIVE:
            response.success = False
            response.message = 'Not running'
            return response

        self._stop_cmd_mono_ns = time.monotonic_ns()
        self._send_command('SYNC_STOP')
        time.sleep(MARKER_LEN / 30.0 + 0.1)  # wait for marker to complete

        self._log_edge_timing()
        self._state = NodeState.IDLE
        self._reset_session_state()
        self.get_logger().info(f'✓ Session stopped [{self._state.value}]')
        response.success = True
        response.message = 'Stopped'
        return response

    # ─────────────────────────────────────────────────────────────────────────
    # Cleanup
    # ─────────────────────────────────────────────────────────────────────────

    def cleanup(self):
        self._pps_stop_evt.set()
        if self._pps_thread and self._pps_thread.is_alive():
            self._pps_thread.join(timeout=1.0)
        if self._pps_gpio_req:
            try:
                self._pps_gpio_req.release()
            except Exception:
                pass
            self._pps_gpio_req = None

        trigger_req = self.trigger_gpio_req
        self.trigger_gpio_req = None  # signals monitor thread to exit
        if self.gpio_thread and self.gpio_thread.is_alive():
            self.gpio_thread.join(timeout=2.0)
            self.gpio_thread = None
        if trigger_req:
            try:
                trigger_req.release()
            except Exception:
                pass

        if self.serial_conn and self.serial_conn.is_open:
            with self._serial_lock:
                self.serial_conn.close()
            self.serial_conn = None

        self.get_logger().info('Cleanup complete')


def main(args=None):
    rclpy.init(args=args)
    node = TeensyCombinedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
