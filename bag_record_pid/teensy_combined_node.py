#!/usr/bin/env python3
"""
ROS2 Node: Teensy Combined — Wave Trigger + GQ7 PPS Time Sync
=============================================================

Merges teensy_wave_trigger_node and gq7_pps_time_sync_node into a single
process that opens the Teensy serial device **once** and shares it between
both functions, protected by a threading.Lock().

GPIO pins are independent:
  trigger_gpio_chip/line  (/dev/gpiochip1 : 8)  — 30 Hz wave trigger edges
  pps_gpio_chip/line      (/dev/gpiochip0 : 96) — 1 Hz GPS PPS edges

Services:
  /teensy/start_trigger  — SYNC_START marker + continuous 30 Hz
  /teensy/stop_trigger   — SYNC_STOP marker + STOP

Topics published:
  /teensy/timestamps          (std_msgs/Header)      — every rising trigger edge
  /teensy/pulse_interval_ns   (std_msgs/Int64)       — interval between edges
  /teensy/diagnostics         (diagnostic_msgs/...)  — session diagnostics
  /gq7/ext/time               (sensor_msgs/TimeRef)  — GPS PPS timestamp

Topics subscribed:
  /gq7/mip/system/time_sync_status — enables PPS publishing when synced
"""

import re
import sys
import os
import time
import threading
import datetime
import statistics
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import serial
import gpiod
from gpiod.line import Direction, Edge, Clock

from std_msgs.msg import Header, Int64
from sensor_msgs.msg import TimeReference
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_srvs.srv import Trigger
from microstrain_inertial_msgs.msg import MipSystemTimeSyncStatus

# ─────────────────────────────────────────────────────────────────────────────
# Sync pattern constants (trigger wave)
# ─────────────────────────────────────────────────────────────────────────────
MARKER_BITS     = [1, 0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1]
MARKER_LEN      = len(MARKER_BITS)
FRAME_PERIOD_NS = 1_000_000_000 // 30   # 33_333_333 ns


def _slots_from_gap(gap_ns: int) -> int:
    return max(1, min(round(gap_ns / FRAME_PERIOD_NS), 2))


# ─────────────────────────────────────────────────────────────────────────────
# Pattern detector (unchanged from teensy_wave_trigger_node)
# ─────────────────────────────────────────────────────────────────────────────

class SyncPatternDetector:
    def __init__(self):
        self.reset()

    @property
    def diag_start_ts_ns(self):
        return self._diag_start_ts_ns

    @property
    def diag_stop_ts_ns(self):
        return self._diag_stop_ts_ns

    @property
    def diag_pulses(self):
        return list(self._diag_pulse_ts)

    def reset(self):
        self._bit_buf           = []
        self._pulse_ts_buf      = []
        self._prev_ts_ns        = None
        self._state             = 'idle'
        self._diag_start_ts_ns  = None
        self._diag_stop_ts_ns   = None
        self._diag_pulse_ts     = []
        self._last_pulse_ts     = None
        self._pre_stop_pulse_ts = None

    def on_pulse(self, timestamp_ns: int):
        self._last_pulse_ts = timestamp_ns

        if self._prev_ts_ns is None:
            self._emit_bit(1, timestamp_ns)
            self._prev_ts_ns = timestamp_ns
            if self._state == 'in_session':
                self._record_diag_pulse(timestamp_ns)
            return None

        gap_ns           = timestamp_ns - self._prev_ts_ns
        self._prev_ts_ns = timestamp_ns
        slots            = _slots_from_gap(gap_ns)

        for _ in range(slots - 1):
            self._emit_bit(0, None)
        self._emit_bit(1, timestamp_ns)

        result = self._check_pattern()

        if result == 'start':
            return 'start'

        if result == 'stop':
            self._diag_stop_ts_ns = self._pre_stop_pulse_ts
            return 'stop'

        if self._state == 'in_session':
            self._record_diag_pulse(timestamp_ns)

        return None

    def _record_diag_pulse(self, ts_ns):
        if self._diag_start_ts_ns is None:
            self._diag_start_ts_ns = ts_ns
        self._diag_pulse_ts.append(ts_ns)

    def _emit_bit(self, bit: int, ts_ns):
        self._bit_buf.append(bit)
        self._pulse_ts_buf.append(ts_ns if bit == 1 else None)
        if len(self._bit_buf) > MARKER_LEN * 2:
            self._bit_buf      = self._bit_buf[-MARKER_LEN * 2:]
            self._pulse_ts_buf = self._pulse_ts_buf[-MARKER_LEN * 2:]

    def _check_pattern(self):
        if len(self._bit_buf) < MARKER_LEN:
            return None
        window    = self._bit_buf[-MARKER_LEN:]
        window_ts = self._pulse_ts_buf[-MARKER_LEN:]
        if window != MARKER_BITS:
            return None

        if self._state == 'idle':
            self._state        = 'in_session'
            self._bit_buf      = []
            self._pulse_ts_buf = []
            return 'start'

        elif self._state == 'in_session':
            marker_pulse_ts = set(ts for ts in window_ts if ts is not None)
            while self._diag_pulse_ts and self._diag_pulse_ts[-1] in marker_pulse_ts:
                self._diag_pulse_ts.pop()
            self._pre_stop_pulse_ts = (
                self._diag_pulse_ts[-1] if self._diag_pulse_ts else None)
            self._state        = 'idle'
            self._bit_buf      = []
            self._pulse_ts_buf = []
            return 'stop'

        return None


# ─────────────────────────────────────────────────────────────────────────────
# Merged ROS2 Node
# ─────────────────────────────────────────────────────────────────────────────

class TeensyCombinedNode(Node):

    _LASTPPS_RE = re.compile(r"(\d+)\.(\d+)")
    _SERIAL_RECONNECT_COOLDOWN = 5.0  # seconds between reconnect attempts

    def __init__(self):
        super().__init__('teensy_combined_node')

        # ── Parameters ───────────────────────────────────────────────────────
        # Shared serial
        self.declare_parameter('serial_port', '/dev/teensy_trigger')
        self.declare_parameter('baud_rate',   921600)

        # Trigger GPIO (30 Hz wave)
        self.declare_parameter('trigger_gpio_chip',     '/dev/gpiochip1')
        self.declare_parameter('trigger_gpio_line',     8)
        self.declare_parameter('trigger_frame_id',      'teensy_trigger')
        self.declare_parameter('debounce_time_us',      0)

        # PPS GPIO (1 Hz GPS)
        self.declare_parameter('pps_gpio_chip',         '/dev/gpiochip0')
        self.declare_parameter('pps_gpio_line',         96)
        self.declare_parameter('pps_frame_id',          'gq7_pps')
        self.declare_parameter('pps_debounce_time_us',  0)

        # Trigger diagnostics / session
        self.declare_parameter('freq_window_size',              10)
        self.declare_parameter('running_freq_window_size',      100)
        self.declare_parameter('debug_mode',                    False)
        self.declare_parameter('debug_duration_s',              0.2)
        self.declare_parameter('enable_continuous_diagnostics', False)
        self.declare_parameter('diagnostic_period',             1.0)
        self.declare_parameter('stop_capture_grace_ms',         50.0)
        self.declare_parameter('stop_empty_polls_before_exit',  3)
        self.declare_parameter('start_drain_ms',                50.0)

        p = self.get_parameter
        self.serial_port                   = p('serial_port').value
        self.baud_rate                     = p('baud_rate').value
        self.trigger_gpio_chip             = p('trigger_gpio_chip').value
        self.trigger_gpio_line             = p('trigger_gpio_line').value
        self.trigger_frame_id              = p('trigger_frame_id').value
        self.debounce_time_us              = p('debounce_time_us').value
        self.pps_gpio_chip                 = p('pps_gpio_chip').value
        self.pps_gpio_line                 = p('pps_gpio_line').value
        self.pps_frame_id                  = p('pps_frame_id').value
        self.pps_debounce_time_us          = p('pps_debounce_time_us').value
        self.freq_window_size              = p('freq_window_size').value
        self.running_freq_window_size      = p('running_freq_window_size').value
        self.debug_mode                    = p('debug_mode').value
        self.debug_duration_s              = p('debug_duration_s').value
        self.enable_continuous_diagnostics = p('enable_continuous_diagnostics').value
        self.diagnostic_period             = p('diagnostic_period').value
        self.stop_capture_grace_ms         = p('stop_capture_grace_ms').value
        self.stop_empty_polls_before_exit  = p('stop_empty_polls_before_exit').value
        self.start_drain_ms                = p('start_drain_ms').value

        # ── Serial (shared, one connection) ───────────────────────────────────
        self._serial_lock      = threading.Lock()
        self.serial_conn       = None
        self._serial_last_fail = 0.0
        self._init_serial()

        # ── GQ7 PPS state ─────────────────────────────────────────────────────
        self._time_sync_ok  = False
        self._sync_lock     = threading.Lock()
        self._pps_edge_count = 0

        # ── Trigger session state ─────────────────────────────────────────────
        self.is_running         = False
        self.trigger_gpio_req   = None
        self.gpio_thread        = None
        self.gpio_stop_event    = threading.Event()
        self.gpio_ready_event   = threading.Event()
        self.first_trigger      = True
        self.time_offset        = None
        self.start_cmd_mono_ns  = None
        self.stop_cmd_mono_ns   = None
        self.first_edge_mono_ns = None
        self.last_edge_mono_ns  = None
        self.diagnostic_timer   = None
        self.debug_timer        = None

        # Diagnostic accumulators
        self._diag_lock          = threading.Lock()
        self._diag_count         = 0
        self._diag_intervals_ns  = []
        self._diag_first_ts_ns   = None
        self._diag_last_ts_ns    = None
        self._diag_running_ts    = deque(maxlen=self.running_freq_window_size)
        self._diag_window_open   = False

        self._session_result_lock   = threading.Lock()
        self._session_diag_count    = None
        self._session_diag_start_ts = None
        self._session_diag_stop_ts  = None
        self._session_intervals     = None

        self.sync_detector   = SyncPatternDetector()
        self._detector_armed = False

        # ── ROS interfaces ────────────────────────────────────────────────────
        # Trigger publishers + services
        self.timestamp_pub      = self.create_publisher(Header, '/teensy/timestamps',        10)
        self.pulse_interval_pub = self.create_publisher(Int64,  '/teensy/pulse_interval_ns',  10)
        diag_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.diagnostic_pub = self.create_publisher(DiagnosticArray, '/teensy/diagnostics', diag_qos)

        self.start_srv = self.create_service(Trigger, '/teensy/start_trigger', self.start_trigger_callback)
        self.stop_srv  = self.create_service(Trigger, '/teensy/stop_trigger',  self.stop_trigger_callback)

        # GQ7 publisher + subscription
        self._time_pub = self.create_publisher(TimeReference, '/gq7/ext/time', 10)
        self._sync_sub = self.create_subscription(
            MipSystemTimeSyncStatus,
            '/gq7/mip/system/time_sync_status',
            self._sync_status_cb,
            10)

        # ── PPS GPIO (always-on) ──────────────────────────────────────────────
        self._pps_gpio_req  = None
        self._pps_stop_evt  = threading.Event()
        if self._init_pps_gpio():
            self._pps_thread = threading.Thread(
                target=self._pps_gpio_loop, daemon=True)
            self._pps_thread.start()
        else:
            self.get_logger().error('PPS GPIO init failed — /gq7/ext/time will not publish')
            self._pps_thread = None

        # ── Send START to Teensy ──────────────────────────────────────────────
        self._send_command('START')
        self.get_logger().info('✓ Teensy START sent on launch')

        self.get_logger().info(
            f'Ready | serial={self.serial_port} | '
            f'trigger GPIO={self.trigger_gpio_chip}:{self.trigger_gpio_line} | '
            f'PPS GPIO={self.pps_gpio_chip}:{self.pps_gpio_line}')

    # ─────────────────────────────────────────────────────────────────────────
    # Serial — shared, lock-protected
    # ─────────────────────────────────────────────────────────────────────────

    def _init_serial(self) -> None:
        """Open serial port. Logs warning on failure (node continues without it)."""
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
        """Send a command to Teensy and return the last response line. Serial-locked."""
        with self._serial_lock:
            if self.serial_conn is None:
                return None
            try:
                self.serial_conn.write(f'{cmd}\n'.encode())
                self.serial_conn.flush()
                time.sleep(0.3 if cmd == 'STATUS' else 0.2)
                lines = []
                while self.serial_conn.in_waiting:
                    line = self.serial_conn.readline().decode().strip()
                    if line:
                        lines.append(line)
                if lines:
                    for l in lines:
                        self.get_logger().info(f'Teensy: {l}')
                    return lines[-1]
                self.get_logger().warn(f'No response for {cmd}')
                return None
            except Exception as e:
                self.get_logger().error(f'Serial error sending {cmd}: {e}')
                return None

    def _query_ptp_lastpps(self) -> int | None:
        """Send LASTPPS, return edge timestamp as nanoseconds. Serial-locked."""
        with self._serial_lock:
            if self.serial_conn is None:
                now = time.monotonic()
                if now - self._serial_last_fail >= self._SERIAL_RECONNECT_COOLDOWN:
                    # Serial is managed globally — just note unavailability
                    self._serial_last_fail = now
                return None
            try:
                self.serial_conn.reset_input_buffer()
                self.serial_conn.write(b'LASTPPS\n')
                self.serial_conn.flush()
                line = self.serial_conn.readline().decode('ascii', errors='replace').strip()
                m = self._LASTPPS_RE.search(line)
                if not m:
                    self.get_logger().warn(f'LASTPPS parse failed, got: {line!r}')
                    return None
                return int(m.group(1)) * 1_000_000_000 + int(m.group(2))
            except serial.SerialException as e:
                self.get_logger().warn(f'LASTPPS serial error: {e}')
                return None
            except Exception as e:
                self.get_logger().warn(f'LASTPPS unexpected error: {e}')
                return None

    def _read_teensy_session_count(self, wait_s: float = 0.6) -> int | None:
        """Poll for async session count line from Teensy after SYNC_STOP."""
        deadline = time.time() + wait_s
        while time.time() < deadline:
            line = None
            with self._serial_lock:
                if self.serial_conn and self.serial_conn.in_waiting:
                    line = self.serial_conn.readline().decode().strip()
            if line:
                self.get_logger().info(f'Teensy async: {line}')
                m = re.search(r'Session pulses.*?:\s*(\d+)', line)
                if m:
                    count = int(m.group(1))
                    self.get_logger().info(f'Teensy session count: {count}')
                    return count
            else:
                time.sleep(0.01)
        self.get_logger().warn('Teensy session count not received within timeout')
        return None

    # ─────────────────────────────────────────────────────────────────────────
    # GQ7 PPS — subscription callback
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
    # PPS GPIO — always-on thread
    # ─────────────────────────────────────────────────────────────────────────

    def _init_pps_gpio(self) -> bool:
        try:
            settings = gpiod.LineSettings()
            settings.direction      = Direction.INPUT
            settings.edge_detection = Edge.RISING
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
                events = req.read_edge_events()
            except ValueError:
                continue

            with self._sync_lock:
                synced = self._time_sync_ok

            if not synced:
                continue

            # Query PTP timestamp from Teensy via shared serial (locked inside)
            ptp_ns = self._query_ptp_lastpps()
            self._pps_edge_count += 1

            if ptp_ns is None:
                self.get_logger().warn(
                    f'PPS edge #{self._pps_edge_count} | ptp=unavailable')
                continue

            ptp_sec, ptp_nsec = divmod(ptp_ns, 1_000_000_000)
            self.get_logger().info(
                f'PPS edge #{self._pps_edge_count} | ptp={ptp_sec}.{ptp_nsec:09d}s')

            ros_time = Time(nanoseconds=ptp_ns)
            msg = TimeReference()
            msg.header.stamp    = ros_time.to_msg()
            msg.header.frame_id = self.pps_frame_id
            msg.time_ref        = ros_time.to_msg()
            msg.source          = 'gq7_pps'
            self._time_pub.publish(msg)

        self.get_logger().info('PPS GPIO thread exiting')

    # ─────────────────────────────────────────────────────────────────────────
    # Trigger GPIO — per-session
    # ─────────────────────────────────────────────────────────────────────────

    def _init_trigger_gpio(self) -> bool:
        try:
            settings = gpiod.LineSettings()
            settings.direction      = Direction.INPUT
            settings.edge_detection = Edge.RISING
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

        timeout_count            = 0
        max_timeouts_before_warn = 50
        stop_seen                = False
        stop_empty_polls         = 0

        try:
            while True:
                req = self.trigger_gpio_req
                if req is None:
                    break
                if req.wait_edge_events(timeout=datetime.timedelta(milliseconds=100)):
                    req = self.trigger_gpio_req
                    if req is None:
                        break
                    try:
                        events = req.read_edge_events()
                    except ValueError:
                        continue
                    if stop_seen:
                        stop_empty_polls = 0
                    timeout_count = 0

                    for event in events:
                        hte_ts_ns = event.timestamp_ns

                        if self.first_trigger and self.time_offset is None:
                            self.time_offset = (
                                time.clock_gettime_ns(time.CLOCK_REALTIME) - hte_ts_ns)

                        final_ts_ns = hte_ts_ns + self.time_offset
                        ros_time    = Time(nanoseconds=final_ts_ns)

                        if self.first_trigger:
                            self.first_trigger = False
                            self.get_logger().info(
                                f'First trigger | ROS: {ros_time.nanoseconds / 1e9:.6f} s')

                        hdr          = Header()
                        hdr.stamp    = ros_time.to_msg()
                        hdr.frame_id = self.trigger_frame_id
                        self.timestamp_pub.publish(hdr)

                        mono_ns = time.monotonic_ns()
                        if self.first_edge_mono_ns is None:
                            self.first_edge_mono_ns = mono_ns
                        self.last_edge_mono_ns = mono_ns

                        if not self._detector_armed:
                            continue
                        result    = self.sync_detector.on_pulse(hte_ts_ns)
                        skip_diag = False

                        if result == 'start':
                            with self._diag_lock:
                                self._diag_window_open = True
                            skip_diag = True
                            self.get_logger().info(
                                '★ SYNC_START detected — diagnostic window OPEN')

                        elif result == 'stop':
                            with self._diag_lock:
                                self._diag_window_open = False

                            diag_pulses = self.sync_detector.diag_pulses
                            n           = len(diag_pulses)
                            intervals   = [
                                diag_pulses[i] - diag_pulses[i - 1]
                                for i in range(1, n)
                            ] if n >= 2 else []

                            with self._session_result_lock:
                                self._session_diag_count    = n
                                self._session_diag_start_ts = (
                                    diag_pulses[0] if diag_pulses else None)
                                self._session_diag_stop_ts  = (
                                    diag_pulses[-1] if diag_pulses else None)
                                self._session_intervals     = intervals

                            self.get_logger().info(
                                f'★ SYNC_STOP detected — window CLOSED | session pulses: {n}')

                        with self._diag_lock:
                            window_open = self._diag_window_open

                        if window_open and not skip_diag:
                            self._diag_add_pulse(hte_ts_ns)
                            with self._diag_lock:
                                if len(self._diag_intervals_ns) >= 1:
                                    iv_msg      = Int64()
                                    iv_msg.data = self._diag_intervals_ns[-1]
                                    self.pulse_interval_pub.publish(iv_msg)

                        with self._diag_lock:
                            cnt      = self._diag_count
                            win_open = self._diag_window_open
                        if win_open:
                            if cnt == 1:
                                self.get_logger().info(
                                    f'Session pulse #1 | ts {ros_time.nanoseconds / 1e9:.6f} s')
                            elif cnt > 0 and cnt % 30 == 0:
                                self.get_logger().info(f'Session pulse #{cnt}')

                else:
                    if self.gpio_stop_event.is_set():
                        stop_seen = True
                        stop_empty_polls += 1
                        if stop_empty_polls >= self.stop_empty_polls_before_exit:
                            break
                        continue
                    timeout_count += 1
                    if timeout_count == max_timeouts_before_warn:
                        self.get_logger().warn(
                            'No GPIO triggers for 5 s — check wiring')

        except Exception as e:
            self.get_logger().error(f'Trigger GPIO thread error: {e}')
        finally:
            with self._session_result_lock:
                cnt = self._session_diag_count
            self.get_logger().info(
                f'Trigger GPIO thread stopped | session pulses (frozen): {cnt}')

    def _drain_pending_trigger_events(self, max_wait_ms: float = 50.0):
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

    def _stop_trigger_gpio_monitoring(self, grace_ms: float = 0.0):
        if grace_ms > 0:
            time.sleep(grace_ms / 1000.0)
        self.gpio_stop_event.set()
        if self.gpio_thread:
            self.gpio_thread.join(timeout=2.0)
            self.gpio_thread = None
        self.gpio_ready_event.clear()

    def _log_edge_timing(self):
        if self.start_cmd_mono_ns and self.first_edge_mono_ns:
            self.get_logger().info(
                f'start→first_edge: '
                f'{(self.first_edge_mono_ns - self.start_cmd_mono_ns) / 1e6:.2f} ms')
        if self.stop_cmd_mono_ns and self.last_edge_mono_ns:
            self.get_logger().info(
                f'last_edge vs stop_cmd: '
                f'{(self.last_edge_mono_ns - self.stop_cmd_mono_ns) / 1e6:.2f} ms')

    # ─────────────────────────────────────────────────────────────────────────
    # Diagnostic accumulators
    # ─────────────────────────────────────────────────────────────────────────

    def _diag_add_pulse(self, hte_ts_ns: int):
        with self._diag_lock:
            if self._diag_first_ts_ns is None:
                self._diag_first_ts_ns = hte_ts_ns
            if self._diag_last_ts_ns is not None:
                self._diag_intervals_ns.append(hte_ts_ns - self._diag_last_ts_ns)
            self._diag_last_ts_ns = hte_ts_ns
            self._diag_count     += 1
            self._diag_running_ts.append(hte_ts_ns)

    def _diag_reset(self):
        with self._diag_lock:
            self._diag_count        = 0
            self._diag_intervals_ns = []
            self._diag_first_ts_ns  = None
            self._diag_last_ts_ns   = None
            self._diag_running_ts.clear()
            self._diag_window_open  = False

    def _diag_snapshot(self) -> dict:
        with self._diag_lock:
            return dict(
                count       = self._diag_count,
                intervals   = list(self._diag_intervals_ns),
                first_ts_ns = self._diag_first_ts_ns,
                last_ts_ns  = self._diag_last_ts_ns,
                running_ts  = list(self._diag_running_ts),
            )

    # ─────────────────────────────────────────────────────────────────────────
    # Diagnostics publisher
    # ─────────────────────────────────────────────────────────────────────────

    def _publish_diagnostic(self, is_final: bool = False,
                            teensy_session_count: int | None = None):
        try:
            if is_final:
                with self._session_result_lock:
                    diag_count  = self._session_diag_count  or 0
                    intervals   = list(self._session_intervals or [])
                    first_ts_ns = self._session_diag_start_ts
                    last_ts_ns  = self._session_diag_stop_ts
                    running_ts  = []
            else:
                snap        = self._diag_snapshot()
                diag_count  = snap['count']
                intervals   = snap['intervals']
                first_ts_ns = snap['first_ts_ns']
                last_ts_ns  = snap['last_ts_ns']
                running_ts  = snap['running_ts']

            if is_final:
                teensy_total = teensy_session_count
            else:
                resp         = self._send_command('STATUS')
                teensy_total = self._parse_pulse_count(resp) or 0

            runtime_s  = 0.0
            total_freq = 0.0
            if diag_count >= 2 and first_ts_ns and last_ts_ns and last_ts_ns > first_ts_ns:
                runtime_s  = (last_ts_ns - first_ts_ns) / 1e9
                total_freq = (diag_count - 1) / runtime_s

            running_freq = 0.0
            if len(running_ts) >= 2:
                diffs = [running_ts[i] - running_ts[i - 1]
                         for i in range(1, len(running_ts))]
                running_freq = 1e9 / (sum(diffs) / len(diffs))
            elif is_final:
                running_freq = total_freq

            if len(intervals) >= 2:
                iv_min    = min(intervals)
                iv_max    = max(intervals)
                iv_mean   = statistics.mean(intervals)
                iv_std    = statistics.stdev(intervals)
                iv_unique = sorted(set(intervals))
            elif len(intervals) == 1:
                iv_min = iv_max = iv_mean = intervals[0]
                iv_std    = 0.0
                iv_unique = [intervals[0]]
            else:
                iv_min = iv_max = iv_mean = iv_std = 0
                iv_unique = []

            GROUP_RES_NS = 32
            groups = []
            for v in iv_unique:
                placed = False
                for g in groups:
                    if any(abs(v - m) <= GROUP_RES_NS for m in g):
                        g.append(v)
                        placed = True
                        break
                if not placed:
                    groups.append([v])
            iv_group_count   = len(groups)
            iv_group_centres = [round(statistics.mean(g)) for g in groups]
            iv_group_sizes   = [len(g) for g in groups]
            iv_group_summary = '; '.join(
                f'g{i+1}: centre={c} ns, members={s}'
                for i, (c, s) in enumerate(zip(iv_group_centres, iv_group_sizes)))

            if teensy_total is not None and teensy_total > 0:
                dropped   = teensy_total - diag_count
                drop_note = f'orin={diag_count} | teensy={teensy_total} | dropped={dropped}'
            else:
                dropped   = 0
                drop_note = 'teensy session count unavailable'

            if is_final:
                msg = (f"Stopped: {diag_count} session pulses captured "
                       f"| {total_freq:.3f} Hz | runtime {runtime_s:.1f} s")
            else:
                msg = f"Running: {diag_count} session pulses | {total_freq:.3f} Hz"

            status             = DiagnosticStatus()
            status.level       = DiagnosticStatus.OK
            status.name        = 'teensy_trigger'
            status.message     = msg
            status.hardware_id = 'Teensy 4.1'
            status.values      = [
                KeyValue(key='session_pulse_count',       value=str(diag_count)),
                KeyValue(key='teensy_session_count',      value=str(teensy_total if teensy_total is not None else 'N/A')),
                KeyValue(key='dropped_triggers',          value=str(dropped)),
                KeyValue(key='cross_check',               value=drop_note),
                KeyValue(key='frequency_total_hz',        value=f'{total_freq:.4f}'),
                KeyValue(key='frequency_running_hz',      value=f'{running_freq:.4f}'),
                KeyValue(key='runtime_seconds',           value=f'{runtime_s:.3f}'),
                KeyValue(key='session_first_ts_ns',       value=str(first_ts_ns)),
                KeyValue(key='session_last_ts_ns',        value=str(last_ts_ns)),
                KeyValue(key='interval_min_ns',           value=str(iv_min)),
                KeyValue(key='interval_max_ns',           value=str(iv_max)),
                KeyValue(key='interval_mean_ns',          value=f'{iv_mean:.1f}'),
                KeyValue(key='interval_std_ns',           value=f'{iv_std:.1f}'),
                KeyValue(key='interval_sample_count',     value=str(len(intervals))),
                KeyValue(key='interval_unique_count',     value=str(len(iv_unique))),
                KeyValue(key='interval_unique_values_ns', value=str(iv_unique)),
                KeyValue(key='interval_group_count',      value=str(iv_group_count)),
                KeyValue(key='interval_group_centres_ns', value=str(iv_group_centres)),
                KeyValue(key='interval_group_sizes',      value=str(iv_group_sizes)),
                KeyValue(key='interval_group_summary',    value=iv_group_summary),
            ]

            arr              = DiagnosticArray()
            arr.header.stamp = self.get_clock().now().to_msg()
            arr.status       = [status]
            self.diagnostic_pub.publish(arr)
            self.get_logger().info(f'Diagnostic: {msg}')

        except Exception as e:
            self.get_logger().error(f'Diagnostic publish failed: {e}')

    def _parse_pulse_count(self, response: str | None) -> int | None:
        if not response:
            return None
        m = re.search(r'(?:Total pulses|Pulses):\s*(\d+)', response)
        return int(m.group(1)) if m else None

    # ─────────────────────────────────────────────────────────────────────────
    # Session helpers
    # ─────────────────────────────────────────────────────────────────────────

    def _reset_run_state(self):
        self.first_trigger      = True
        self.time_offset        = None
        self.start_cmd_mono_ns  = None
        self.stop_cmd_mono_ns   = None
        self.first_edge_mono_ns = None
        self.last_edge_mono_ns  = None
        self.sync_detector.reset()
        self._detector_armed = False
        self._diag_reset()
        with self._session_result_lock:
            self._session_diag_count    = None
            self._session_diag_start_ts = None
            self._session_diag_stop_ts  = None
            self._session_intervals     = None

    def _do_stop(self) -> int | None:
        self.stop_cmd_mono_ns = time.monotonic_ns()
        self._send_command('SYNC_STOP')
        time.sleep(MARKER_LEN / 30.0 + 0.1)
        return self._read_teensy_session_count()

    # ─────────────────────────────────────────────────────────────────────────
    # Service callbacks
    # ─────────────────────────────────────────────────────────────────────────

    def start_trigger_callback(self, request, response):
        if self.is_running:
            response.success = False
            response.message = 'Already running'
            return response

        if not self._init_trigger_gpio():
            response.success = False
            response.message = 'Trigger GPIO init failed'
            return response

        self._reset_run_state()
        self.gpio_stop_event.clear()
        self.gpio_ready_event.clear()

        self._drain_pending_trigger_events(self.start_drain_ms)

        self.gpio_thread = threading.Thread(
            target=self._gpio_monitor_thread, daemon=True)
        self.gpio_thread.start()

        if not self.gpio_ready_event.wait(timeout=0.5):
            response.success = False
            response.message = 'Trigger GPIO thread not ready'
            self._stop_trigger_gpio_monitoring()
            if self.trigger_gpio_req:
                self.trigger_gpio_req.release()
                self.trigger_gpio_req = None
            return response

        self.start_cmd_mono_ns = time.monotonic_ns()
        self.sync_detector.reset()
        self._detector_armed = True
        # Wait for recording to be fully initialized before SYNC_START
        time.sleep(0.5)
        teensy_resp = self._send_command('SYNC_START')

        if not teensy_resp or 'OK' not in teensy_resp:
            response.success = False
            response.message = f'SYNC_START failed: {teensy_resp}'
            self.gpio_stop_event.set()
            if self.gpio_thread:
                self.gpio_thread.join(timeout=2.0)
                self.gpio_thread = None
            if self.trigger_gpio_req:
                self.trigger_gpio_req.release()
                self.trigger_gpio_req = None
            return response

        self.is_running = True

        if self.enable_continuous_diagnostics:
            self.diagnostic_timer = self.create_timer(
                self.diagnostic_period, self._publish_diagnostic)

        if self.debug_mode:
            self.debug_timer = self.create_timer(
                self.debug_duration_s, self._debug_auto_stop)

        response.success = True
        response.message = 'SYNC_START sent — marker firing, then continuous 30 Hz'
        self.get_logger().info('✓ Started')
        return response

    def stop_trigger_callback(self, request, response):
        if not self.is_running:
            response.success = False
            response.message = 'Not running'
            return response

        if self.debug_timer:
            self.debug_timer.cancel()
            self.debug_timer = None
        if self.diagnostic_timer:
            self.diagnostic_timer.cancel()
            self.diagnostic_timer = None

        teensy_sess_count = self._do_stop()
        self._stop_trigger_gpio_monitoring(self.stop_capture_grace_ms)
        if self.trigger_gpio_req:
            self.trigger_gpio_req.release()
            self.trigger_gpio_req = None
        self._log_edge_timing()
        self._publish_diagnostic(is_final=True, teensy_session_count=teensy_sess_count)
        self._reset_run_state()
        self.is_running = False
        response.success = True
        response.message = 'Stopped'
        self.get_logger().info('✓ Stopped')
        return response

    def _debug_auto_stop(self):
        self.get_logger().warn('DEBUG: auto-stopping')
        if self.diagnostic_timer:
            self.diagnostic_timer.cancel()
            self.diagnostic_timer = None

        teensy_sess_count = self._do_stop()
        self._stop_trigger_gpio_monitoring(self.stop_capture_grace_ms)
        if self.trigger_gpio_req:
            self.trigger_gpio_req.release()
            self.trigger_gpio_req = None
        self._log_edge_timing()
        self._publish_diagnostic(is_final=True, teensy_session_count=teensy_sess_count)
        self._reset_run_state()
        self.is_running = False
        if self.debug_timer:
            self.debug_timer.cancel()
            self.debug_timer = None
        self.get_logger().info('✓ Debug auto-stop complete')

    # ─────────────────────────────────────────────────────────────────────────
    # Cleanup
    # ─────────────────────────────────────────────────────────────────────────

    def cleanup(self):
        # Stop PPS thread
        self._pps_stop_evt.set()
        if self._pps_thread and self._pps_thread.is_alive():
            self._pps_thread.join(timeout=1.0)
        if self._pps_gpio_req:
            try:
                self._pps_gpio_req.release()
            except Exception:
                pass
            self._pps_gpio_req = None

        # Stop trigger session if active
        if self.debug_timer:
            self.debug_timer.cancel()
        if self.diagnostic_timer:
            self.diagnostic_timer.cancel()
        if self.is_running:
            self._stop_trigger_gpio_monitoring(self.stop_capture_grace_ms)
        if self.trigger_gpio_req:
            try:
                self.trigger_gpio_req.release()
            except Exception:
                pass

        # Send STOP and close serial (once)
        if self.serial_conn and self.serial_conn.is_open:
            self._send_command('STOP')
            with self._serial_lock:
                self.serial_conn.close()
            self.serial_conn = None

        self.get_logger().info('Cleanup complete')


# ─────────────────────────────────────────────────────────────────────────────

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
