#!/usr/bin/env python3
"""
ROS2 Node: Teensy 30Hz Wave Trigger with HTE Timestamps
========================================================

Services:
  - /teensy/start_trigger  → sends SYNC_START marker, then continuous 30 Hz
  - /teensy/stop_trigger   → sends SYNC_STOP  marker, then STOP

Topics:
  - /teensy/timestamps          (std_msgs/msg/Header)  — every rising edge
  - /teensy/pulse_interval_ns   (std_msgs/msg/Int64)   — interval between edges

Diagnostics session window
--------------------------
  • Starts : first pulse AFTER the SYNC_START pattern completes  (bit 13+)
  • Ends   : last  pulse BEFORE the SYNC_STOP  pattern begins    (pulse N-1)
  • The marker pulses themselves are excluded from all diagnostics.

Sync pattern: 1 0 1 1 0 1 0 1 1 0 1 0 1  (13 bits at 30 Hz, ~433 ms)
  Max consecutive zeros = 1  →  gaps are only ~33 ms or ~66 ms.
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_srvs.srv import Trigger
from std_msgs.msg import Header, Int64
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import serial
import time
import sys
import threading
import gpiod
from gpiod.line import Direction, Edge, Clock
import datetime
from collections import deque
import re
import statistics
import os

# ─────────────────────────────────────────────────────────────────────────────
# Sync pattern constants
# ─────────────────────────────────────────────────────────────────────────────
MARKER_BITS     = [1, 0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1]
MARKER_LEN      = len(MARKER_BITS)
FRAME_PERIOD_NS = 1_000_000_000 // 30   # 33_333_333 ns


def _slots_from_gap(gap_ns: int) -> int:
    """Round gap to nearest frame count (clamped 1–2)."""
    return max(1, min(round(gap_ns / FRAME_PERIOD_NS), 2))


# ─────────────────────────────────────────────────────────────────────────────
# Pattern detector
# ─────────────────────────────────────────────────────────────────────────────

class SyncPatternDetector:
    """
    Detects the 13-bit sync marker in a stream of rising-edge timestamps.

    State machine
    -------------
    'idle'       — waiting for SYNC_START pattern
    'in_session' — between patterns; accumulates diagnostic pulses

    After on_pulse() returns 'start':
        diag_start_ts_ns  — ts of first pulse AFTER the SYNC_START pattern

    After on_pulse() returns 'stop':
        diag_stop_ts_ns   — ts of last pulse BEFORE the SYNC_STOP pattern
        diag_pulses       — list of HTE timestamps of every diagnostic pulse
    """

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
        """
        Feed every rising edge.  Returns:
          'start'  — SYNC_START pattern complete
          'stop'   — SYNC_STOP  pattern complete
          None     — nothing completed yet
        """
        self._last_pulse_ts = timestamp_ns

        # ── First pulse ever ─────────────────────────────────────────────────
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
            # Strip SYNC_STOP marker pulses contaminating the diag list
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
# ROS2 Node
# ─────────────────────────────────────────────────────────────────────────────

class TeensyWaveTriggerNode(Node):

    def __init__(self):
        super().__init__('teensy_wave_trigger_node')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('serial_port',                   '/dev/ttyACM0')
        self.declare_parameter('baud_rate',                     921600)
        self.declare_parameter('gpio_chip',                     '/dev/gpiochip1')
        self.declare_parameter('gpio_line',                     8)
        self.declare_parameter('frame_id',                      'teensy_trigger')
        self.declare_parameter('debounce_time_us',              0)
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
        self.gpio_chip                     = p('gpio_chip').value
        self.gpio_line                     = p('gpio_line').value
        self.frame_id                      = p('frame_id').value
        self.debounce_time_us              = p('debounce_time_us').value
        self.freq_window_size              = p('freq_window_size').value
        self.running_freq_window_size      = p('running_freq_window_size').value
        self.debug_mode                    = p('debug_mode').value
        self.debug_duration_s              = p('debug_duration_s').value
        self.enable_continuous_diagnostics = p('enable_continuous_diagnostics').value
        self.diagnostic_period             = p('diagnostic_period').value
        self.stop_capture_grace_ms         = p('stop_capture_grace_ms').value
        self.stop_empty_polls_before_exit  = p('stop_empty_polls_before_exit').value
        self.start_drain_ms                = p('start_drain_ms').value

        # ── Runtime state ─────────────────────────────────────────────────────
        self.is_running         = False
        self.serial_conn        = None
        self.gpio_request       = None
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

        # ── Diagnostic accumulators (session window only) ─────────────────────
        self._diag_lock          = threading.Lock()
        self._diag_count         = 0
        self._diag_intervals_ns  = []
        self._diag_first_ts_ns   = None
        self._diag_last_ts_ns    = None
        self._diag_running_ts    = deque(maxlen=self.running_freq_window_size)
        self._diag_window_open   = False

        # Frozen result written by GPIO thread when SYNC_STOP detected
        self._session_result_lock   = threading.Lock()
        self._session_diag_count    = None
        self._session_diag_start_ts = None
        self._session_diag_stop_ts  = None
        self._session_intervals     = None

        # Pattern detector
        self.sync_detector   = SyncPatternDetector()
        self._detector_armed = False   # True only after SYNC_START command confirmed

        # ── Serial ────────────────────────────────────────────────────────────
        self._init_serial()

        # ── ROS interfaces ────────────────────────────────────────────────────
        self.start_srv = self.create_service(
            Trigger, '/teensy/start_trigger', self.start_trigger_callback)
        self.stop_srv = self.create_service(
            Trigger, '/teensy/stop_trigger',  self.stop_trigger_callback)

        self.timestamp_pub      = self.create_publisher(Header, '/teensy/timestamps',       10)
        self.pulse_interval_pub = self.create_publisher(Int64,  '/teensy/pulse_interval_ns', 10)

        diag_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, '/teensy/diagnostics', diag_qos)

        self.get_logger().info(
            f'Ready | serial={self.serial_port} gpio={self.gpio_chip}:{self.gpio_line} | '
            f'pattern={MARKER_BITS} ({MARKER_LEN} bits @ 30 Hz = '
            f'{MARKER_LEN * FRAME_PERIOD_NS / 1e6:.0f} ms)')

        self._send_command('START')
        self.get_logger().info('✓ Teensy START sent on launch')

    # ─────────────────────────────────────────────────────────────────────────
    # Serial helpers
    # ─────────────────────────────────────────────────────────────────────────

    def _init_serial(self):
        try:
            self.serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            time.sleep(2)
            while self.serial_conn.in_waiting:
                self.get_logger().info(
                    f'Teensy: {self.serial_conn.readline().decode().strip()}')
            self.get_logger().info('Serial ready')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial open failed: {e}')
            sys.exit(1)

    def _send_command(self, cmd: str) -> str | None:
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
            self.get_logger().error(f'Serial error: {e}')
            return None

    def _parse_pulse_count(self, response: str | None) -> int | None:
        if not response:
            return None
        m = re.search(r'(?:Total pulses|Pulses):\s*(\d+)', response)
        return int(m.group(1)) if m else None

    def _parse_teensy_session_count(self, response: str | None) -> int | None:
        if not response:
            return None
        m = re.search(r'Session pulses.*?:\s*(\d+)', response)
        return int(m.group(1)) if m else None

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

            # ── Frequency ─────────────────────────────────────────────────────
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

            # ── Interval stats ─────────────────────────────────────────────────
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

            # 32 ns group analysis
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
                for i, (c, s) in enumerate(zip(iv_group_centres, iv_group_sizes))
            )

            # ── Cross-check ────────────────────────────────────────────────────
            if teensy_total is not None and teensy_total > 0:
                dropped   = teensy_total - diag_count
                drop_note = f'orin={diag_count} | teensy={teensy_total} | dropped={dropped}'
            else:
                dropped   = 0
                drop_note = 'teensy session count unavailable'

            # ── Build message ──────────────────────────────────────────────────
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

    # ─────────────────────────────────────────────────────────────────────────
    # GPIO
    # ─────────────────────────────────────────────────────────────────────────

    def _init_gpio(self) -> bool:
        try:
            settings = gpiod.LineSettings()
            settings.direction      = Direction.INPUT
            settings.edge_detection = Edge.RISING
            # Always set explicitly — omitting lets the kernel apply a 1 ms default
            settings.debounce_period = datetime.timedelta(
                microseconds=self.debounce_time_us)
            for clock_mode, label in [(Clock.HTE, 'HTE'), (Clock.MONOTONIC, 'MONOTONIC')]:
                try:
                    settings.event_clock = clock_mode
                    self.gpio_request = gpiod.request_lines(
                        path=self.gpio_chip,
                        consumer='teensy-trigger-monitor',
                        config={self.gpio_line: settings})
                    self.get_logger().info(f'GPIO clock: {label}')
                    return True
                except (OSError, AttributeError) as e:
                    self.get_logger().warn(f'{label} unavailable: {e}')
            self.get_logger().error('All GPIO clock modes failed')
            return False
        except Exception as e:
            self.get_logger().error(f'GPIO init error: {e}')
            return False

    def _gpio_monitor_thread(self):
        try:
            param = os.sched_param(os.sched_get_priority_max(os.SCHED_FIFO))
            os.sched_setscheduler(0, os.SCHED_FIFO, param)
        except PermissionError:
            self.get_logger().warn('GPIO thread: realtime priority unavailable')

        self.get_logger().info('GPIO thread started')
        self.gpio_ready_event.set()

        timeout_count            = 0
        max_timeouts_before_warn = 50
        stop_seen                = False
        stop_empty_polls         = 0

        try:
            while True:
                req = self.gpio_request
                if req is None:
                    break
                if req.wait_edge_events(timeout=datetime.timedelta(milliseconds=100)):
                    req = self.gpio_request
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

                        # ── HTE → realtime calibration (first pulse only) ──────
                        if self.first_trigger and self.time_offset is None:
                            self.time_offset = (
                                time.clock_gettime_ns(time.CLOCK_REALTIME) - hte_ts_ns)

                        final_ts_ns = hte_ts_ns + self.time_offset
                        ros_time    = Time(nanoseconds=final_ts_ns)

                        if self.first_trigger:
                            self.first_trigger = False
                            self.get_logger().info(
                                f'First trigger | ROS: {ros_time.nanoseconds / 1e9:.6f} s')

                        # ── Publish every edge unconditionally ────────────────
                        hdr          = Header()
                        hdr.stamp    = ros_time.to_msg()
                        hdr.frame_id = self.frame_id
                        self.timestamp_pub.publish(hdr)

                        mono_ns = time.monotonic_ns()
                        if self.first_edge_mono_ns is None:
                            self.first_edge_mono_ns = mono_ns
                        self.last_edge_mono_ns = mono_ns

                        # ── Pattern detection ─────────────────────────────────
                        if not self._detector_armed:
                            continue
                        result    = self.sync_detector.on_pulse(hte_ts_ns)
                        skip_diag = False

                        if result == 'start':
                            with self._diag_lock:
                                self._diag_window_open = True
                            skip_diag = True
                            self.get_logger().info(
                                '★ SYNC_START detected — diagnostic window OPEN '
                                '(first session pulse is the next one)')

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
                                f'★ SYNC_STOP detected — diagnostic window CLOSED | '
                                f'session pulses: {n}')

                        # ── Accumulate diagnostic pulse ───────────────────────
                        with self._diag_lock:
                            window_open = self._diag_window_open

                        if window_open and not skip_diag:
                            self._diag_add_pulse(hte_ts_ns)
                            with self._diag_lock:
                                if len(self._diag_intervals_ns) >= 1:
                                    iv_msg      = Int64()
                                    iv_msg.data = self._diag_intervals_ns[-1]
                                    self.pulse_interval_pub.publish(iv_msg)

                        # ── Periodic log ──────────────────────────────────────
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
            self.get_logger().error(f'GPIO thread error: {e}')
        finally:
            with self._session_result_lock:
                cnt = self._session_diag_count
            self.get_logger().info(
                f'GPIO thread stopped | session pulses (frozen): {cnt}')

    def _drain_pending_gpio_events(self, max_wait_ms: float = 50.0):
        """Discard stale GPIO edge events. Must be called before GPIO thread starts."""
        if not self.gpio_request:
            return
        end_t = time.time() + max_wait_ms / 1000.0
        while time.time() < end_t:
            if self.gpio_request.wait_edge_events(
                    timeout=datetime.timedelta(milliseconds=1)):
                try:
                    self.gpio_request.read_edge_events()
                except ValueError:
                    pass
            else:
                break

    def _log_edge_timing(self):
        if self.start_cmd_mono_ns and self.first_edge_mono_ns:
            self.get_logger().info(
                f'start→first_edge: '
                f'{(self.first_edge_mono_ns - self.start_cmd_mono_ns) / 1e6:.2f} ms')
        if self.stop_cmd_mono_ns and self.last_edge_mono_ns:
            self.get_logger().info(
                f'last_edge vs stop_cmd: '
                f'{(self.last_edge_mono_ns - self.stop_cmd_mono_ns) / 1e6:.2f} ms')

    def _stop_gpio_monitoring(self, grace_ms: float = 0.0):
        if grace_ms > 0:
            time.sleep(grace_ms / 1000.0)
        self.gpio_stop_event.set()
        if self.gpio_thread:
            self.gpio_thread.join(timeout=2.0)
            self.gpio_thread = None
        self.gpio_ready_event.clear()

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

    # ─────────────────────────────────────────────────────────────────────────
    # Service callbacks
    # ─────────────────────────────────────────────────────────────────────────

    def _read_teensy_session_count(self, wait_s: float = 0.6) -> int | None:
        deadline = time.time() + wait_s
        while time.time() < deadline:
            if self.serial_conn.in_waiting:
                line = self.serial_conn.readline().decode().strip()
                if not line:
                    continue
                self.get_logger().info(f'Teensy async: {line}')
                count = self._parse_teensy_session_count(line)
                if count is not None:
                    self.get_logger().info(
                        f'Teensy session count (excl. markers): {count}')
                    return count
            else:
                time.sleep(0.01)
        self.get_logger().warn('Teensy session count not received within timeout')
        return None

    def _do_stop(self) -> int | None:
        """Send SYNC_STOP, wait for pattern to complete, return Teensy session count."""
        self.stop_cmd_mono_ns = time.monotonic_ns()
        self._send_command('SYNC_STOP')
        time.sleep(MARKER_LEN / 30.0 + 0.1)
        return self._read_teensy_session_count()

    def start_trigger_callback(self, request, response):
        if self.is_running:
            response.success = False
            response.message = 'Already running'
            return response

        if not self._init_gpio():
            response.success = False
            response.message = 'GPIO init failed'
            return response

        self._reset_run_state()
        self.gpio_stop_event.clear()
        self.gpio_ready_event.clear()

        # Drain stale events BEFORE starting the GPIO thread — prevents the
        # drain from racing with the thread and consuming marker pulses.
        self._drain_pending_gpio_events(self.start_drain_ms)

        self.gpio_thread = threading.Thread(
            target=self._gpio_monitor_thread, daemon=True)
        self.gpio_thread.start()

        if not self.gpio_ready_event.wait(timeout=0.5):
            response.success = False
            response.message = 'GPIO thread not ready'
            self._stop_gpio_monitoring()
            if self.gpio_request:
                self.gpio_request.release()
                self.gpio_request = None
            return response

        self.start_cmd_mono_ns  = time.monotonic_ns()
        # Arm and reset detector immediately before sending the command so
        # the GPIO thread captures every marker bit from the very first one.
        self.sync_detector.reset()
        self._detector_armed = True
        teensy_resp = self._send_command('SYNC_START')

        if not teensy_resp or 'OK' not in teensy_resp:
            response.success = False
            response.message = f'SYNC_START failed: {teensy_resp}'
            self.gpio_stop_event.set()
            if self.gpio_thread:
                self.gpio_thread.join(timeout=2.0)
                self.gpio_thread = None
            if self.gpio_request:
                self.gpio_request.release()
                self.gpio_request = None
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
        # Join GPIO thread first — it must finish before we release gpio_request
        self._stop_gpio_monitoring(self.stop_capture_grace_ms)
        # Now safe to release — thread is no longer using gpio_request
        if self.gpio_request:
            self.gpio_request.release()
            self.gpio_request = None
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
        self._stop_gpio_monitoring(self.stop_capture_grace_ms)
        if self.gpio_request:
            self.gpio_request.release()
            self.gpio_request = None
        self._log_edge_timing()
        self._publish_diagnostic(is_final=True, teensy_session_count=teensy_sess_count)
        self._reset_run_state()
        self.is_running = False
        if self.debug_timer:
            self.debug_timer.cancel()
            self.debug_timer = None
        self.get_logger().info('✓ Debug auto-stop complete')

    def cleanup(self):
        if self.debug_timer:
            self.debug_timer.cancel()
        if self.diagnostic_timer:
            self.diagnostic_timer.cancel()

        if self.is_running:
            self._stop_gpio_monitoring(self.stop_capture_grace_ms)
            if self.gpio_request:
                self.gpio_request.release()

        # STOP is only ever sent here — silences the Teensy on shutdown
        if self.serial_conn and self.serial_conn.is_open:
            self._send_command('STOP')
            self.serial_conn.close()

        self.get_logger().info('Cleanup complete')


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = TeensyWaveTriggerNode()
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