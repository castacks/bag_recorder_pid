#!/usr/bin/env python3
"""
ROS2 Node: Teensy Combined — Wave Trigger + GQ7 PPS Time Sync

GPIO pins:
  trigger_gpio_chip/line  — 30 Hz wave trigger edges (always publishing from bootup)
  pps_gpio_chip/line      — 1 Hz PPS edges from the Teensy (also wired to GQ7 GPIO1)

State machine:
  IDLE           — GPIO running, timestamps publishing, no active session
  SESSION_ACTIVE — SYNC_START sent; returns to IDLE on stop_trigger

Services:
  /teensy/start_trigger    — checks sync (safety net), sends SYNC_START, IDLE → SESSION_ACTIVE
  /teensy/stop_trigger     — send SYNC_STOP, SESSION_ACTIVE → IDLE
  /teensy/wait_sync_ready  — PTPRESET, then wait until Teensy + GQ7 timing is healthy (before recording)
  /teensy/ptp_reset        — send PTPRESET (refused during a session)

Topics published:
  /teensy/timestamps  (std_msgs/Header)               — every rising trigger edge (CLOCK_REALTIME, from HTE)
  /gq7/ext/time       (sensor_msgs/TimeReference)     — PTP second of each PPS edge that passes C1–C6;
                                                         header.stamp = Orin time of the edge
  /teensy/ptp_status  (tartanstar_msgs/TeensyPtpStatus) — 1 Hz Teensy PTP / GQ7 sync diagnostics

Topics subscribed:
  /gq7/mip/system/time_sync_status — GQ7 time_sync, last_pps_rcvd, time_valid, GQ7 clock check

Requires the Teensy firmware that reports LOCK/AGE/OFFSET/PPS in LASTPPS (tartan_rgbt_ws 6000d07).

═══════════════════════════════════════════════════════════════════════════════════════════════════════
Timing design
═══════════════════════════════════════════════════════════════════════════════════════════════════════

Clock chain. Every sensor stamp is meant to be on the Orin's CLOCK_REALTIME:
  chrony (NTP) → CLOCK_REALTIME → phc2sys → X550 NIC clocks (enP5p1s0f1 Ouster, enP5p1s0f0 Teensy)
  → ptp4l (master) → Teensy 4.1 PTP slave (t41-ptp) → PPS on Teensy pin 24 at every whole PTP second
  → GQ7 GPIO1 and the Orin PPS GPIO (pps_gpio_chip/line).
  The GQ7 stamps its data with its own device time (timestamp_source 1). It takes the timing within the
  second from the PPS edge and the whole second from /gq7/ext/time, which this node publishes; the GQ7
  driver forwards time_ref to the GQ7 (only the whole second is used, and only within 250 ms of the edge).

Failure this design guards against. The Teensy's PTP servo stops correcting when its master changes rate by
  > 100 ppm and then free-runs. The old firmware kept sending PPS from that wrong clock, so the GQ7 followed
  it (recorded drift 0.2–0.4 ms/s, up to 1.4 s) while every GQ7 status flag (time_sync, last_pps_rcvd,
  time_valid) stayed healthy. The flags only say the GQ7 sees PPS and was given a time, not that the time is
  right. So the node compares clocks against the Orin directly instead of trusting flags:
  - the Teensy's second vs the Orin's own time of the PPS edge (C2), on every edge;
  - the GQ7's clock vs the Orin (GQ7 clock check), every second.

Teensy. The firmware outputs PPS only while it trusts its PTP time (fresh lock since boot / link-up /
  PTPRESET, an update in the last 3 s, |offset| < 10 µs, relock after a dropout). LASTPPS returns
  "<sec>.000000000 LOCK= AGE= OFFSET= PPS=": the PTP second of the last edge it sent plus its servo state.
  LOCK flickers in normal operation and is used for monitoring only.

PPS edge → /gq7/ext/time (_pps_monitor_thread, _check_edge). The PPS line is stamped with CLOCK_REALTIME by
  the kernel (orin_edge_ns). On each edge the node queries LASTPPS and publishes the second only if all pass:
  C1  the reply says PPS=1 (cross-check; read just after the edge);
  C2  |orin_edge_ns − sec·1e9| < ext_time_max_offset_ms (1 ms). Normally ~17 µs (GPIO latency). Catches a
      reply from the wrong second (±1 s), a Teensy locked to a wrong master, and slow drift the firmware
      misses;
  C3  publish within ext_time_max_latency_ms (150 ms) of the edge (the GQ7 needs it within 250 ms);
  C4  sec = previous published sec + 1 (unless after a gap): no repeated or skipped second;
  C5  a valid LASTPPS reply;
  C6  GQ7 time_sync True (the GQ7 reaches it from PPS alone, so this isn't circular).
  It is published on every edge that passes, not only when LOCK=1: the label is the whole second, the edge
  carries the timing, and resending every second corrects a GQ7 seconds miscount after any PPS gap or
  glitch, as the GQ7 manual asks. header.stamp = orin_edge_ns, time_ref = sec, so every bag records the
  Orin − Teensy offset at 1 Hz (header.stamp − time_ref).

GQ7 clock check (_gq7_clock_check). time_sync_status (1 Hz) is stamped with GQ7 device time, so
  d = receive time − header.stamp = GQ7 clock vs Orin + transport latency. Healthy: ~2.5–3.9 ms. Uses the
  minimum of the last 5 messages (a burst of late-delivered messages must not look like a clock error).
  Fails outside gq7_clock_min_ms..gq7_clock_max_ms (0–20 ms). This is the end-to-end check: the GQ7's
  output stamps agree with the Orin, whatever went wrong upstream. Checked only once the GQ7 has been
  given the time (right after a GQ7 power-up its clock is unset; that is normal startup).

Sync events (logged on transitions only; recorded in the bag through /rosout). WARN, also during a
  recording (after the GQ7 has the time, a PPS outage only lets it free-run on its own oscillator):
  W1 PPS stopped (with LOCK/AGE/OFFSET/PPS, which show the cause)   W7 PPS resumed
  W2 GQ7 time_sync True → False                                      W8 GQ7 time_sync False → True
  W3 /gq7/ext/time skipped (which of C1–C6 failed)                   W9 time sent after a gap
  W9a GQ7 did not accept the time (time_valid not True within 3 s after W9)
  W5 missed PPS edge (node fell behind)
  W6 LOCK=0 for > 10 s (servo problem; PPS may still be fine)        W10 LOCK=1 again
  W11 GQ7 clock off                                                  W12 GQ7 clock back
  W13 GQ7 not seeing PPS while the Orin is (wiring; 3 s grace)       W14 GQ7 sees PPS again
  ERROR only when not recoverable or the data would be wrong: W11 lasting ≥ gq7_clock_error_s (5 s; the
  GQ7 stamps in the bag are wrong), W4 no / garbled LASTPPS reply, Teensy firmware too old, PPS GPIO
  without CLOCK_REALTIME, wait_sync_ready failing, start_trigger refused.
  The node never sends PTPRESET or changes anything on its own during a recording: it skips and warns.

Recording flow (with the ros_rec app). sensor_reset (restarts ptp4l + phc2sys, so the Teensy relocks)
  → /teensy/wait_sync_ready → recorders start → /teensy/start_trigger.
  wait_sync_ready: PTPRESET first (a master restart doesn't clear a frozen servo), then waits up to
    wait_sync_timeout_s (45 s) until _sync_problems() is empty (LASTPPS answers with PPS=1, a PPS edge in
    the last 2 s, C2 passes, GQ7 time_sync, the time sent since the last gap, last_pps_rcvd ≤ 1,
    time_valid, GQ7 clock check) AND PPS has run without a gap for wait_sync_settle_s (10 s; the servo can
    overshoot right after relocking). On failure the app blocks the recording.
  start_trigger: the same checks as a safety net (the recorders are already running); refused on failure,
    and the app then stops the recording.
  wait_sync_ready and ptp_reset are refused during a session (PTPRESET steps the Teensy clock).

Trigger stamps (_init_trigger_gpio, _gpio_monitor_thread). The Teensy drives the 30 Hz trigger from its own
  crystal (not PTP); the Orin stamps each rising edge.
  - Edge.BOTH, rising events kept: with Edge.RISING alone the Tegra GPIO driver reports many falling edges as
    rising (~48/s instead of 30). With HTE it also reports some spurious falling events; they are dropped.
  - Debounce 0: the Tegra hardware debounce works in whole milliseconds and would put every stamp on a 1 ms
    grid. The Teensy drives the line directly, so there is nothing to debounce (verified: no doubled or
    missed edges).
  - trigger_event_clock 'hte' (default): the Tegra hardware timestamp engine stamps the edge at the pin, in
    ns of the ARM system counter. It is converted to CLOCK_REALTIME per batch of edges (_HteClock):
    counter − MONOTONIC_RAW is a constant, measured from the counter itself; REALTIME − MONOTONIC_RAW is
    read fresh each time, because chrony keeps a rate correction on REALTIME even when offline (~19 ppm
    measured). A one-time HTE → REALTIME offset drifted by up to 36 ms per session in the old node.
    Jitter ~0.2 µs std. 'realtime': the kernel stamps the edge in the interrupt handler, ~40–100 µs std
    (interrupt latency). HTE falls back to 'realtime' with a WARN if unavailable.
  - The PPS line can't use HTE (only the always-on GPIO controller supports it); REALTIME is fine for C2.
"""

import datetime
import math
import os
import re
import sys
import threading
import time
from collections import deque
from enum import Enum

import gpiod
import rclpy
import serial
from gpiod.line import Clock, Direction, Edge
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

from microstrain_inertial_msgs.msg import MipSystemTimeSyncStatus
from sensor_msgs.msg import TimeReference
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tartanstar_msgs.msg import TeensyPtpStatus


class _HteClock:
    """Converts Tegra HTE stamps to CLOCK_REALTIME, per edge.

    HTE stamps are the ARM system counter (31.25 MHz on Orin, 32 ns ticks) in ns since the counter started,
    which is MONOTONIC_RAW + a constant offset (the counter runs before Linux starts its clocks). The offset
    is measured from the counter itself (CNTVCT_EL0, readable from user space on arm64). REALTIME - MONOTONIC_RAW
    is read fresh for every batch of edges, because chrony keeps a rate correction on REALTIME even when offline
    (a one-time offset drifted by up to 36 ms per session in the old node).
    """

    def __init__(self):
        import ctypes
        import mmap
        import platform
        if platform.machine() != 'aarch64':
            raise RuntimeError('HTE conversion needs the arm64 system counter')
        # Two tiny functions: mrs x0, cntvct_el0 / cntfrq_el0 ; ret
        self._bufs = []
        def fn(mrs):
            buf = mmap.mmap(-1, mmap.PAGESIZE, prot=mmap.PROT_READ | mmap.PROT_WRITE | mmap.PROT_EXEC)
            buf.write(mrs.to_bytes(4, 'little') + (0xd65f03c0).to_bytes(4, 'little'))
            self._bufs.append(buf)
            return ctypes.CFUNCTYPE(ctypes.c_uint64)(ctypes.addressof(ctypes.c_char.from_buffer(buf)))
        self._cntvct = fn(0xd53be040)
        freq = fn(0xd53be000)()
        if freq <= 0 or NS % freq:
            raise RuntimeError(f'counter frequency {freq} Hz is not a whole number of ns per tick')
        self._ns_per_tick = NS // freq
        self.counter_minus_raw_ns = self.measure_offset()

    def measure_offset(self, n: int = 200) -> int:
        """Counter (ns) - MONOTONIC_RAW, median of the tightest reads (sub-µs)."""
        offs = []
        for _ in range(n):
            a = time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)
            c = self._cntvct()
            b = time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)
            if b - a < 20_000:
                offs.append(c * self._ns_per_tick - (a + b) // 2)
        if len(offs) < n // 4:
            raise RuntimeError('could not read the counter and MONOTONIC_RAW close together')
        offs.sort()
        return offs[len(offs) // 2]

    def to_realtime_ns(self) -> int:
        """Offset to add to an HTE stamp: REALTIME - MONOTONIC_RAW - (counter - MONOTONIC_RAW).

        The clock pair is read up to 3 times and the tightest read is used (a read interrupted between the two
        clocks would add its delay to every stamp of the batch)."""
        best = None
        for _ in range(3):
            r1 = time.clock_gettime_ns(time.CLOCK_REALTIME)
            mr = time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)
            r2 = time.clock_gettime_ns(time.CLOCK_REALTIME)
            if best is None or r2 - r1 < best[0]:
                best = (r2 - r1, (r1 + r2) // 2 - mr)
            if best[0] < 5_000:
                break
        return best[1] - self.counter_minus_raw_ns

MARKER_LEN = 13  # bits in SYNC_START / SYNC_STOP pattern
NS = 1_000_000_000


class NodeState(Enum):
    IDLE           = 'IDLE'
    SESSION_ACTIVE = 'SESSION_ACTIVE'


def _realtime_ns() -> int:
    return time.clock_gettime_ns(time.CLOCK_REALTIME)


class TeensyCombinedNode(Node):

    # LAST_PPS_HW: <sec>.000000000 LOCK=<0|1> AGE=<ms> OFFSET=<ns> PPS=<0|1>
    _LASTPPS_RE = re.compile(
        r'LAST_PPS_HW:\s*(\d+)\.(\d+)\s+LOCK=(\d)\s+AGE=(-?\d+)\s+OFFSET=(-?\d+)\s+PPS=(\d)')
    _LASTPPS_OLD_RE = re.compile(r'(\d+)\.(\d+)')

    def __init__(self):
        super().__init__('teensy_combined_node')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('serial_port',             '/dev/teensy_trigger')
        self.declare_parameter('baud_rate',               921600)
        self.declare_parameter('trigger_gpio_chip',       '/dev/gpiochip1')
        self.declare_parameter('trigger_gpio_line',       8)
        self.declare_parameter('trigger_frame_id',        'teensy_trigger')
        self.declare_parameter('debounce_time_us',        0)
        self.declare_parameter('trigger_event_clock',     'hte')  # 'hte' (converted per edge) or 'realtime'
        self.declare_parameter('pps_gpio_chip',          '/dev/gpiochip0')
        self.declare_parameter('pps_gpio_line',           96)
        self.declare_parameter('pps_frame_id',            'gq7_pps')
        self.declare_parameter('pps_debounce_time_us',    0)
        self.declare_parameter('start_drain_ms',          50.0)
        self.declare_parameter('ext_time_max_offset_ms',  1.0)    # C2: Orin edge vs Teensy second
        self.declare_parameter('ext_time_max_latency_ms', 150.0)  # C3: GQ7 needs it < 250 ms after the edge
        self.declare_parameter('pps_gap_s',               1.5)    # no edge for this long = PPS stopped
        self.declare_parameter('gq7_clock_min_ms',        0.0)    # GQ7 clock check band on
        self.declare_parameter('gq7_clock_max_ms',        20.0)   #   receive - header (healthy ~3.4 ms)
        self.declare_parameter('gq7_clock_error_s',       5.0)    # W11 becomes ERROR after this long
        self.declare_parameter('time_valid_timeout_s',    3.0)    # W9a: time_valid expected within this
        self.declare_parameter('lock_warn_s',             10.0)   # W6: LOCK=0 for this long
        self.declare_parameter('wait_sync_timeout_s',     45.0)
        self.declare_parameter('wait_sync_settle_s',      10.0)   # PPS must run this long without a gap

        p = self.get_parameter
        self.serial_port          = p('serial_port').value
        self.baud_rate            = p('baud_rate').value
        self.trigger_gpio_chip    = p('trigger_gpio_chip').value
        self.trigger_gpio_line    = p('trigger_gpio_line').value
        self.trigger_frame_id     = p('trigger_frame_id').value
        self.debounce_time_us     = p('debounce_time_us').value
        self.trigger_event_clock  = p('trigger_event_clock').value
        self._hte = None   # _HteClock when the trigger line is stamped by HTE
        self.pps_gpio_chip        = p('pps_gpio_chip').value
        self.pps_gpio_line        = p('pps_gpio_line').value
        self.pps_frame_id         = p('pps_frame_id').value
        self.pps_debounce_time_us = p('pps_debounce_time_us').value
        self.start_drain_ms       = p('start_drain_ms').value
        self.max_offset_ns        = int(p('ext_time_max_offset_ms').value * 1e6)
        self.max_latency_ns       = int(p('ext_time_max_latency_ms').value * 1e6)
        self.pps_gap_ns           = int(p('pps_gap_s').value * NS)
        self.gq7_clock_min_ms     = p('gq7_clock_min_ms').value
        self.gq7_clock_max_ms     = p('gq7_clock_max_ms').value
        self.gq7_clock_error_s    = p('gq7_clock_error_s').value
        self.time_valid_timeout_s = p('time_valid_timeout_s').value
        self.lock_warn_s          = p('lock_warn_s').value
        self.wait_sync_timeout_s  = p('wait_sync_timeout_s').value
        self.wait_sync_settle_s   = p('wait_sync_settle_s').value

        # ── Serial ────────────────────────────────────────────────────────────
        self._serial_lock = threading.Lock()
        self._rx_buf      = b''
        self.serial_conn  = None
        self._init_serial()

        # ── Sync state (shared between PPS thread, monitor timer, callbacks) ──
        self._state_lock = threading.Lock()
        # Teensy, from the last LASTPPS reply
        self._reply_ok      = False
        self._last_pps_sec  = 0
        self._lock          = False
        self._age_ms        = -1
        self._offset_ns     = 0
        self._pps           = False
        self._fw_too_old    = False
        self._reply_fail_logged = False
        # Orin PPS edges
        self._last_edge_ns        = None   # Orin REALTIME of the last edge
        self._orin_minus_teensy   = 0
        self._last_c2_ok          = False
        self._last_published_sec  = None
        self._last_publish_ns     = None
        self._ext_time_published  = False
        self._skip_reason         = ''
        self._skipping            = False
        self._skip_count          = 0
        self._sent_since_gap      = False  # a /gq7/ext/time went out since the last PPS gap / time_sync change
        self._ever_sent           = False  # the GQ7 has been given the time at least once since node start
        self._pps_edge_count      = 0
        # GQ7
        self._time_sync_ok    = False
        self._last_pps_rcvd   = 255
        self._time_valid      = False
        self._gq7_d_ms        = deque(maxlen=5)
        self._gq7_clock_ok    = False
        self._gq7_clock_bad_since = None
        self._gq7_clock_error_logged = False
        self._gq7_seen_pps_ok = True
        self._time_valid_deadline = None   # W9 follow-up
        # Monitoring
        self._pps_stopped     = False
        self._edges_since     = None       # monotonic time PPS edges started arriving (W13 grace)
        self._lock_zero_since = None
        self._lock_warned     = False
        self._waiting_sync    = False      # suppress W11 while wait_sync_ready runs

        # ── State machine + session timing ────────────────────────────────────
        self._state              = NodeState.IDLE
        self.trigger_gpio_req    = None
        self.gpio_thread         = None
        self.gpio_ready_event    = threading.Event()
        self._start_cmd_mono_ns  = None
        self._stop_cmd_mono_ns   = None
        self._first_edge_mono_ns = None
        self._last_edge_mono_ns  = None

        # ── ROS interfaces ────────────────────────────────────────────────────
        # Reentrant group + MultiThreadedExecutor: wait_sync_ready waits for state that the
        # subscription and the monitor timer keep updating.
        self._cb_group = ReentrantCallbackGroup()
        self.timestamp_pub = self.create_publisher(Header, '/teensy/timestamps', 10)
        self._time_pub     = self.create_publisher(TimeReference, '/gq7/ext/time', 10)
        self._status_pub   = self.create_publisher(TeensyPtpStatus, '/teensy/ptp_status', 10)
        self.start_srv = self.create_service(
            Trigger, '/teensy/start_trigger', self.start_trigger_callback, callback_group=self._cb_group)
        self.stop_srv = self.create_service(
            Trigger, '/teensy/stop_trigger', self.stop_trigger_callback, callback_group=self._cb_group)
        self.wait_srv = self.create_service(
            Trigger, '/teensy/wait_sync_ready', self.wait_sync_ready_callback, callback_group=self._cb_group)
        self.reset_srv = self.create_service(
            Trigger, '/teensy/ptp_reset', self.ptp_reset_callback, callback_group=self._cb_group)
        self._sync_sub = self.create_subscription(
            MipSystemTimeSyncStatus, '/gq7/mip/system/time_sync_status',
            self._sync_status_cb, 10, callback_group=self._cb_group)
        self._monitor_timer = self.create_timer(1.0, self._monitor_tick, callback_group=self._cb_group)

        # ── PPS GPIO (always-on) ──────────────────────────────────────────────
        self._pps_gpio_req = None
        self._pps_stop_evt = threading.Event()
        if self._init_pps_gpio():
            self._pps_thread = threading.Thread(target=self._pps_gpio_loop, daemon=True)
            self._pps_thread.start()
        else:
            self.get_logger().error('PPS GPIO init failed — /gq7/ext/time will not publish')
            self._pps_thread = None

        # ── Trigger GPIO (always-on) ──────────────────────────────────────────
        self.gpio_ready_event.clear()
        if self._init_trigger_gpio():
            self.gpio_thread = threading.Thread(target=self._gpio_monitor_thread, daemon=True)
            self.gpio_thread.start()
            if not self.gpio_ready_event.wait(timeout=0.5):
                self.get_logger().warn('Trigger GPIO thread not ready at bootup')
        else:
            self.get_logger().error('Trigger GPIO init failed — timestamps will not publish')
            self.gpio_thread = None

        # Firmware check at startup (logs ERROR if too old)
        self._update_teensy_state()

        self.get_logger().info(
            f'Ready [{self._state.value}] | serial={self.serial_port} | '
            f'trigger={self.trigger_gpio_chip}:{self.trigger_gpio_line} | '
            f'pps={self.pps_gpio_chip}:{self.pps_gpio_line}')

    # ─────────────────────────────────────────────────────────────────────────
    # Serial
    # ─────────────────────────────────────────────────────────────────────────

    def _init_serial(self) -> None:
        try:
            self.serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=0.05)
            time.sleep(2)
            with self._serial_lock:
                self._log_pending_lines_locked()
            self.get_logger().info('Serial ready')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial open failed: {e}')
            sys.exit(1)

    def _read_line_locked(self, deadline: float) -> str | None:
        """Next complete line, or None at the deadline (monotonic s). Keeps partial lines buffered."""
        while True:
            i = self._rx_buf.find(b'\n')
            if i >= 0:
                line, self._rx_buf = self._rx_buf[:i], self._rx_buf[i + 1:]
                return line.decode('ascii', errors='replace').strip()
            if time.monotonic() >= deadline:
                return None
            chunk = self.serial_conn.read(self.serial_conn.in_waiting or 1)
            if chunk:
                self._rx_buf += chunk

    def _log_pending_lines_locked(self) -> None:
        """Log lines that arrived unsolicited (e.g. 'EVENT: Session Complete') instead of discarding them."""
        n = self.serial_conn.in_waiting
        if n:
            self._rx_buf += self.serial_conn.read(n)
        while b'\n' in self._rx_buf:
            line = self._read_line_locked(time.monotonic())
            if line:
                self.get_logger().info(f'Teensy: {line}')

    def _transact(self, cmd: str, prefixes: tuple, timeout_s: float = 0.5, log: bool = True) -> str | None:
        """Send cmd, return the first reply line starting with one of prefixes (None on timeout).
        Other lines received meanwhile are logged."""
        with self._serial_lock:
            if self.serial_conn is None:
                return None
            try:
                self._log_pending_lines_locked()
                self.serial_conn.write(f'{cmd}\n'.encode())
                self.serial_conn.flush()
                deadline = time.monotonic() + timeout_s
                while True:
                    line = self._read_line_locked(deadline)
                    if line is None:
                        return None
                    if line.startswith(prefixes):
                        if log:
                            self.get_logger().info(f'Teensy: {line}')
                        return line
                    if line:
                        self.get_logger().info(f'Teensy: {line}')
            except Exception as e:
                self.get_logger().error(f'Serial error sending {cmd}: {e}')
                return None

    def _query_lastpps(self):
        """LASTPPS → (sec, lock, age_ms, offset_ns, pps) or None. Sets _fw_too_old on an old-format reply."""
        line = self._transact('LASTPPS', ('LAST_PPS_HW',), timeout_s=0.3, log=False)
        if line is None:
            return None
        m = self._LASTPPS_RE.search(line)
        if m:
            return (int(m.group(1)), m.group(3) == '1', int(m.group(4)), int(m.group(5)), m.group(6) == '1')
        if self._LASTPPS_OLD_RE.search(line):
            if not self._fw_too_old:
                self.get_logger().error(
                    f'Teensy firmware too old: LASTPPS has no LOCK/PPS fields ({line!r}). '
                    'Flash tartan_rgbt_ws scripts/teensy/teensy.ino (6000d07 or later). '
                    '/gq7/ext/time will not be published.')
            self._fw_too_old = True
        return None

    def _update_teensy_state(self):
        """Query LASTPPS and store the result. Returns the parsed tuple or None. W4 on failure."""
        r = self._query_lastpps()
        with self._state_lock:
            self._reply_ok = r is not None
            if r is not None:
                self._last_pps_sec, self._lock, self._age_ms, self._offset_ns, self._pps = r
        if r is None and not self._fw_too_old:
            if not self._reply_fail_logged:   # W4, rate-limited: once per failure streak
                self.get_logger().error('W4 LASTPPS: no reply or garbled reply from the Teensy')
                self._reply_fail_logged = True
        elif r is not None and self._reply_fail_logged:
            self.get_logger().warn('LASTPPS replies again')
            self._reply_fail_logged = False
        return r

    # ─────────────────────────────────────────────────────────────────────────
    # GQ7 sync status (1 Hz)
    # ─────────────────────────────────────────────────────────────────────────

    def _sync_status_cb(self, msg: MipSystemTimeSyncStatus):
        recv_ns = _realtime_ns()
        h = msg.header.header.stamp   # MipHeader wraps a std_msgs/Header: GQ7 device time (timestamp_source 1)
        d_ms = (recv_ns - (h.sec * NS + h.nanosec)) / 1e6
        tv = msg.header.gps_timestamp.valid_flags.time_valid
        with self._state_lock:
            prev_sync = self._time_sync_ok
            prev_tv = self._time_valid
            self._time_sync_ok = msg.time_sync
            self._last_pps_rcvd = msg.last_pps_rcvd
            self._time_valid = tv
            self._gq7_d_ms.append(d_ms)
            if msg.time_sync != prev_sync:
                self._sent_since_gap = False
            waiting_tv = self._time_valid_deadline is not None
        if msg.time_sync != prev_sync:
            if msg.time_sync:
                self.get_logger().warn('W8 GQ7 time_sync False → True')
            else:
                self.get_logger().warn('W2 GQ7 time_sync True → False (GQ7 not locked to PPS)')
        if waiting_tv and tv:
            with self._state_lock:
                self._time_valid_deadline = None
            self.get_logger().warn('GQ7 accepted the time (time_valid)')
        elif tv != prev_tv and not tv:
            self.get_logger().warn('GQ7 time_valid True → False')

    def _gq7_clock_check(self):
        """GQ7 clock vs Orin (change 11): minimum of receive - header over the last 5 time_sync_status.
        Minimum, not median: delivery delays (driver or transport stalls, seen as bursts of status messages
        arriving 1-4 s late) only ever add to receive - header, while a wrong GQ7 clock shifts every sample."""
        with self._state_lock:
            vals = list(self._gq7_d_ms)
        if not vals:
            return float('nan'), False
        d = min(vals)
        return d, self.gq7_clock_min_ms <= d <= self.gq7_clock_max_ms

    # ─────────────────────────────────────────────────────────────────────────
    # PPS GPIO (always-on)
    # ─────────────────────────────────────────────────────────────────────────

    def _init_pps_gpio(self) -> bool:
        # Clock.REALTIME: the kernel stamps the edge with CLOCK_REALTIME in the interrupt handler, the same
        # clock as PTP time (phc2sys -O 0). No HTE/MONOTONIC fallback: those can't be compared with the second.
        try:
            settings = gpiod.LineSettings(
                direction=Direction.INPUT, edge_detection=Edge.RISING, event_clock=Clock.REALTIME,
                debounce_period=datetime.timedelta(microseconds=self.pps_debounce_time_us))
            self._pps_gpio_req = gpiod.request_lines(
                path=self.pps_gpio_chip, consumer='teensy-combined-pps',
                config={self.pps_gpio_line: settings})
            self.get_logger().info('PPS GPIO clock: REALTIME')
            return True
        except Exception as e:
            self.get_logger().error(f'PPS GPIO: Clock.REALTIME unavailable ({e}); /gq7/ext/time disabled')
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
            if not events:
                continue
            if len(events) > 1:
                self.get_logger().warn(f'W5 missed PPS edge: {len(events)} edges in one read; using the last')
            self._handle_pps_edge(events[-1].timestamp_ns)

        self.get_logger().info('PPS GPIO thread exiting')

    def _handle_pps_edge(self, orin_edge_ns: int):
        self._pps_edge_count += 1
        with self._state_lock:
            prev_edge = self._last_edge_ns
            self._last_edge_ns = orin_edge_ns
            was_stopped = self._pps_stopped
            self._pps_stopped = False
        if was_stopped or prev_edge is None:
            self._edges_since = time.monotonic()
        if was_stopped:
            gap = (orin_edge_ns - prev_edge) / NS if prev_edge else float('nan')
            self.get_logger().warn(f'W7 PPS resumed after {gap:.1f} s')
            with self._state_lock:
                self._sent_since_gap = False

        r = self._update_teensy_state()   # LASTPPS right after the edge
        reason = self._ext_time_checks(orin_edge_ns, r)
        if reason is None:
            sec = r[0]
            msg = TimeReference()
            msg.header.stamp    = Time(nanoseconds=orin_edge_ns).to_msg()   # Orin time of the edge
            msg.header.frame_id = self.pps_frame_id
            msg.time_ref        = Time(nanoseconds=sec * NS).to_msg()       # PTP second, nsec = 0
            msg.source          = 'gq7_pps'
            self._time_pub.publish(msg)
            with self._state_lock:
                first_after_gap = not self._sent_since_gap
                skipped = self._skip_count
                self._sent_since_gap = True
                self._ever_sent = True
                self._last_published_sec = sec
                self._last_publish_ns = orin_edge_ns
                self._ext_time_published = True
                self._skip_reason = ''
                self._skipping = False
                self._skip_count = 0
            if first_after_gap:
                with self._state_lock:
                    self._time_valid_deadline = time.monotonic() + self.time_valid_timeout_s
                self.get_logger().warn(
                    f'W9 time sent to GQ7 after a gap: sec={sec}, Orin - Teensy = '
                    f'{(orin_edge_ns - sec * NS) / 1e3:.1f} us'
                    + (f' ({skipped} edges skipped)' if skipped else ''))
        else:
            with self._state_lock:
                first_skip = not self._skipping
                self._skipping = True
                self._skip_count += 1
                self._ext_time_published = False
                self._skip_reason = reason
            if first_skip:
                self.get_logger().warn(f'W3 /gq7/ext/time skipped: {reason}')

    def _ext_time_checks(self, orin_edge_ns: int, r) -> str | None:
        """C1–C6. Returns None if the edge's second may be sent to the GQ7, else the reason."""
        if self._fw_too_old:
            return 'Teensy firmware too old'
        if r is None:
            return 'C5 no/garbled LASTPPS reply'
        sec, _lock, _age, _off, pps = r
        diff = orin_edge_ns - sec * NS
        with self._state_lock:
            self._orin_minus_teensy = diff
            self._last_c2_ok = abs(diff) < self.max_offset_ns
            last_sec, last_pub = self._last_published_sec, self._last_publish_ns
            synced = self._time_sync_ok
        if not pps:
            return 'C1 Teensy reports PPS=0'
        if abs(diff) >= self.max_offset_ns:
            return f'C2 Orin edge vs Teensy second off by {diff / 1e6:.3f} ms'
        latency = _realtime_ns() - orin_edge_ns
        if latency >= self.max_latency_ns:
            return f'C3 {latency / 1e6:.0f} ms after the edge (GQ7 window 250 ms)'
        if last_sec is not None and last_pub is not None and orin_edge_ns - last_pub < self.pps_gap_ns:
            if sec != last_sec + 1:
                return f'C4 second not +1 ({last_sec} → {sec})'
        if not synced:
            return 'C6 GQ7 time_sync False'
        return None

    # ─────────────────────────────────────────────────────────────────────────
    # Monitor (1 Hz): query with PPS off, diagnostics, transition warnings
    # ─────────────────────────────────────────────────────────────────────────

    def _monitor_tick(self):
        now_ns = _realtime_ns()
        with self._state_lock:
            last_edge = self._last_edge_ns
        edge_recent = last_edge is not None and now_ns - last_edge < self.pps_gap_ns

        if not edge_recent:
            self._update_teensy_state()          # PPS off: still record the Teensy state
            with self._state_lock:
                newly_stopped = not self._pps_stopped and last_edge is not None
                self._pps_stopped = last_edge is not None
                lock, age, off, pps, ok = self._lock, self._age_ms, self._offset_ns, self._pps, self._reply_ok
            if newly_stopped:
                cause = ('master gone (AGE > 3 s)' if age > 3000 else
                         'large OFFSET (step or frozen servo)' if abs(off) >= 10_000 else
                         'reset / relock pending' if not pps else 'unknown')
                self.get_logger().warn(
                    f'W1 PPS stopped: LOCK={int(lock)} AGE={age} OFFSET={off} PPS={int(pps)} '
                    f'reply_ok={ok} → {cause}')

        with self._state_lock:
            lock = self._lock
            last_pps_rcvd = self._last_pps_rcvd
            tv_deadline = self._time_valid_deadline
            waiting = self._waiting_sync

        # W6 / W10: LOCK=0 for long (PPS may still be fine)
        if not lock:
            if self._lock_zero_since is None:
                self._lock_zero_since = time.monotonic()
            elif not self._lock_warned and time.monotonic() - self._lock_zero_since > self.lock_warn_s:
                self.get_logger().warn(f'W6 Teensy LOCK=0 for > {self.lock_warn_s:.0f} s (servo problem? '
                                       'PTPRESET between recordings)')
                self._lock_warned = True
        else:
            if self._lock_warned:
                self.get_logger().warn('W10 Teensy LOCK=1 again')
            self._lock_zero_since, self._lock_warned = None, False

        # W11 / W12: GQ7 clock vs Orin. Only once the GQ7 has been given the time: right after a GQ7
        # power-up its clock is unset until the first /gq7/ext/time, which is normal startup, not an error.
        med, clk_ok = self._gq7_clock_check()
        with self._state_lock:
            ever_sent = self._ever_sent
        if not math.isnan(med) and ever_sent:
            if not clk_ok and not waiting:
                if self._gq7_clock_bad_since is None:
                    self._gq7_clock_bad_since = time.monotonic()
                    self.get_logger().warn(f'W11 GQ7 clock off: receive - header min {med:.1f} ms '
                                           f'(band {self.gq7_clock_min_ms:.0f}–{self.gq7_clock_max_ms:.0f} ms)')
                elif (not self._gq7_clock_error_logged and
                      time.monotonic() - self._gq7_clock_bad_since >= self.gq7_clock_error_s):
                    self.get_logger().error(f'W11 GQ7 clock off for {self.gq7_clock_error_s:.0f} s: '
                                            f'min {med:.1f} ms — GQ7 stamps in the bag are wrong')
                    self._gq7_clock_error_logged = True
            elif clk_ok and self._gq7_clock_bad_since is not None:
                self.get_logger().warn(f'W12 GQ7 clock back: min {med:.1f} ms')
                self._gq7_clock_bad_since, self._gq7_clock_error_logged = None, False
        with self._state_lock:
            self._gq7_clock_ok = clk_ok

        # W13 / W14: GQ7 not seeing PPS while the Orin is. Grace of 3 s after PPS (re)starts: the GQ7's
        # last_pps_rcvd lags by a status message.
        edges_steady = (edge_recent and self._edges_since is not None
                        and time.monotonic() - self._edges_since > 3.0)
        if edges_steady and last_pps_rcvd > 1 and self._gq7_seen_pps_ok:
            self.get_logger().warn(f'W13 GQ7 not seeing PPS: last_pps_rcvd={last_pps_rcvd} s while the Orin '
                                   'receives edges (Teensy → GQ7 wire or GPIO1 config)')
            self._gq7_seen_pps_ok = False
        elif last_pps_rcvd <= 1 and not self._gq7_seen_pps_ok:
            self.get_logger().warn('W14 GQ7 sees PPS again')
            self._gq7_seen_pps_ok = True

        # W9a: GQ7 did not accept the time
        if tv_deadline is not None and time.monotonic() > tv_deadline:
            with self._state_lock:
                self._time_valid_deadline = None
            self.get_logger().warn(f'W9a GQ7 did not accept the time: time_valid still False '
                                   f'{self.time_valid_timeout_s:.0f} s after W9')

        self._publish_status(now_ns, edge_recent, med, clk_ok)

    def _publish_status(self, now_ns, edge_recent, med, clk_ok):
        m = TeensyPtpStatus()
        m.header.stamp = Time(nanoseconds=now_ns).to_msg()
        with self._state_lock:
            m.reply_ok             = self._reply_ok
            m.last_pps_sec         = self._last_pps_sec
            m.lock                 = self._lock
            m.age_ms               = self._age_ms
            m.offset_ns            = self._offset_ns
            m.pps                  = self._pps
            m.orin_minus_teensy_ns = self._orin_minus_teensy
            m.ext_time_published   = self._ext_time_published and edge_recent
            m.skip_reason          = self._skip_reason if edge_recent else 'no PPS edge'
            m.gq7_time_sync        = self._time_sync_ok
            m.gq7_last_pps_rcvd    = min(self._last_pps_rcvd, 255)
            m.gq7_time_valid       = self._time_valid
        m.edge_recent         = edge_recent
        m.gq7_clock_offset_ms = med
        m.gq7_clock_ok        = clk_ok
        m.session_active      = self._state == NodeState.SESSION_ACTIVE
        self._status_pub.publish(m)

    def _sync_problems(self) -> list:
        """Conditions for a healthy timing chain (wait_sync_ready, start_trigger). Empty list = OK."""
        now_ns = _realtime_ns()
        with self._state_lock:
            s = dict(reply_ok=self._reply_ok, pps=self._pps, lock=self._lock, last_edge=self._last_edge_ns,
                     c2=self._last_c2_ok, sync=self._time_sync_ok, sent=self._sent_since_gap,
                     rcvd=self._last_pps_rcvd, tv=self._time_valid, too_old=self._fw_too_old,
                     diff=self._orin_minus_teensy)
        _med, clk_ok = self._gq7_clock_check()
        problems = []
        if s['too_old']:
            problems.append('Teensy firmware too old')
        if not s['reply_ok']:
            problems.append('no LASTPPS reply')
        elif not s['pps']:
            problems.append('Teensy PPS=0')
        if s['last_edge'] is None or now_ns - s['last_edge'] > 2 * NS:
            problems.append('no PPS edge on the Orin in the last 2 s')
        elif not s['c2']:
            problems.append(f'Orin edge vs Teensy second off by {s["diff"] / 1e6:.3f} ms')
        if not s['sync']:
            problems.append('GQ7 time_sync False')
        if not s['sent']:
            problems.append('no /gq7/ext/time sent since the last PPS gap / time_sync change')
        if s['rcvd'] > 1:
            problems.append(f'GQ7 last_pps_rcvd={s["rcvd"]} s')
        if not s['tv']:
            problems.append('GQ7 time_valid False')
        if not clk_ok:
            problems.append(f'GQ7 clock check failed (min receive - header {_med:.1f} ms)')
        return problems

    # ─────────────────────────────────────────────────────────────────────────
    # Trigger GPIO (always-on)
    # ─────────────────────────────────────────────────────────────────────────

    def _init_trigger_gpio(self) -> bool:
        # Edge.BOTH, keep rising events: with Edge.RISING alone the Tegra GPIO driver reports many falling
        # edges as rising (~48/s instead of 30). A debounce > 0 uses the Tegra hardware debounce, which works
        # in whole milliseconds and puts every stamp on a 1 ms grid.
        # HTE stamps the edge in hardware (no interrupt-latency jitter: ~1 µs vs ~40-100 µs std with REALTIME)
        # and is converted to REALTIME per edge (_HteClock). Falls back to REALTIME if HTE or the counter read
        # isn't available. HTE reports some spurious falling edges; only rising edges are used.
        if self.debounce_time_us > 0:
            self.get_logger().warn(f'Trigger debounce {self.debounce_time_us} us: Tegra debounces in 1 ms steps, '
                                   'trigger stamps will be quantized to 1 ms. Use 0.')
        debounce = datetime.timedelta(microseconds=self.debounce_time_us)
        if self.trigger_event_clock == 'hte':
            try:
                hte = _HteClock()
                settings = gpiod.LineSettings(
                    direction=Direction.INPUT, edge_detection=Edge.BOTH, event_clock=Clock.HTE,
                    debounce_period=debounce)
                self.trigger_gpio_req = gpiod.request_lines(
                    path=self.trigger_gpio_chip, consumer='teensy-combined-trigger',
                    config={self.trigger_gpio_line: settings})
                self._hte = hte
                self.get_logger().info(
                    'Trigger GPIO clock: HTE, converted to REALTIME per edge (both edges, rising kept; '
                    f'counter - MONOTONIC_RAW = {hte.counter_minus_raw_ns / 1e6:.3f} ms)')
                return True
            except Exception as e:
                self.get_logger().warn(f'Trigger HTE unavailable ({e}); using REALTIME stamps')
        elif self.trigger_event_clock != 'realtime':
            self.get_logger().warn(f'Unknown trigger_event_clock {self.trigger_event_clock!r}; using REALTIME')
        try:
            settings = gpiod.LineSettings(
                direction=Direction.INPUT, edge_detection=Edge.BOTH, event_clock=Clock.REALTIME,
                debounce_period=debounce)
            self.trigger_gpio_req = gpiod.request_lines(
                path=self.trigger_gpio_chip, consumer='teensy-combined-trigger',
                config={self.trigger_gpio_line: settings})
            self.get_logger().info('Trigger GPIO clock: REALTIME (both edges, rising kept)')
            return True
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
        rising = gpiod.EdgeEvent.Type.RISING_EDGE
        hte = self._hte
        next_offset_check = time.monotonic() + 60.0
        bad_stamp_warned = False

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

                to_rt = 0
                if hte is not None:
                    to_rt = hte.to_realtime_ns()
                    if time.monotonic() > next_offset_check:
                        # counter - MONOTONIC_RAW is constant; re-check it once a minute
                        next_offset_check = time.monotonic() + 60.0
                        old = hte.counter_minus_raw_ns
                        try:
                            new = hte.measure_offset()
                            if abs(new - old) > 2_000:
                                self.get_logger().warn(f'HTE counter offset changed by {(new - old) / 1e3:.1f} us')
                                hte.counter_minus_raw_ns = new
                        except RuntimeError as e:
                            self.get_logger().warn(f'HTE counter offset re-check failed: {e}')

                for event in events:
                    if event.event_type != rising:
                        continue
                    stamp_ns = event.timestamp_ns + to_rt   # CLOCK_REALTIME
                    if hte is not None and not bad_stamp_warned:
                        age = time.clock_gettime_ns(time.CLOCK_REALTIME) - stamp_ns
                        if not 0 <= age < NS:
                            bad_stamp_warned = True
                            self.get_logger().warn(f'HTE trigger stamp {age / 1e6:.3f} ms before now '
                                                   '(expected 0-1000 ms); check the HTE conversion')
                    hdr = Header()
                    hdr.stamp    = Time(nanoseconds=stamp_ns).to_msg()
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

    def _send_ptpreset(self) -> bool:
        resp = self._transact('PTPRESET', ('OK: PTP reset',), timeout_s=1.0)
        with self._state_lock:
            self._sent_since_gap = False
        return resp is not None

    def wait_sync_ready_callback(self, request, response):
        """Before recording: PTPRESET, then wait until the Teensy and the GQ7 are back in sync."""
        if self._state == NodeState.SESSION_ACTIVE:
            response.success = False
            response.message = 'Refused: session active (PTPRESET would step the Teensy clock)'
            return response
        with self._state_lock:
            self._waiting_sync = True
        try:
            t0 = time.monotonic()
            if not self._send_ptpreset():
                response.success = False
                response.message = 'PTPRESET: no reply from the Teensy'
                self.get_logger().error(f'wait_sync_ready failed: {response.message}')
                return response
            self.get_logger().info('wait_sync_ready: PTPRESET sent, waiting for Teensy relock and GQ7 sync')
            time.sleep(2.0)   # let PPS go off and the edge/LASTPPS state reflect the reset
            # Besides the sync conditions, PPS must have run without a gap for wait_sync_settle_s: right after
            # the relock the Teensy servo can still overshoot past the 10 us PPS gate (seen 2026-10-19: PPS
            # stopped again 2.6 s after a too-early "in sync").
            def problems_now():
                pr = self._sync_problems()
                since = self._edges_since
                steady = 0.0 if since is None else time.monotonic() - since
                if steady < self.wait_sync_settle_s:
                    pr.append(f'PPS steady for {steady:.0f} s (< {self.wait_sync_settle_s:.0f} s)')
                return pr
            problems = problems_now()
            while problems and time.monotonic() - t0 < self.wait_sync_timeout_s:
                time.sleep(0.5)
                problems = problems_now()
            dt = time.monotonic() - t0
            if problems:
                response.success = False
                response.message = f'Not in sync after {dt:.0f} s: ' + '; '.join(problems)
                self.get_logger().error(f'wait_sync_ready failed (recording blocked): {response.message}')
            else:
                response.success = True
                response.message = f'In sync after {dt:.1f} s'
                self.get_logger().info(f'wait_sync_ready: {response.message}')
            return response
        finally:
            with self._state_lock:
                self._waiting_sync = False

    def ptp_reset_callback(self, request, response):
        if self._state == NodeState.SESSION_ACTIVE:
            response.success = False
            response.message = 'Refused: session active (PTPRESET would step the Teensy clock)'
            return response
        ok = self._send_ptpreset()
        response.success = ok
        response.message = 'PTPRESET sent' if ok else 'PTPRESET: no reply from the Teensy'
        return response

    def start_trigger_callback(self, request, response):
        if self._state == NodeState.SESSION_ACTIVE:
            response.success = False
            response.message = 'Already running'
            return response

        if self.trigger_gpio_req is None:
            response.success = False
            response.message = 'Trigger GPIO not initialized'
            return response

        # Safety net (change 7): wait_sync_ready should already have ensured this before recording started.
        problems = self._sync_problems()
        if problems:
            response.success = False
            response.message = 'Timing not in sync: ' + '; '.join(problems)
            self.get_logger().error(f'start_trigger refused: {response.message}')
            return response

        self._reset_session_state()
        self._drain_pending_trigger_events(self.start_drain_ms)

        self._start_cmd_mono_ns = time.monotonic_ns()
        time.sleep(0.5)  # let recording initialize before marker fires
        resp = self._transact('SYNC_START', ('OK',), timeout_s=1.0)

        if not resp:
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
        self._transact('SYNC_STOP', ('OK',), timeout_s=1.0)
        time.sleep(MARKER_LEN / 30.0 + 0.1)  # wait for marker to complete
        with self._serial_lock:
            self._log_pending_lines_locked()   # 'EVENT: Session Complete. Pulses: N'

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
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        if rclpy.ok():   # on SIGINT rclpy's handler has already shut the context down
            rclpy.shutdown()


if __name__ == '__main__':
    main()
