"""
Generic pre/post logger routines for launching and killing arbitrary bash
diagnostic scripts (tcpdump, candump, serial taps, etc.) alongside a bag
recording session. Scripts live in bag_recorder_pid/diagnostics/.
"""

import os
import signal
import subprocess

import yaml

_DIAG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "diagnostics")

# Single source of truth for name -> script, shared with remote_log.sh
with open(os.path.join(_DIAG_DIR, "scripts.yaml")) as f:
    SCRIPTS = yaml.safe_load(f)

# Populated by start_log, consumed by stop_log. Safe as module-level state
# since pre_logger, run_node, and post_logger all run in one process.
_procs = {}

def start_log(output, name, host=None, container=None):
    """
    container is only meaningful when host is set: it's the docker container
    to reach the script inside of on that remote host. Locally, no container
    hop is needed since this process already runs in the same environment as
    whatever it's logging.
    """
    log_dir = os.path.join(output, "diagnostics")
    os.makedirs(log_dir, exist_ok=True)
    script_name = SCRIPTS[name]
    if host:
        script = os.path.join(_DIAG_DIR, "remote_log.sh")
        cmd = [script, log_dir, host, name, script_name, container or ""]
    else:
        script = os.path.join(_DIAG_DIR, script_name)
        cmd = [script, log_dir]
    proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
    _procs[name] = proc
    where = f" on {host}" if host else ""
    if container:
        where += f" (container {container})"
    print(f"[diagnostics] started {name}{where} (pid {proc.pid}) -> {log_dir}")

def stop_log(name):
    proc = _procs.pop(name, None)
    if proc is None:
        print(f"[diagnostics] no running process for {name}")
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        proc.wait()
    except ProcessLookupError:
        pass
    print(f"[diagnostics] stopped {name}")
