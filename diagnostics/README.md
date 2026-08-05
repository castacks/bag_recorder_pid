# Diagnostics

This directory stores diagnostic-specific scripts, utilities, and nodes for
deployment. This includes:
1. Recording raw serial and ethernet traffic
2. Saving terminal printout logs
3. Reporting port diagnostics (port voltages, among other diagnostics)
4. Reports on system setup for repeatability (What sensors are connected to what ports)
5. Saving master configs used during deployment
6. (Possibly) saving commit hash's or perhaps diff or copy of files used during deployment

Currently implemented: raw traffic capture (ethernet, CAN, serial), launched
and killed alongside a bag recording session via `routines/diagnostics.py`'s
`pre_logger`/`post_logger` hooks (see `config/tartandriver.yaml`).

## `log_<name>.sh` scripts

One small script per capture source (`log_velodyne.sh`, `log_multisense.sh`,
`log_actuation.sh`, `log_thermal.sh`).

- **In**: `$1` = directory to write into (defaults to `tmp` if run standalone).
  Source-specific settings (IPs, interface, device path) are top-of-file vars,
  overridable via env (e.g. `VELODYNE1_IP`, `CAN_INTERFACE`, `THERMAL_DEV`).
- **Out**: one capture file per source in that directory (`velodyne.pcap`,
  `actuation.can`, `thermal.raw`, ...). Runs in the foreground until killed
  (SIGINT, then SIGKILL if it doesn't exit).

## `scripts.yaml`

Single source of truth mapping a logical `name` (`velodyne`, `actuation`, ...)
to its script filename. Read by `routines.diagnostics` — nothing else should
hardcode this mapping.

## `remote_log.sh`

Generic SSH wrapper used when a capture source is physically connected to a
different machine than the one running `atvbag`.

- **In**: `<local_log_dir> <host> <name> <script_name>` — `host`/`name` are
  only used for logging/temp-dir naming; `script_name` is the already-resolved
  filename to run on the remote side (resolution happens in
  `routines.diagnostics`, not here).
- **Behavior**: SSHes to `host`, backgrounds `${REMOTE_DIAG_DIR}/<script_name>`
  there in its own session, then blocks in the foreground. On INT/TERM: kills
  the remote process (group), `rsync`s back whatever it wrote into
  `local_log_dir`, then removes the remote temp dir. Idempotent — safe if
  signaled more than once.
- **Env vars**: `REMOTE_TMP_ROOT` (default `/tmp/tartandriver_diagnostics`),
  `REMOTE_DIAG_DIR` (default `~/tartandriver_ws/src/core/bag_recorder_pid/diagnostics`
  — where this same repo is expected to live on the remote host).
- **Requires**: passwordless SSH to `host` already set up.

## `routines/diagnostics.py`

The `bagger.py` hook interface. Both are plain functions with kwargs
auto-filtered/merged from `tartandriver.yaml`'s `common.args` (which is where
`output`, the bag's own output directory, comes from).

- **`start_log(output, name, host=None)`** — `pre_logger` routine. Looks up
  `name` in `scripts.yaml`. If `host` is omitted, runs the script locally; if
  given, runs it via `remote_log.sh` on that host instead. Writes into
  `<output>/diagnostics/`. Tracks the launched process by `name` for
  `stop_log` to find later.
- **`stop_log(name)`** — `post_logger` routine. Looks up the process started
  under `name`, sends SIGINT (escalates to SIGKILL after 5s), and waits for it
  to exit. No `host` needed — it just signals whatever local process
  `start_log` launched (local capture, or the local `remote_log.sh` wrapper).

To enable/disable a source for a given run, comment out its matching
`start_log`/`stop_log` row pair in `tartandriver.yaml`.
