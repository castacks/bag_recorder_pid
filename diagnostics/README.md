# Diagnostics

This directory stores diagnostic-specific scripts, utilities, and nodes for
deployment. This includes:
1. Recording raw serial and ethernet traffic
2. Saving terminal printout logs
3. Reporting port diagnostics (port voltages, among other diagnostics)
4. Reports on system setup for repeatability (What sensors are connected to what ports)
5. Saving master configs used during deployment
6. (Possibly) saving commit hash's or perhaps diff or copy of files used during deployment

Currently implemented: raw traffic capture (ethernet, CAN, serial), tmux pane
logs, USB power/enumeration events, a port-connection snapshot, and a copy of
the deploy config — launched and killed alongside a bag recording session via
`routines/diagnostics.py`'s `pre_logger`/`post_logger` hooks (see
`config/tartandriver.yaml`).

> **Known gap:** `udevadm` is not installed in the `tartandriver` container
> image. Anything run locally (i.e. in-container) therefore cannot capture
> udev events or identify serial devices — `log_usb_power.sh` skips its
> udevadm stream and `log_connections.sh` reports its serial devices as
> `(unidentified)`, both loudly on stderr. Bare hosts (e.g. the desktop) are
> unaffected. Fix is to add the `udev` package to `docker/3_deps.dockerfile`.

## `log_<name>.sh` scripts

One small script per capture source (`log_velodyne.sh`, `log_multisense.sh`,
`log_actuation.sh`, `log_thermal.sh`, `log_tmux.sh`, `log_usb_power.sh`,
`log_connections.sh`, `log_deploy_config.sh`).

- **In**: `$1` = directory to write into (defaults to `tmp` if run standalone).
  `$2` = the routine `name` from the config, used as the filename prefix (see
  below); defaults to the source's own name when run standalone.
  Source-specific settings (IPs, interface, device path) are top-of-file vars,
  overridable via env (e.g. `VELODYNE1_IP`, `CAN_INTERFACE`, `THERMAL_DEV_LEFT`).
- **Out**: capture file(s) in that directory. Runs in the foreground until
  killed (SIGINT, then SIGKILL if it doesn't exit), except the one-shot
  snapshots (`log_connections.sh`, `log_deploy_config.sh`) which write once
  and exit.

### Filename convention

The routine `name` from the config leads every filename, and a suffix is added
only when one script writes more than one file:

| routine `name` | files written |
| --- | --- |
| `velodyne` | `velodyne.pcap` |
| `connections_laptop` | `connections_laptop.log` |
| `usb_power_desktop` | `usb_power_desktop_dmesg.log`, `usb_power_desktop_udevadm.log` |
| `tmux_laptop` | `tmux_laptop_<pane>.log`, one per tmux pane |
| `thermal` | `thermal_{left,right}_{ffc,video}.raw` |

This is what keeps a local and a remote copy of the same source (e.g.
`connections_laptop` vs `connections_desktop`) from overwriting each other in
the shared `<output>/diagnostics/` directory.

## Name -> script mapping

The `SCRIPTS` dict at the top of `routines/diagnostics.py` maps a logical
`name` (`velodyne`, `usb_power_laptop`, ...) to its script filename. Two names
may point at the same script — that's how `tmux_laptop`/`tmux_desktop` both
run `log_tmux.sh` while writing distinctly-prefixed files.

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

> **The remote host runs its _own_ copy of these scripts**, from
> `REMOTE_DIAG_DIR` — not the copy on the machine running `atvbag`. If the two
> drift apart, remote captures silently behave like the older version: this
> bit us once already, when a stale remote checkout ignored the `$2` prefix
> argument and wrote unprefixed filenames (`connections.log` instead of
> `connections_desktop.log`), which then looked like a naming bug in the local
> code. **After editing any `log_*.sh`, push it to every host that runs it:**
>
> ```bash
> rsync -a --include='*.sh' --exclude='*' \
>     src/core/bag_recorder_pid/diagnostics/ \
>     atv@192.168.3.30:'~/tartandriver_ws/src/core/bag_recorder_pid/diagnostics/'
> ```

## `routines/diagnostics.py`

The `bagger.py` hook interface. Both are plain functions with kwargs
auto-filtered/merged from `tartandriver.yaml`'s `common.args` (which is where
`output`, the bag's own output directory, comes from).

- **`start_log(output, name, host=None, container=None)`** — `pre_logger`
  routine. Looks up `name` in `SCRIPTS` and passes it to the script as `$2` so
  it can prefix its output files. If `host` is omitted, runs the script
  locally; if given, runs it via `remote_log.sh` on that host instead, with
  `container` optionally naming a docker container to `docker exec` into
  there. Writes into `<output>/diagnostics/`. Tracks the launched process by
  `name` for `stop_log` to find later.
- **`stop_log(name)`** — `post_logger` routine. Looks up the process started
  under `name`, sends SIGINT (escalates to SIGKILL after 5s), and waits for it
  to exit. No `host` needed — it just signals whatever local process
  `start_log` launched (local capture, or the local `remote_log.sh` wrapper).

To enable/disable a source for a given run, comment out its matching
`start_log`/`stop_log` row pair in `tartandriver.yaml`.
