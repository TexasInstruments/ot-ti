#!/usr/bin/env python3
"""
ot_thread_test.py — OpenThread OTBR (Leader) + FTD (Child) network test

Two-node Thread network test: an OTBR Leader running locally on this same
host (otbr-agent started manually and separately by the caller — this
script only talks to it via `sudo ot-ctl <cmd>`) and an FTD Child (a
serial OpenThread CLI device, e.g. the RCP+BLE Controller's Thread PTY
or a real UART).

Runs 4 phases per iteration:
  1. Network Setup       — reset both devices, leader forms a new Thread
                            network, child joins it
  2. Connectivity (Ping) — fetch each side's mesh-local IP, ping the
                            leader from the child
  3. Network Health      — collect device info + topology tables from
                            both sides, verify roles
  4. FTD Leave and Rejoin — child leaves the network and rejoins it

Usage:
    python3 ot_thread_test.py --child-port /dev/ttyACM0 --iterations 20
"""

import argparse
import re
import subprocess
import sys
import time

import serial

LEADER = 'OTBR_Leader'
CHILD  = 'FTD_Child'


# ---------------------------------------------------------------------------
# Console output helpers
# ---------------------------------------------------------------------------
def _ts() -> str:
    now = time.time()
    return time.strftime('%H:%M:%S', time.localtime(now)) + f'.{int(now * 1000) % 1000:03d}'


def log_info(device, msg):    print(f"[INFO]    [{device}] {msg}")
def log_warn(device, msg):    print(f"[WARN]    [{device}] {msg}")
def log_error(device, msg):   print(f"[ERROR]   [{device}] {msg}")
def log_success(device, msg): print(f"[SUCCESS] [{device}] {msg}")
def log_error_general(msg):   print(f"[ERROR] {msg}")
def log_request(device, msg): print(f"[{_ts()}] [{device}] CLI_REQUEST {msg}")


def log_response(device, text):
    for line in (text.splitlines() if text else ['']):
        print(f"[{_ts()}] [{device}] CLI_RESPONSE {line}")


def print_banner(title):
    bar = '=' * 70
    print(f"\n{bar}\n  {title}\n{bar}\n")


def print_full_results(results):
    print_banner("Full Results")
    for it, phase_name, ok in results:
        mark = 'PASS' if ok else 'FAIL'
        print(f"  [{mark}]  Iteration: {it}, {phase_name}")
    print()


def wait_for_state(get_state, expected: set, device: str, attempts: int = 15, interval: float = 2.0):
    """Poll get_state() until it returns a value in `expected`, logging a
    'Waiting for ...' line between attempts (matching OT test-log style)."""
    state = None
    for i in range(attempts):
        state = get_state()
        if state in expected:
            return state, True
        if i < attempts - 1:
            log_info(device, f"Waiting for {expected}, current: {state!r}")
            time.sleep(interval)
    return state, False


# ---------------------------------------------------------------------------
# Leader control — `sudo ot-ctl <cmd>` run locally. otbr-agent is started
# separately/manually by the caller; this just talks to it, with retry on
# a transient "connect session failed" from ot-ctl's own socket connect.
# ---------------------------------------------------------------------------
class LeaderCli:
    def __init__(self, retries=3, retry_delay=1.0, timeout=10.0):
        self.retries, self.retry_delay, self.timeout = retries, retry_delay, timeout

    def run(self, cmd: str, timeout: float = None) -> str:
        """Run one `sudo ot-ctl <cmd>` locally. Returns the stripped
        stdout+stderr text. Raises ConnectionError if the otbr-agent socket
        stays unavailable after all retries."""
        timeout = timeout if timeout is not None else self.timeout
        log_request(LEADER, f"sudo ot-ctl: {cmd}")
        ot_ctl_cmd = ['sudo', 'ot-ctl'] + cmd.split()

        output = ''
        for attempt in range(1, self.retries + 1):
            try:
                result = subprocess.run(ot_ctl_cmd, capture_output=True, text=True, timeout=timeout)
                output = (result.stdout + result.stderr).strip()
            except subprocess.TimeoutExpired:
                output = f"ot-ctl timeout after {timeout:.0f}s"
            log_response(LEADER, output)

            if 'connect session failed' in output.lower() or 'connection refused' in output.lower():
                if attempt < self.retries:
                    log_warn(LEADER, f"ot-ctl session unavailable (attempt {attempt}/{self.retries}), "
                                      f"retrying in {self.retry_delay:.0f}s...")
                    time.sleep(self.retry_delay)
                    continue
                log_error(LEADER, f"ot-ctl session unavailable after {self.retries} attempts: {output}")
                raise ConnectionError(output)
            return self._strip_done(output)
        raise ConnectionError(output)

    @staticmethod
    def _strip_done(output: str) -> str:
        """ot-ctl, like the underlying OpenThread CLI, appends a trailing
        'Done' (or 'Error ...') line to every command's output. Strip it so
        callers get just the value(s) — mirrors ChildCli._strip_framing."""
        lines = output.splitlines()
        if lines and (lines[-1] == 'Done' or lines[-1].startswith('Error')):
            lines = lines[:-1]
        return '\n'.join(lines)

    def try_run(self, cmd: str, timeout: float = None):
        """Like run(), but returns None instead of raising on failure."""
        try:
            return self.run(cmd, timeout)
        except ConnectionError:
            return None


# ---------------------------------------------------------------------------
# Child control — direct OpenThread CLI over a serial port. Every command
# is echoed by the CLI, followed by zero or more value lines and a
# terminating "Done" (or "Error ...").
# ---------------------------------------------------------------------------
class ChildCli:
    def __init__(self, port: str, baud: int, timeout: float = 5.0):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.timeout = timeout
        time.sleep(0.2)
        self.ser.reset_input_buffer()

    def run(self, cmd: str, timeout: float = None) -> list:
        """Send one CLI command and return its value lines (echo and the
        trailing Done/Error line stripped). Raises TimeoutError if no
        terminator arrives in time, or RuntimeError on a CLI "Error ...".
        """
        timeout = timeout if timeout is not None else self.timeout
        log_request(CHILD, f"Sending: {cmd}")
        self.ser.reset_input_buffer()
        self.ser.write((cmd + '\r\n').encode())

        lines, buf = [], ''
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            self.ser.timeout = max(0.05, min(0.2, remaining))
            chunk = self.ser.read(256).decode(errors='replace')
            if not chunk:
                continue
            buf += chunk
            while '\n' in buf:
                line, buf = buf.split('\n', 1)
                line = line.strip('\r\n> ').strip()
                if not line:
                    continue
                log_response(CHILD, line)
                lines.append(line)
                if self._is_terminator(line):
                    return self._strip_framing(lines, cmd)
        raise TimeoutError(f"Timeout waiting for CLI response to '{cmd}'")

    @staticmethod
    def _is_terminator(line: str) -> bool:
        """A command's final line is 'Done' or 'Error ...'. Match with
        endswith/contains rather than exact equality — a byte dropped on the
        wire during a heavier command (e.g. 'thread start') can merge a
        stray leftover character onto the same line as the terminator
        (observed as 'tDone' instead of 'thread start'/'Done' on two
        lines), which would otherwise never be recognized and just time out
        even though the command actually succeeded on the device."""
        return line == 'Done' or line.endswith('Done') or 'Error' in line

    @staticmethod
    def _strip_framing(lines, cmd):
        if lines and lines[0] == cmd:
            lines = lines[1:]
        if lines and ChildCli._is_terminator(lines[-1]):
            terminator, values = lines[-1], lines[:-1]
        else:
            terminator, values = 'Done', lines
        if 'Error' in terminator:
            raise RuntimeError(f"CLI error for '{cmd}': {terminator}")
        return values

    def value(self, cmd: str, timeout: float = None):
        vals = self.run(cmd, timeout)
        return vals[0] if vals else None

    def close(self):
        self.ser.close()


# ---------------------------------------------------------------------------
# Phase 1 — Network Setup
# ---------------------------------------------------------------------------
def phase1_network_setup(leader: LeaderCli, child: ChildCli) -> bool:
    print_banner("Phase 1 - Network Setup")

    log_info(LEADER, "Stopping any existing Thread state...")
    leader.try_run("thread stop")
    leader.try_run("ifconfig down")

    log_info(CHILD, "Stopping any existing Thread state...")
    child.run("thread stop")
    child.run("ifconfig down")

    log_info(LEADER, "Creating new Thread network...")
    try:
        leader.run("dataset init new")
        leader.run("dataset commit active")
        active_dataset = leader.run("dataset active -x")
        leader.run("ifconfig up")
        leader.run("thread start")
    except ConnectionError:
        log_error(LEADER, "Failed: dataset init new")
        return False

    if not active_dataset:
        log_error_general("Could not read leader's active dataset")
        return False

    log_info(CHILD, "Applying leader's network credentials...")
    child.run(f"dataset set active {active_dataset}")

    leader_state, leader_ok = wait_for_state(lambda: leader.try_run("state"), {'leader'}, LEADER)
    if not leader_ok:
        log_warn(LEADER, f"Expected leader, got: {leader_state}")

    log_info(CHILD, "Joining Thread network...")
    child.run("ifconfig up")
    child.run("thread start")
    child_state, child_ok = wait_for_state(lambda: child.value("state"), {'child', 'router'}, CHILD)
    if not child_ok:
        log_warn(CHILD, f"Expected child or router, got: {child_state}")

    return leader_ok and child_ok


def _ping_ok(ping_text: str) -> bool:
    return bool(re.search(r'\b[1-9]\d* packets received\b', ping_text)) or '0% packet loss' in ping_text


# ---------------------------------------------------------------------------
# Phase 2 — Connectivity (Ping) — both directions, each for `duration` seconds
# ---------------------------------------------------------------------------
def phase2_connectivity(leader: LeaderCli, child: ChildCli, duration: float = 10.0) -> bool:
    print_banner("Phase 2 - Connectivity (Ping)")

    # Routing to the leader may not have converged the instant the child
    # reports 'child'/'router', so give the mesh a moment before pinging.
    time.sleep(2.0)

    leader_mleid = leader.try_run("ipaddr mleid")
    child_mleid  = child.value("ipaddr mleid")

    if not leader_mleid or not child_mleid:
        log_error_general("Could not retrieve ML-EIDs")
        return False

    count = max(1, int(duration))

    log_info(CHILD, f"Pinging leader at {leader_mleid} for {duration:.0f}s ...")
    ping_lines = child.run(f"ping {leader_mleid} 8 {count} 1", timeout=duration + 5.0)
    child_ping_text = '\n'.join(ping_lines)
    child_ok = _ping_ok(child_ping_text)
    if child_ok:
        log_success(CHILD, "Ping to leader succeeded")
    else:
        log_error(CHILD, f"Ping to leader failed: {child_ping_text or '(no response)'}")

    log_info(LEADER, f"Pinging child at {child_mleid} for {duration:.0f}s ...")
    leader_ping_text = leader.try_run(f"ping {child_mleid} 8 {count} 1", timeout=duration + 5.0)
    leader_ok = bool(leader_ping_text) and _ping_ok(leader_ping_text)
    if leader_ok:
        log_success(LEADER, "Ping to child succeeded")
    else:
        log_error(LEADER, f"Ping to child failed: {leader_ping_text or '(no response)'}")

    return child_ok and leader_ok


# ---------------------------------------------------------------------------
# Phase 3 — Network Health
# ---------------------------------------------------------------------------
_INFO_CMDS = (("Version", "version"), ("ExtAddr", "extaddr"), ("EUI64", "eui64"), ("RLOC16", "rloc16"))


def phase3_network_health(leader: LeaderCli, child: ChildCli) -> bool:
    print_banner("Phase 3 - Network Health")

    log_info(LEADER, "--- Device info ---")
    for label, cmd in _INFO_CMDS:
        log_info(LEADER, f"{label:<7}: {leader.try_run(cmd)}")
    leader_state = leader.try_run("state")
    log_info(LEADER, f"State  : {leader_state}")

    log_info(LEADER, "--- Neighbor table ---")
    neighbor_table = leader.try_run("neighbor table")
    rows = 0
    if neighbor_table:
        for line in neighbor_table.splitlines():
            log_info(LEADER, f"  {line}")
        rows = max(0, len(neighbor_table.splitlines()) - 2)   # minus header/separator
    if rows == 0:
        log_warn(LEADER, "No neighbors in table")

    log_info(CHILD, "--- Device info ---")
    for label, cmd in _INFO_CMDS:
        log_info(CHILD, f"{label:<7}: {child.value(cmd)}")
    child_state = child.value("state")
    log_info(CHILD, f"State  : {child_state}")

    log_info(CHILD, "--- Neighbor table ---")
    nt_lines = child.run("neighbor table")
    for line in nt_lines:
        log_info(CHILD, f"  {line}")
    if len(nt_lines) <= 2:
        log_warn(CHILD, "No neighbors in table")

    log_info(LEADER, "--- Router table ---")
    leader.try_run("router table")

    leader_state = leader.try_run("state")
    child_state  = child.value("state")

    leader_ok = leader_state == 'leader'
    child_ok  = child_state in ('child', 'router')
    if not leader_ok:
        log_warn(LEADER, f"Expected leader, got: {leader_state}")
    if not child_ok:
        log_warn(CHILD, f"Expected child or router, got: {child_state}")

    return leader_ok and child_ok


# ---------------------------------------------------------------------------
# Phase 4 — FTD Leave and Rejoin
# ---------------------------------------------------------------------------
def phase4_leave_rejoin(child: ChildCli) -> bool:
    print_banner("Phase 4 - FTD Leave and Rejoin")

    log_info(CHILD, "Leaving network (thread stop + ifconfig down)...")
    child.run("thread stop")
    child.run("ifconfig down")
    state = child.value("state")
    if state == 'disabled':
        log_success(CHILD, "FTD is disabled (left network)")
        left_ok = True
    else:
        log_warn(CHILD, f"Expected disabled, got: {state}")
        left_ok = False

    log_info(CHILD, "Rejoining network (ifconfig up + thread start)...")
    child.run("ifconfig up")
    child.run("thread start")

    _, rejoin_ok = wait_for_state(lambda: child.value("state"), {'child', 'router'}, CHILD)

    return left_ok and rejoin_ok


def _make_phases(ping_duration: float):
    return [
        ("Phase 1 - Network Setup",        lambda leader, child: phase1_network_setup(leader, child)),
        ("Phase 2 - Connectivity (Ping)",  lambda leader, child: phase2_connectivity(leader, child, ping_duration)),
        ("Phase 3 - Network Health",       lambda leader, child: phase3_network_health(leader, child)),
        ("Phase 4 - FTD Leave and Rejoin", lambda leader, child: phase4_leave_rejoin(child)),
    ]


def main() -> int:
    ap = argparse.ArgumentParser(
        description="OpenThread OTBR (Leader, local) + FTD (Child) network test",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument('--child-port', required=True,
                     help='Serial device for the FTD child, e.g. /dev/ttyACM0')
    ap.add_argument('--child-baud', type=int, default=921600, help='Baud rate for the child port')
    ap.add_argument('--iterations', '-n', type=int, default=1,
                     help='Number of times to repeat the full 4-phase suite')
    ap.add_argument('--ping-duration', type=float, default=10.0,
                     help='Ping duration in seconds, applied to both directions')
    args = ap.parse_args()

    leader = LeaderCli()
    phases = _make_phases(args.ping_duration)

    try:
        child = ChildCli(args.child_port, args.child_baud)
    except serial.SerialException as e:
        print(f"ERROR: could not open {args.child_port}: {e}")
        return 1
    log_info(CHILD, f"Connected to {args.child_port} at {args.child_baud} baud")

    results = []
    try:
        for it in range(1, args.iterations + 1):
            print_banner(f"OpenThread OTBR + FTD Testing - Iteration {it}")
            for phase_name, phase_fn in phases:
                try:
                    ok = phase_fn(leader, child)
                except Exception as e:
                    log_error_general(f"Unhandled exception in {phase_name}: {e}")
                    ok = False
                results.append((it, phase_name, ok))
                print_full_results(results)
    finally:
        child.close()
        log_info(CHILD, "Disconnected")

    passed = sum(1 for _, _, ok in results if ok)
    return 0 if passed == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
