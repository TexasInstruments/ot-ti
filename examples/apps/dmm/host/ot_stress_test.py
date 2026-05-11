#!/usr/bin/env python3
"""
ot_stress_test.py — Stress test for the OpenThread data path and the
child's UART/CLI link.

Joins the child to a locally-running OTBR leader (reusing the same
setup as ot_thread_test.py — leader forms a fresh network, its dataset
is pushed to the child), then hammers two independent things:

  1. UART/CLI stress — many rapid-fire, back-to-back CLI commands sent
     to the child, checking every response comes back well-formed and
     within a short timeout (catches dropped/garbled bytes on the
     serial link, e.g. the framing corruption that showed up on
     'thread start' during network join).
  2. Data path stress — one large burst of ping traffic across the
     Thread mesh with a configurable payload size and interval,
     checking packet loss stays under a threshold.

Usage:
    python3 ot_stress_test.py --child-port /dev/ttyACM0 \
        --cli-count 200 --ping-count 200 --ping-size 100
"""

import argparse
import re
import sys
import time

from ot_thread_test import (
    CHILD,
    LEADER,
    ChildCli,
    LeaderCli,
    log_error,
    log_error_general,
    log_info,
    log_success,
    phase1_network_setup,
    print_banner,
)

_PING_SUMMARY_RE = re.compile(
    r'(\d+) packets transmitted, (\d+) packets received\. Packet loss = ([\d.]+)%')


def _parse_ping_summary(text: str):
    m = _PING_SUMMARY_RE.search(text)
    if not m:
        return None
    return int(m.group(1)), int(m.group(2)), float(m.group(3))


# ---------------------------------------------------------------------------
# UART/CLI stress — rapid-fire commands over the child's serial link
# ---------------------------------------------------------------------------
def uart_stress(child: ChildCli, count: int) -> bool:
    print_banner("UART/CLI Stress")

    errors, latencies = 0, []
    report_every = max(1, count // 10)
    for i in range(1, count + 1):
        t0 = time.monotonic()
        try:
            child.run("state", timeout=2.0)
        except Exception as e:
            errors += 1
            log_error(CHILD, f"[{i}/{count}] command failed: {e}")
            continue
        latencies.append(time.monotonic() - t0)
        if i % report_every == 0:
            log_info(CHILD, f"[{i}/{count}] ok")

    if latencies:
        log_info(CHILD, f"round-trip avg={sum(latencies) / len(latencies) * 1000:.1f}ms "
                         f"min={min(latencies) * 1000:.1f}ms max={max(latencies) * 1000:.1f}ms")

    ok = errors == 0
    if ok:
        log_success(CHILD, f"UART stress passed: {count}/{count} commands OK")
    else:
        log_error(CHILD, f"UART stress FAILED: {errors}/{count} commands failed "
                          f"({100.0 * errors / count:.1f}%)")
    return ok


# ---------------------------------------------------------------------------
# Data path stress — one large ping burst across the Thread mesh
# ---------------------------------------------------------------------------
def data_path_stress(leader: LeaderCli, child: ChildCli, count: int, size: int,
                      interval: float, max_loss_pct: float) -> bool:
    print_banner("Data Path Stress (Ping)")

    leader_mleid = leader.try_run("ipaddr mleid")
    if not leader_mleid:
        log_error_general("Could not retrieve leader ML-EID")
        return False

    duration = count * interval
    log_info(CHILD, f"Pinging leader with {count} x {size}B packets "
                     f"(~{duration:.0f}s, interval={interval:g}s) ...")
    try:
        lines = child.run(f"ping {leader_mleid} {size} {count} {interval:g}",
                           timeout=duration + 10.0)
    except Exception as e:
        log_error(CHILD, f"Ping burst failed: {e}")
        return False

    summary = _parse_ping_summary('\n'.join(lines))
    if not summary:
        log_error(CHILD, f"Could not parse ping summary: {lines}")
        return False

    sent, received, loss_pct = summary
    log_info(CHILD, f"{sent} sent, {received} received, {loss_pct:.1f}% loss")

    ok = loss_pct <= max_loss_pct
    if ok:
        log_success(CHILD, f"Data path stress passed ({loss_pct:.1f}% <= {max_loss_pct:.1f}% threshold)")
    else:
        log_error(CHILD, f"Data path stress FAILED ({loss_pct:.1f}% > {max_loss_pct:.1f}% threshold)")
    return ok


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Stress test for the OpenThread data path and child UART/CLI link",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument('--child-port', required=True,
                     help='Serial device for the FTD child, e.g. /dev/ttyACM0')
    ap.add_argument('--child-baud', type=int, default=921600, help='Baud rate for the child port')
    ap.add_argument('--cli-count', type=int, default=200,
                     help='Number of rapid-fire CLI commands for the UART stress test')
    ap.add_argument('--ping-count', type=int, default=200,
                     help='Number of ping packets for the data path stress test')
    ap.add_argument('--ping-size', type=int, default=64, help='Ping payload size in bytes')
    ap.add_argument('--ping-interval', type=float, default=0.1,
                     help='Interval between pings in seconds')
    ap.add_argument('--max-loss-pct', type=float, default=1.0,
                     help='Maximum acceptable ping packet loss percentage')
    args = ap.parse_args()

    leader = LeaderCli()
    try:
        child = ChildCli(args.child_port, args.child_baud)
    except Exception as e:
        print(f"ERROR: could not open {args.child_port}: {e}")
        return 1
    log_info(CHILD, f"Connected to {args.child_port} at {args.child_baud} baud")

    ok = True
    try:
        if not phase1_network_setup(leader, child):
            log_error_general("Network setup failed — aborting stress test")
            return 1

        ok = uart_stress(child, args.cli_count) and ok
        ok = data_path_stress(leader, child, args.ping_count, args.ping_size,
                               args.ping_interval, args.max_loss_pct) and ok
    finally:
        child.close()
        log_info(CHILD, "Disconnected")

    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
