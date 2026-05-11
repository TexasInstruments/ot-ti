#!/usr/bin/env python3
"""
check_spinel_acks.py — detect unacknowledged/slow spinel requests in a
spinel_bridge.py --verbose log.

Parses every `>> PTY [thread]` (host -> RCP request) and `<< PHY [thread]`
(RCP -> host reply/notification) line, decodes the Spinel header byte
(NLI/TID) and the CMD/KEY fields, and pairs each request with its reply by
TID (TID 0 is reserved for unsolicited notifications and is never a reply
to a request, so it's tracked separately).

Flags two situations that otbr-agent's fixed 2000ms RadioSpinel timeout
(kMaxWaitTime) turns fatal (HandleRcpTimeout -> DieNow):
  - UNACKED  — a request's TID never got a matching reply before the log
               ends (or before that TID was reused by a later request).
  - SLOW     — a request got a reply, but after more than --slow-ms
               (default 1500ms), close enough to the 2000ms cliff to be
               worth flagging before it actually times out.

Usage:
    python3 check_spinel_acks.py /tmp/bridgelog1.log
    python3 check_spinel_acks.py /tmp/bridgelog1.log --slow-ms 1000
"""

import argparse
import re
import sys

_LINE_RE = re.compile(
    r'^(?P<ts>\d{2}:\d{2}:\d{2}) \[DEBUG\] (?P<dir>>> PTY|<< PHY) \[thread\].*?bytes: (?P<hex>[0-9a-f ]+)$')

_CMD_NAMES = {1: 'RESET', 2: 'GET', 3: 'SET', 4: 'INSERT', 5: 'REMOVE',
              6: 'IS', 7: 'INSERTED', 8: 'REMOVED'}

# Spinel property keys worth calling out by name — anything needing actual
# radio time to complete (unlike simple NCP-metadata GETs) is what's
# vulnerable to BLE/DMM radio contention.
_KEY_NAMES = {0x71: 'STREAM_RAW', 0x20: 'PHY_ENABLED', 0x24: 'MAC_15_4_PANID',
              0x25: 'MAC_15_4_SADDR', 0x22: 'PHY_RX_SENSITIVITY'}

_RADIO_TIME_KEYS = {0x71}   # STREAM_RAW: raw 802.15.4 TX — needs the radio


def _hms_to_seconds(ts: str) -> float:
    h, m, s = ts.split(':')
    return int(h) * 3600 + int(m) * 60 + int(s)


def _parse_frame(hexstr: str):
    """Returns (tid, cmd, key) from a raw HDLC frame's hex byte string, or
    None if the frame is too short to contain a header+cmd+key."""
    data = bytes.fromhex(hexstr.replace(' ', ''))
    if len(data) < 4 or data[0] != 0x7E:
        return None
    header = data[1]
    tid = header & 0x0F
    cmd = data[2] if len(data) > 2 else None
    key = data[3] if (cmd is not None and cmd != 1 and len(data) > 3) else None
    return tid, cmd, key


def analyze(log_path: str, slow_ms: float):
    events = []
    with open(log_path, 'r', errors='replace') as f:
        for line in f:
            m = _LINE_RE.match(line.strip())
            if not m:
                continue
            parsed = _parse_frame(m.group('hex'))
            if not parsed:
                continue
            tid, cmd, key = parsed
            events.append({
                'ts': _hms_to_seconds(m.group('ts')),
                'ts_str': m.group('ts'),
                'dir': 'sent' if m.group('dir') == '>> PTY' else 'recv',
                'tid': tid, 'cmd': cmd, 'key': key,
            })

    if not events:
        print(f"No '[thread]' PTY/PHY frames found in {log_path} — "
              f"is this a --verbose bridge log?")
        return 1

    # Pending requests by TID (TID 0 excluded — reserved for unsolicited).
    pending = {}
    unacked, slow = [], []

    for ev in events:
        if ev['tid'] == 0:
            continue   # unsolicited notification, not a request/reply pair
        if ev['dir'] == 'sent':
            if ev['tid'] in pending:
                # Previous request with this TID was never acked before
                # this TID got reused — that's an unacked request too.
                unacked.append(pending[ev['tid']])
            pending[ev['tid']] = ev
        else:  # recv
            req = pending.pop(ev['tid'], None)
            if req is not None:
                latency_ms = (ev['ts'] - req['ts']) * 1000.0
                if latency_ms > slow_ms:
                    slow.append((req, ev, latency_ms))

    unacked.extend(pending.values())   # anything still pending at EOF

    def describe(ev):
        cmd_name = _CMD_NAMES.get(ev['cmd'], f"cmd={ev['cmd']}")
        key_name = _KEY_NAMES.get(ev['key'], f"key=0x{ev['key']:02x}" if ev['key'] is not None else '')
        flag = '  [RADIO-TIME]' if ev['key'] in _RADIO_TIME_KEYS else ''
        return f"tid={ev['tid']} {cmd_name} {key_name}{flag}"

    print(f"Parsed {len(events)} thread frames from {log_path}\n")

    if unacked:
        print(f"UNACKED requests ({len(unacked)}) — never got a reply:")
        for req in sorted(unacked, key=lambda e: e['ts']):
            print(f"  [{req['ts_str']}] {describe(req)}")
    else:
        print("No unacked requests found.")

    print()
    if slow:
        print(f"SLOW replies ({len(slow)}) — took > {slow_ms:.0f}ms "
              f"(2000ms is otbr-agent's fatal cutoff):")
        for req, resp, latency_ms in sorted(slow, key=lambda t: -t[2]):
            print(f"  [{req['ts_str']} -> {resp['ts_str']}] "
                  f"{latency_ms:.0f}ms  {describe(req)}")
    else:
        print(f"No replies slower than {slow_ms:.0f}ms found.")

    return 1 if unacked else 0


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Detect unacknowledged/slow spinel requests in a spinel_bridge.py --verbose log",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument('log_path', help='Path to the bridge log (run with --verbose | tee <file>)')
    ap.add_argument('--slow-ms', type=float, default=1500.0,
                     help='Flag replies slower than this many milliseconds')
    args = ap.parse_args()
    return analyze(args.log_path, args.slow_ms)


if __name__ == '__main__':
    sys.exit(main())
