#!/usr/bin/env python3
"""Read S3 tactile SHM and report what a continuously-held close would trip.

Observation only: no ROS, command topic, actuator handle, hold, or dataset.
"""

import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
import time

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tactile_limit import HandLimit, Intent, positive_limit


def observe(capture, limit, seconds, max_age, healthy):
    guards = {s: HandLimit(s, limit, max_age_s=max_age, healthy_status={healthy})
              for s in ("left", "right")}
    stats = {s: dict(distinct_packets=0, missing_reads=0, fault_ticks=0, max_reported_normal_n=None,
                     first_trip=None) for s in guards}
    last = dict.fromkeys(guards)
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        capture.poll()
        now, tick = time.time(), time.monotonic()
        for side, guard in guards.items():
            row = capture.latest_touch(side)
            entry = stats[side]
            if row is None:
                entry["missing_reads"] += 1
            else:
                identity = tuple(row.stamp_s)
                if identity != last[side]:
                    entry["distinct_packets"] += 1
                    last[side] = identity
                peak = float(max(row.normal_n))
                previous = entry["max_reported_normal_n"]
                entry["max_reported_normal_n"] = peak if previous is None else max(previous, peak)
            # The observation process is not the motor owner. It cannot ACK a
            # stop, even in a trace where the threshold is genuinely exceeded.
            decision = guard.tick(Intent.CLOSE, row, now_s=now, monotonic_s=tick,
                                  cancel_and_hold=lambda _side: False)
            if decision.trip:
                entry["first_trip"] = asdict(decision.trip)
            if decision.sample_issue not in (None, "force_limit"):
                entry["fault_ticks"] += 1
        time.sleep(0.01)
    return {"mode": "read_only_no_motor_commands", "limit_n": limit, "hands": stats}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--limit-n", required=True, type=positive_limit, help="Diagnostic threshold in newtons.")
    parser.add_argument("--seconds", type=float, default=3, help="Observe for 1 through 60 seconds.")
    parser.add_argument("--max-age-ms", type=float, default=50,
                        help="Per-point freshness deadline for this observation, not a hardware stopping guarantee.")
    args = parser.parse_args()
    if not 1 <= args.seconds <= 60 or not math.isfinite(args.max_age_ms) or args.max_age_ms <= 0:
        parser.error("seconds must be in [1,60] and max-age-ms must be finite and positive")
    from hand_sensors import AGILE_STATUS_NORMAL, HandSensorCapture

    capture = HandSensorCapture(with_force=False, with_tactile=True, buffer_size=2)
    try:
        result = observe(capture, args.limit_n, args.seconds, args.max_age_ms / 1000, AGILE_STATUS_NORMAL)
    finally:
        capture.close()
    print(json.dumps(result, allow_nan=False), flush=True)
    if any(h["distinct_packets"] < 2 or h["fault_ticks"] for h in result["hands"].values()):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
