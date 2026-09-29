#!/usr/bin/env python3
"""Look for meaning in the Yaskawa incremental payload bits that are not decoded yet.

Reads CSVs from record_monitor.py or the monitor's "Download CSV" (they need the
raw_payload_* columns, i.e. an FPGA build from 2026-09-26 on) and, over the new frames
in them:

  1. a map of all 96 payload bits: constant 0, constant 1, or changing (with counts)
  2. every field, the known ones (controller-firmware/python/sim/yaskawa_protocol.py) and
     each run of changing bits outside them: how many values it takes, whether it
     changes at standstill (a clock or noise) or only with motion, and its correlation
     with speed, direction and position
  3. for fields that change rarely, every change: when, the old and new value, and where
     the shaft was (single_turn, the hall-referenced electrical angle, hall, direction)

usage: analyse_yaskawa_payload.py capture.csv[.gz] [more.csv ...] [--fields 89:93,93:96] [--max-events 40]
"""
import argparse
import csv
import gzip
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "../../../controller-firmware/python/sim"))
from yaskawa_protocol import INCREMENTAL_FIELDS    # noqa: E402

# the CRC is not captured; mt4 is the low bits of index_count
KNOWN = {name: (o, w) for name, (o, w) in INCREMENTAL_FIELDS.items() if o < 96 and name != "mt4"}


def load(path):
    rows = []
    with (gzip.open(path, "rt") if path.endswith(".gz") else open(path)) as f:
        for r in csv.DictReader(f):
            if "raw_payload_0" not in r:
                sys.exit(f"{path}: no raw_payload columns (FPGA build from before 2026-09-26?)")
            if not int(r["flags"]) & 1:
                continue    # only new, good frames
            payload = int(r["raw_payload_0"]) | int(r["raw_payload_1"]) << 32 | int(r["raw_payload_2"]) << 64
            if payload == 0:
                continue    # before the first frame, or no raw registers
            rows.append({
                "session": int(r.get("session", 0)),
                "t": float(r["time_s"]),
                "payload": payload,
                "rpm": float(r["velocity_rpm"]),
                "single_turn": int(r["single_turn"]),
                "elec": float(r["electrical_angle"]),
                "hall": int(r["hall"]),
                "hallref": int(r["hall_reference"]),
                "restarts": int(r.get("hall_reference_restarts", 0)),
            })
    return rows


def bits(payload, offset, width):
    return (payload >> offset) & ((1 << width) - 1)


def pearson(xs, ys):
    n = len(xs)
    if n < 3:
        return float("nan")
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    syy = sum((y - my) ** 2 for y in ys)
    if sxx == 0 or syy == 0:
        return float("nan")
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / math.sqrt(sxx * syy)


def bit_map(rows):
    ones = [0] * 96
    toggles = [0] * 96
    prev = None
    for r in rows:
        p = r["payload"]
        for b in range(96):
            ones[b] += (p >> b) & 1
        if prev is not None:
            diff = p ^ prev
            while diff:
                low = diff & -diff
                toggles[low.bit_length() - 1] += 1
                diff ^= low
        prev = p
    return ones, toggles


def field_name(b):
    for name, (o, w) in KNOWN.items():
        if o <= b < o + w:
            return name
    return None


def unknown_runs(toggles):
    """Maximal runs of changing bits outside the known fields."""
    runs, start = [], None
    for b in range(97):
        changing = b < 96 and toggles[b] > 0 and field_name(b) is None
        if changing and start is None:
            start = b
        elif not changing and start is not None:
            runs.append((f"bits_{start}_{b}", (start, b - start)))
            start = None
    return runs


def describe(rows, name, offset, width, max_events):
    values = [bits(r["payload"], offset, width) for r in rows]
    distinct = sorted(set(values))
    changes = [i for i in range(1, len(rows)) if values[i] != values[i - 1]]
    still = [i for i in changes if abs(rows[i]["rpm"]) < 1.0]
    moving_samples = sum(1 for r in rows if abs(r["rpm"]) >= 1.0)
    duration = rows[-1]["t"] - rows[0]["t"] if len(rows) > 1 else 0

    shown = ", ".join(str(v) for v in distinct[:12]) + (" …" if len(distinct) > 12 else "")
    print(f"\n{name}  (bits {offset}:{offset + width})")
    print(f"  {len(distinct)} values: {shown}")
    if not changes:
        return
    print(f"  {len(changes)} changes in {duration:.1f} s; {len(still)} of them at standstill "
          f"({moving_samples / len(rows):.0%} of the frames were moving)")
    if len(distinct) > 2:
        speed = [abs(r["rpm"]) for r in rows]
        signed = [r["rpm"] for r in rows]
        pos = [r["single_turn"] for r in rows]
        print(f"  correlation with |rpm| {pearson(values, speed):+.2f}, rpm {pearson(values, signed):+.2f}, "
              f"single_turn {pearson(values, pos):+.2f}")
    if len(changes) <= max_events:
        print("       t (s)   old -> new   single_turn   elec angle   hall   rpm")
        for i in changes:
            r = rows[i]
            elec = f"{r['elec'] * 360:7.1f}°" + ("" if r["hallref"] == 2 else "?")
            print(f"  {r['t']:10.3f}  {values[i - 1]:5} -> {values[i]:<5} {r['single_turn']:10}   {elec:>10}   "
                  f"{r['hall']:4}   {r['rpm']:+7.1f}")
    else:
        # where in the turn do the changes happen?
        per_value = {}
        for i in changes:
            per_value.setdefault((values[i - 1], values[i]), []).append(rows[i]["single_turn"])
        common = sorted(per_value.items(), key=lambda kv: -len(kv[1]))[:8]
        print("  most common transitions (count, single_turn range):")
        for (a, b), where in common:
            print(f"    {a} -> {b}: {len(where)}, single_turn {min(where)}..{max(where)}")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("csv", nargs="+")
    parser.add_argument("--fields", help="extra fields to report, e.g. 89:93,93:96")
    parser.add_argument("--max-events", type=int, default=40)
    args = parser.parse_args()

    for path in args.csv:
        rows = load(path)
        print(f"==== {path}: {len(rows)} new frames")
        if len(rows) < 2:
            continue
        ones, toggles = bit_map(rows)

        print("\nbit map (0/1 constant, * changes); row = 16 bits, bit 0 first")
        for base in range(0, 96, 16):
            cells = "".join("*" if toggles[b] else str(ones[b] // len(rows)) for b in range(base, base + 16))
            labels = " ".join(sorted({field_name(b) or "?" for b in range(base, base + 16) if toggles[b]}))
            print(f"  {base:2}-{base + 15:2}  {cells[:8]} {cells[8:]}   {labels}")

        fields = [(n, ow) for n, ow in KNOWN.items()]
        fields += unknown_runs(toggles)
        if args.fields:
            for spec in args.fields.split(","):
                a, b = (int(x) for x in spec.split(":"))
                fields.append((f"bits_{a}_{b}", (a, b - a)))
        for name, (o, w) in fields:
            if name in ("single_turn", "ts_fast", "ts_slow", "position_age"):
                continue    # changing nearly every frame: see the bit map
            describe(rows, name, o, w, args.max_events)


if __name__ == "__main__":
    main()
