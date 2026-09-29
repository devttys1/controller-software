#!/usr/bin/env python3
"""Build data/yaskawa_index_frames.csv for yaskawa_index_test.cpp from bench recordings.

The recordings (core/tools/record_monitor.py, 2026-09-26, the UTSIH-B17CK on the bench) are too big
to keep, so this keeps the good frames around every index_count step (±80 frames) and the first 80
frames after every encoder power-up, in order. Rows from different recordings, or with frames missing
between them, start a new "segment".

usage: make_yaskawa_index_fixtures.py recording.csv [more.csv ...]
The fixture in the repo came from the three bench runs of 2026-09-26 (the payload decode run up to the
freeze at 212.5 s, the power-cycle run, and the confirmation run after the receiver fix).
"""
import csv
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "data", "yaskawa_index_frames.csv")
CPR = 1 << 17
WINDOW = 80


def count(raw2):
    return (raw2 >> (89 - 64)) & 0x7F


def main():
    out_rows = []
    segment = -1
    for path in sys.argv[1:]:
        frames = []
        with open(path) as f:
            for r in csv.DictReader(f):
                if not int(r["flags"]) & 1:
                    continue
                raw = [int(r[f"raw_payload_{w}"]) for w in range(3)]
                if raw == [0, 0, 0]:
                    continue
                frames.append({
                    "t": float(r["time_s"]),
                    "restarts": int(r["hall_reference_restarts"]),
                    "single_turn": int(r["single_turn"]),
                    "position": round(float(r["total_turns"]) * CPR),
                    "raw": raw,
                    "elec": float(r["electrical_angle"]),
                    "hallref": int(r["hall_reference"]),
                })
        keep = set()
        for i in range(len(frames)):
            first_of_power_up = i == 0 or frames[i]["restarts"] != frames[i - 1]["restarts"]
            stepped = i > 0 and count(frames[i]["raw"][2]) != count(frames[i - 1]["raw"][2])
            if first_of_power_up:
                keep.update(range(i, min(len(frames), i + WINDOW)))
            if stepped:
                keep.update(range(max(0, i - WINDOW), min(len(frames), i + WINDOW)))
        prev = None
        for i in sorted(keep):
            if prev is None or i != prev + 1:
                segment += 1
            prev = i
            fr = frames[i]
            out_rows.append([segment, fr["single_turn"], fr["position"], *fr["raw"], f"{fr['elec']:.6f}", fr["hallref"]])
        print(f"{path}: {len(frames)} good frames, kept {len(keep)}")

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w") as f:
        f.write("segment,single_turn,position,raw_payload_0,raw_payload_1,raw_payload_2,electrical_angle,hall_reference\n")
        for row in out_rows:
            f.write(",".join(str(v) for v in row) + "\n")
    print(f"wrote {OUT}: {len(out_rows)} frames in {segment + 1} segments")


if __name__ == "__main__":
    main()
