#!/usr/bin/env python3
"""Record the monitor web page's stream to CSV, for as long as you like.

The monitor's "Download CSV" only holds the last 16 s. This follows the live stream
(em_monitor.py /api/stream) from the moment it connects, every cycle, until --seconds
have passed or Ctrl-C, and writes the same columns as the download plus a "session"
column that counts controller restarts.

usage: record_monitor.py out.csv [--seconds 20] [--url http://em-os:8080] [--ring NAME]
"""
import argparse
import base64
import json
import struct
import sys
import time
import urllib.request


def events(stream):
    kind = None
    for line in stream:
        line = line.decode().rstrip("\n")
        if line.startswith("event: "):
            kind = line[7:]
        elif line.startswith("data: "):
            yield kind, json.loads(line[6:])


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("out")
    parser.add_argument("--seconds", type=float, default=0, help="stop after this long (default: Ctrl-C)")
    parser.add_argument("--url", default="http://em-os:8080")
    parser.add_argument("--ring", help="default: the first ring")
    args = parser.parse_args()

    url = args.url.rstrip("/")
    ring = args.ring
    if not ring:
        rings = json.load(urllib.request.urlopen(url + "/api/rings", timeout=10))
        if not rings:
            sys.exit("no rings: is controller_core running?")
        ring = rings[0]["name"]

    stream = urllib.request.urlopen(f"{url}/api/stream?ring={ring}", timeout=10)
    out = open(args.out, "w")
    unpack, names, hz, start, session = None, [], 1000, 0, -1
    expected, cycle0, written, lost = None, None, 0, 0
    t_start = time.monotonic()
    last_report = t_start
    try:
        for kind, data in events(stream):
            if kind == "schema":
                session += 1
                fields = sorted(data["schema"]["fields"], key=lambda f: f["offset"])
                fmt, pos = "<", 0
                for f in fields:
                    fmt += "x" * (f["offset"] - pos) + f["type"]
                    pos = f["offset"] + struct.calcsize("<" + f["type"])
                fmt += "x" * (data["sample_size"] - pos)
                unpack = struct.Struct(fmt)
                hz = data["update_frequency"] or 1000
                start = data["write_count"]     # only what arrives from now on, not the backlog
                expected = None
                if session == 0:
                    names = [f["name"] for f in fields]
                    out.write("session,time_s," + ",".join(names) + "\n")
                else:
                    print(f"controller restarted: session {session}", file=sys.stderr)
            elif kind == "data" and unpack is not None:
                raw = base64.b64decode(data["b64"])
                first = data["first"]
                cycle_index = names.index("cycle")
                for k, row in enumerate(unpack.iter_unpack(raw)):
                    n = first + k
                    if n < start:
                        continue
                    if expected is not None and n > expected:
                        lost += n - expected
                    expected = n + 1
                    if cycle0 is None:
                        cycle0 = row[cycle_index]
                    t = (row[cycle_index] - cycle0) / hz
                    out.write(f"{session},{t:.6f}," + ",".join(repr(v) for v in row) + "\n")
                    written += 1
            now = time.monotonic()
            if now - last_report >= 1:
                last_report = now
                print(f"\r{now - t_start:6.1f} s  {written} samples" + (f"  ({lost} lost)" if lost else ""),
                      end="", file=sys.stderr, flush=True)
            if args.seconds and now - t_start >= args.seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        out.close()
    print(f"\nwrote {written} samples to {args.out}" + (f", {lost} lost" if lost else ""), file=sys.stderr)


if __name__ == "__main__":
    main()
