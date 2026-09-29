#!/usr/bin/env python3
"""Host test for the monitor web server (zynq_files/controller/monitor/em_monitor.py).

Rings are written by fake_monitor_rings.RingWriter in telemetry_ring.h's layout (the C++ side
of that layout is checked by telemetry_ring_test.cpp and yaskawa_driver_test.cpp). Checks
reading across the wrap, samples overwritten mid-read, a controller restart, and the HTTP
endpoints the page uses. Run by run_host_tests.sh.
"""
import base64
import json
import os
import sys
import tempfile
import threading
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "../zynq_files/controller/monitor"))
sys.path.insert(0, HERE)
import em_monitor                                               # noqa: E402
from fake_monitor_rings import SAMPLE, RingWriter, encoder_schema   # noqa: E402

failures = 0


def expect(ok, what):
    global failures
    if not ok:
        print(f"  FAIL {what}")
        failures += 1


def sample(n):
    return SAMPLE.pack(n, n / 100, 10.0, 0.25, n % 131072, 0, 0, 0, 3, 2, 5, n * 7, n, 0xFFFFFFFF, 0x80000000, 0, n / 100, (n / 100) % 1, 1)


def make_ring(path, count, capacity=8):
    w = RingWriter(path, SAMPLE.size, capacity, encoder_schema(), 1000)
    for n in range(1, count + 1):
        w.push(sample(n))
    return w


def test_reading(tmp):
    path = os.path.join(tmp, "em_test_0")
    w = make_ring(path, 20)
    r = em_monitor.Ring(path)
    expect(r.names[0] == "cycle" and r.names[-1] == "indexed" and r.capacity == 8, "schema fields and capacity")

    first, rows, nxt = r.read_from(0)
    # 20 written into 8 slots; slot 20 % 8 is the next one being written, so 7 are safe
    expect(first == 13 and [row[0] for row in rows] == list(range(14, 21)) and nxt == 20,
           f"read across the wrap: first {first}, cycles {[row[0] for row in rows]}")
    first, rows, nxt = r.read_from(17)
    expect(first == 17 and [row[0] for row in rows] == [18, 19, 20], "read from a later sample")
    expect(r.read_from(20)[1] == [], "nothing new")

    class Racing(em_monitor.Ring):
        """The controller writes 3 more samples while the reader copies."""
        calls = 0

        def write_count(self):
            self.calls += 1
            return 20 if self.calls == 1 else 23

    racing = Racing(path)
    first, rows, _ = racing.read_from(0)
    expect(first == 16 and [row[0] for row in rows] == [17, 18, 19, 20], f"overwritten samples dropped: first {first}")
    racing.close()

    expect(r.is_current(), "ring is current")
    w.close()
    make_ring(path, 3).close()      # the controller restarted
    expect(not r.is_current(), "a replaced ring is noticed")
    r.close()


def get(port, path):
    with urllib.request.urlopen(f"http://127.0.0.1:{port}{path}", timeout=5) as f:
        return f.status, f.headers, f.read()


def read_events(port, path, count):
    events = []
    with urllib.request.urlopen(f"http://127.0.0.1:{port}{path}", timeout=5) as f:
        kind = None
        while len(events) < count:
            line = f.readline().decode().rstrip("\n")
            if line.startswith("event: "):
                kind = line[7:]
            elif line.startswith("data: "):
                events.append((kind, json.loads(line[6:])))
    return events


def test_http(tmp):
    make_ring(os.path.join(tmp, "em_node_4_yaskawa_encoders_0"), 30, capacity=16).close()
    log = os.path.join(tmp, "controller.log")
    with open(log, "w") as f:
        f.write("".join(f"line {i}\n" for i in range(500)))

    server = em_monitor.ThreadingHTTPServer(("127.0.0.1", 0), em_monitor.Handler)
    server.daemon_threads = True
    server.shm_dir, server.log_path = tmp, log
    port = server.server_address[1]
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        status, headers, body = get(port, "/")
        expect(status == 200 and b"Encoder monitor" in body and "text/html" in headers["Content-Type"], "page served")

        rings = json.loads(get(port, "/api/rings")[2])
        expect([r["name"] for r in rings] == ["em_node_4_yaskawa_encoders_0"] and rings[0]["write_count"] == 30,
               "rings listed")

        _, headers, body = get(port, "/api/capture.csv?ring=em_node_4_yaskawa_encoders_0")
        lines = body.decode().splitlines()
        expect(lines[0].startswith("time_s,cycle,total_turns,") and len(lines) == 1 + 15, f"CSV has 15 samples ({len(lines) - 1})")
        expect(lines[1].startswith("0.000000,16,") and lines[-1].startswith("0.014000,30,"), "CSV time column from cycles")
        expect("attachment" in headers["Content-Disposition"], "CSV is a download")

        text = get(port, "/api/log?lines=3")[2].decode()
        expect(text == "line 497\nline 498\nline 499", f"log tail: {text!r}")

        for bad in ("../etc/passwd", "controller.log", "em_missing"):
            try:
                get(port, "/api/capture.csv?ring=" + bad)
                expect(False, f"ring {bad!r} served")
            except urllib.error.HTTPError as e:
                expect(e.code == 404, f"ring {bad!r}: {e.code}")

        events = read_events(port, "/api/stream?ring=em_node_4_yaskawa_encoders_0&every=4", 2)
        expect(events[0][0] == "schema" and events[0][1]["every"] == 4 and events[0][1]["names"][0] == "cycle", "stream starts with the schema")
        kind, data = events[1]
        raw = base64.b64decode(data["b64"])
        cycles = [row[0] for row in SAMPLE.iter_unpack(raw)]
        expect(data["count"] * SAMPLE.size == len(raw), "count matches the bytes")
        # samples 15..29 are in the ring (cycles 16..30); every 4th sample number: 16, 20, 24, 28
        expect(kind == "data" and data["first"] == 16 and cycles == [17, 21, 25, 29], f"decimated stream: {data['first']}, {cycles}")
    finally:
        server.shutdown()
        server.server_close()


def main():
    tests = [("ring reading", test_reading), ("http endpoints", test_http)]
    for name, test in tests:
        before = failures
        with tempfile.TemporaryDirectory() as tmp:
            try:
                test(tmp)
            except Exception as e:     # a crash is a failure, and the other tests still run
                expect(False, f"exception: {e!r}")
        print(f"  {'PASS' if failures == before else 'FAIL'} {name}")
    print(f"{failures} failed" if failures else "all passed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
