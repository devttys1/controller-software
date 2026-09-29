#!/usr/bin/env python3
"""Web monitor for controller_core: live telemetry rings in the browser.

The controller writes telemetry rings to shared memory (/dev/shm/em_*, see
core/controller/inc/telemetry_ring.h); the yaskawa_encoders driver writes one per
encoder, every cycle. This serves index.html (next to this file) and:

  GET /api/rings                 the rings and their schemas
  GET /api/stream?ring=NAME      Server-Sent Events: "schema" (again whenever the controller
                                 restarts), "data" {first, count, b64} about 20 times a second:
                                 the samples as they are in shared memory, base64 encoded, for
                                 the page to decode with the schema's offsets (so the board does
                                 no per-sample work), "status" {running, live} every second.
                                 optional &every=N keeps every Nth sample
  GET /api/capture.csv?ring=NAME everything in the ring (16 s at 1 kHz by default) as CSV
  GET /api/log?lines=N           the end of the controller log

Read-only and unauthenticated: for the bench network. Python standard library only
(the board has no pip). Runs on macOS too, against a --shm-dir of fake rings.

usage: em_monitor.py [--port 8080] [--shm-dir /dev/shm] [--log /var/volatile/controller.log]
"""
import argparse
import base64
import glob
import json
import mmap
import os
import struct
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

HEADER_SIZE = 4096
SCHEMA_OFFSET = 64
HERE = os.path.dirname(os.path.abspath(__file__))


class Ring:
    """Read-only view of one telemetry ring."""

    def __init__(self, path):
        self.path = path
        self.name = os.path.basename(path)
        fd = os.open(path, os.O_RDONLY)
        try:
            st = os.fstat(fd)
            self.inode = st.st_ino
            if st.st_size < HEADER_SIZE:
                raise ValueError(f"{path}: too small for a telemetry ring")
            self.mm = mmap.mmap(fd, st.st_size, mmap.MAP_SHARED, mmap.PROT_READ)
        finally:
            os.close(fd)
        if self.mm[:4] != b"EMTR":
            raise ValueError(f"{path}: not a telemetry ring")
        version, self.sample_size, self.capacity = struct.unpack_from("<III", self.mm, 4)
        if version != 1 or len(self.mm) < HEADER_SIZE + self.sample_size * self.capacity:
            raise ValueError(f"{path}: unsupported ring (version {version})")
        raw = self.mm[SCHEMA_OFFSET:HEADER_SIZE].split(b"\0", 1)[0]
        self.schema = json.loads(raw)

        # a struct format that picks each field out of the sample at its offset
        fields = sorted(self.schema["fields"], key=lambda f: f["offset"])
        fmt, pos = "<", 0
        for f in fields:
            fmt += "x" * (f["offset"] - pos) + f["type"]
            pos = f["offset"] + struct.calcsize("<" + f["type"])
        fmt += "x" * (self.sample_size - pos)
        self.names = [f["name"] for f in fields]
        self.struct = struct.Struct(fmt)

    def close(self):
        self.mm.close()

    def write_count(self):
        # the controller stores it atomically, but a 32-bit reader can still see half of it
        while True:
            a = struct.unpack_from("<Q", self.mm, 16)[0]
            b = struct.unpack_from("<Q", self.mm, 16)[0]
            if a == b:
                return a

    def update_frequency(self):
        return struct.unpack_from("<I", self.mm, 24)[0]

    def is_current(self):
        """False once the controller has replaced or removed the ring."""
        try:
            return os.stat(self.path).st_ino == self.inode
        except FileNotFoundError:
            return False

    def read_raw_from(self, start):
        """Samples numbered start.. that are still in the ring, as bytes: (first number, bytes, next start)."""
        count = self.write_count()
        # slot count % capacity may be half written (it is the next sample's), so keep capacity - 1
        first = max(start, count - self.capacity + 1, 0)
        chunks = []
        n = first
        while n < count:
            slot = n % self.capacity
            run = min(count - n, self.capacity - slot)
            offset = HEADER_SIZE + slot * self.sample_size
            chunks.append(self.mm[offset:offset + run * self.sample_size])
            n += run
        raw = b"".join(chunks)
        # drop anything overwritten while it was being read
        overwritten = self.write_count() - self.capacity + 1 - first
        if overwritten > 0:
            raw = raw[overwritten * self.sample_size:]
            first += overwritten
        return first, raw, count

    def read_from(self, start):
        """Like read_raw_from, with the samples unpacked into tuples in the order of self.names."""
        first, raw, count = self.read_raw_from(start)
        return first, list(self.struct.iter_unpack(raw)), count

    def info(self):
        return {
            "name": self.name,
            "schema": self.schema,
            "names": self.names,
            "sample_size": self.sample_size,
            "capacity": self.capacity,
            "update_frequency": self.update_frequency(),
            "write_count": self.write_count(),
        }


def controller_running():
    for cmdline in glob.glob("/proc/[0-9]*/cmdline"):
        try:
            with open(cmdline, "rb") as f:
                if b"controller_core" in f.read().split(b"\0", 1)[0]:
                    return True
        except OSError:
            pass
    return False


class Handler(BaseHTTPRequestHandler):
    server_version = "em_monitor/1"

    def log_message(self, fmt, *args):
        pass    # quiet: the stream would log a line per connection anyway

    def do_GET(self):
        url = urlparse(self.path)
        query = parse_qs(url.query)
        try:
            if url.path in ("/", "/index.html"):
                self.send_file(os.path.join(HERE, "index.html"), "text/html; charset=utf-8")
            elif url.path == "/api/rings":
                rings = self.open_rings()
                infos = [r.info() for r in rings]
                for r in rings:
                    r.close()
                self.send_json(infos)
            elif url.path == "/api/stream":
                self.stream(query.get("ring", [""])[0], max(1, int(query.get("every", ["1"])[0])))
            elif url.path == "/api/capture.csv":
                self.capture(query.get("ring", [""])[0])
            elif url.path == "/api/log":
                self.send_log(int(query.get("lines", ["200"])[0]))
            else:
                self.send_error(404)
        except (BrokenPipeError, ConnectionResetError):
            pass

    # ---- helpers

    def open_rings(self):
        rings = []
        for path in sorted(glob.glob(os.path.join(self.server.shm_dir, "em_*"))):
            try:
                rings.append(Ring(path))
            except (OSError, ValueError, json.JSONDecodeError):
                pass
        return rings

    def open_ring(self, name):
        if not name or "/" in name or not name.startswith("em_"):
            return None
        try:
            return Ring(os.path.join(self.server.shm_dir, name))
        except (OSError, ValueError, json.JSONDecodeError):
            return None

    def send_body(self, body, content_type, extra_headers=()):
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for k, v in extra_headers:
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def send_json(self, value):
        self.send_body(json.dumps(value).encode(), "application/json")

    def send_file(self, path, content_type):
        with open(path, "rb") as f:
            self.send_body(f.read(), content_type)

    def send_log(self, lines):
        try:
            with open(self.server.log_path, "rb") as f:
                f.seek(0, os.SEEK_END)
                f.seek(max(0, f.tell() - 200 * max(1, lines)))
                text = f.read().decode(errors="replace").splitlines()[-lines:]
        except OSError as e:
            text = [f"(no log: {e.strerror})"]
        self.send_body("\n".join(text).encode(), "text/plain; charset=utf-8")

    def capture(self, name):
        ring = self.open_ring(name)
        if ring is None:
            self.send_error(404, "no such ring")
            return
        try:
            first, rows, _ = ring.read_from(0)
            hz = ring.update_frequency() or 1
        finally:
            ring.close()
        out = ["time_s," + ",".join(ring.names)]
        cycle = ring.names.index("cycle") if "cycle" in ring.names else None
        c0 = rows[0][cycle] if rows and cycle is not None else 0
        for i, row in enumerate(rows):
            t = (row[cycle] - c0) / hz if cycle is not None else i / hz
            out.append(f"{t:.6f}," + ",".join(repr(v) for v in row))
        stamp = time.strftime("%Y%m%d-%H%M%S")
        self.send_body(("\n".join(out) + "\n").encode(), "text/csv",
                       [("Content-Disposition", f'attachment; filename="{name}-{stamp}.csv"')])

    def event(self, kind, value):
        self.wfile.write(f"event: {kind}\ndata: {json.dumps(value, separators=(',', ':'))}\n\n".encode())
        self.wfile.flush()

    def stream(self, name, every):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()

        ring, start = None, 0
        last_status = 0.0
        last_count, last_change = -1, 0.0
        try:
            while True:
                now = time.monotonic()
                if ring is not None and not ring.is_current():
                    ring.close()
                    ring = None
                if ring is None:
                    ring = self.open_ring(name)
                    if ring is not None:
                        info = ring.info()
                        info["every"] = every
                        self.event("schema", info)
                        # start with what the browser can show: the whole ring
                        start = 0
                if ring is not None:
                    first, raw, start = ring.read_raw_from(start)
                    size = ring.sample_size
                    if raw and every > 1:
                        # keep the samples numbered a multiple of every, so batches stay evenly spaced
                        skip = (-first) % every
                        raw = b"".join(raw[i:i + size] for i in range(skip * size, len(raw), every * size))
                        first += skip
                    if raw:
                        self.event("data", {"first": first, "count": len(raw) // size,
                                            "b64": base64.b64encode(raw).decode()})
                    if start != last_count:
                        last_count, last_change = start, now
                if now - last_status >= 1.0:
                    last_status = now
                    self.event("status", {
                        "running": controller_running(),
                        "ring": ring is not None,
                        "live": ring is not None and now - last_change < 1.0,
                    })
                time.sleep(0.05)
        finally:
            if ring is not None:
                ring.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--shm-dir", default="/dev/shm")
    parser.add_argument("--log", default="/var/volatile/controller.log")
    args = parser.parse_args()

    server = ThreadingHTTPServer((args.bind, args.port), Handler)
    server.daemon_threads = True
    server.shm_dir = args.shm_dir
    server.log_path = args.log
    print(f"em_monitor: http://{args.bind}:{args.port}/ reading {args.shm_dir}/em_*", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
