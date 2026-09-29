#!/usr/bin/env python3
"""Write telemetry rings like controller_core's, for trying the monitor web page without a board.

Simulates one yaskawa_encoders incremental port: a spindle that idles, is turned slowly by
hand (which locks the hall reference), runs up to 1500 rpm and back, and reverses, with an
occasional frame error. The ring layout is telemetry_ring.h's; the sample is the driver's
monitor_sample.

    python3 tests/fake_monitor_rings.py /tmp/em_shm &
    python3 zynq_files/controller/monitor/em_monitor.py --shm-dir /tmp/em_shm --port 8080

RingWriter is also used by em_monitor_test.py.
"""
import json
import math
import mmap
import os
import random
import struct
import sys
import time

HEADER_SIZE = 4096
SCHEMA_OFFSET = 64

# yaskawa_encoders::monitor_sample (96 bytes)
SAMPLE = struct.Struct("<QdddIIIIBBBxH2xIIIIddB7x")
FIELDS = [
    ("cycle", "Q", 0), ("total_turns", "d", 8), ("velocity_rpm", "d", 16), ("electrical_angle", "d", 24),
    ("single_turn", "I", 32), ("frame_error_count", "I", 36), ("no_response_count", "I", 40),
    ("hall_deviation", "I", 44), ("hall", "B", 48), ("hall_reference", "B", 49), ("flags", "B", 50),
    ("timestamp_fast", "H", 52), ("raw_payload_0", "I", 56), ("raw_payload_1", "I", 60), ("raw_payload_2", "I", 64),
    ("hall_reference_restarts", "I", 68), ("index_turns", "d", 72), ("index_angle", "d", 80), ("indexed", "B", 88),
]
SEQUENCE = [3, 1, 5, 4, 6, 2]
EDGES = [0, 5234, 11080, 16192, 21880, 26990]
COUNTS = 1 << 17
ELEC = COUNTS // 4


def encoder_schema(index=0):
    return {
        "module": "yaskawa_encoders", "driver": "node_4_yaskawa_encoders", "index": index,
        "mode": "17bit_incremental",
        "fields": [{"name": n, "type": t, "offset": o} for n, t, o in FIELDS],
        "flags": ["frame_valid", "no_response", "unindexed", "battery_fail"],
        "pole_pairs": 4, "hall_sequence": SEQUENCE, "hall_edges": EDGES,
    }


class RingWriter:
    def __init__(self, path, sample_size, capacity, schema, update_frequency=1000):
        self.sample_size, self.capacity, self.count = sample_size, capacity, 0
        size = HEADER_SIZE + sample_size * capacity
        # replace, as the controller does, so a reader holding the old one sees a new inode
        if os.path.exists(path):
            os.unlink(path)
        fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o644)
        try:
            os.ftruncate(fd, size)
            self.mm = mmap.mmap(fd, size)
        finally:
            os.close(fd)
        self.mm[0:4] = b"EMTR"
        struct.pack_into("<III", self.mm, 4, 1, sample_size, capacity)
        struct.pack_into("<I", self.mm, 24, update_frequency)
        raw = json.dumps(schema).encode() + b"\0"
        self.mm[SCHEMA_OFFSET:SCHEMA_OFFSET + len(raw)] = raw

    def push(self, sample_bytes):
        offset = HEADER_SIZE + (self.count % self.capacity) * self.sample_size
        self.mm[offset:offset + self.sample_size] = sample_bytes
        self.count += 1
        struct.pack_into("<Q", self.mm, 16, self.count)

    def close(self):
        self.mm.close()


def hall_at(rotor):
    e = rotor % ELEC
    for i in range(5, -1, -1):
        if e >= EDGES[i]:
            return SEQUENCE[i]
    return SEQUENCE[5]


def speed_profile(t):
    """rpm at time t (s): a 40 s loop."""
    t %= 40.0
    if t < 3:
        return 0.0
    if t < 8:
        return 30.0 * math.sin((t - 3) / 5 * math.pi)      # turned by hand
    if t < 13:
        return 1500.0 * (t - 8) / 5                         # run-up
    if t < 20:
        return 1500.0
    if t < 25:
        return 1500.0 * (25 - t) / 5
    if t < 28:
        return 0.0
    if t < 33:
        return -600.0 * math.sin((t - 28) / 5 * math.pi)   # reverse
    return 0.0


def main():
    shm_dir = sys.argv[1] if len(sys.argv) > 1 else "/tmp/em_shm"
    os.makedirs(shm_dir, exist_ok=True)
    hz, capacity = 1000, 16384
    ring = RingWriter(os.path.join(shm_dir, "em_node_4_yaskawa_encoders_0"), SAMPLE.size, capacity, encoder_schema(), hz)
    zero = random.randrange(COUNTS)           # the encoder's power-up zero
    rotor = random.randrange(COUNTS) * 1.0    # true rotor angle, counts
    ref = 1
    frame_errors, rpm_filtered = 0, 0.0
    prev_hall = hall_at(int(rotor))
    start = time.monotonic()
    cycle = 0
    print(f"writing {shm_dir}/em_node_4_yaskawa_encoders_0 at {hz} Hz", flush=True)
    while True:
        due = int((time.monotonic() - start) * hz)
        while cycle < due:
            cycle += 1
            rpm = speed_profile(cycle / hz)
            rotor += rpm / 60.0 * COUNTS / hz
            counts = int(rotor) - zero
            hall = hall_at(int(rotor))
            if ref == 1 and hall != prev_hall and abs(rpm) < 230:
                ref = 2
            prev_hall = hall
            valid = random.random() > 0.0005
            frame_errors += not valid
            rpm_filtered += 0.47 * (rpm + random.gauss(0, 0.3) - rpm_filtered)
            elec = (int(rotor) % ELEC) / ELEC
            if ref == 1:    # sector estimate: the sector centre
                i = SEQUENCE.index(hall)
                elec = ((EDGES[i] + (EDGES[i + 1] if i < 5 else ELEC)) / 2) / ELEC
            ring.push(SAMPLE.pack(
                cycle, counts / COUNTS, rpm_filtered, elec, counts % COUNTS, frame_errors, 0,
                40 if ref == 2 else 0, hall, ref, int(valid) | 1 << 2, (65536 - cycle * 500) % 65536, 0, 0, 0, 0,
                (rotor - 50000) / COUNTS, ((rotor - 50000) / COUNTS) % 1.0, 1))
        time.sleep(0.01)


if __name__ == "__main__":
    main()
