"""Headless dashboard tests against the simulated rig (no hardware needed).

    python tests/test_dashboard.py      (or: pytest tests/)
"""
from __future__ import annotations

import csv
import queue
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "dashboard"))
from rig_dashboard import CsvLogger, Link, envelope  # noqa: E402
from rig_protocol import ParseError, command, frame, parse_sample, split_line  # noqa: E402


def test_protocol_roundtrip_and_rejects_bad_lines():
    line = frame("D,1000,5,123.45,246900,1.2345,0.0100,HOLD,500.0,NONE")
    s = parse_sample(split_line(line))
    assert s.force == 123.45 and s.mode == "HOLD" and s.seq == 5
    for bad in (line[:-4] + "\n", line.replace("123.45", "923.45"), "garbage"):
        try:
            split_line(bad)
        except ParseError:
            continue
        raise AssertionError(f"accepted bad line {bad!r}")
    assert command("HOLD 500 20").endswith(b"\n")


def test_heartbeat_line_parses_nan_force():
    s = parse_sample(split_line(frame("D,100,0,nan,0,1.2345,0.0000,BENCH,0.0,NONE")))
    assert np.isnan(s.force) and s.mode == "BENCH"


def test_envelope_keeps_spikes():
    x = np.arange(100_000.0)
    y = np.zeros_like(x)
    y[54_321] = 999
    _, (ye,) = envelope(x, [y])
    assert ye.max() == 999 and len(ye) <= 3000


def test_link_reconnects_and_keeps_logging():
    with tempfile.TemporaryDirectory() as tmp:
        q: queue.Queue = queue.Queue()
        lg = CsvLogger()
        link = Link(q, lg, sim_drop=3.0)
        link.start()
        link.connect("SIM")
        t0 = time.time()
        while not link.connected and time.time() - t0 < 5:
            time.sleep(0.05)
        path = lg.start(Path(tmp), "selftest", {"k": "v"})
        assert link.send("HOLD 300 100")
        statuses, acks = [], []
        end = time.time() + 8
        while time.time() < end:
            try:
                kind, _t, p = q.get(timeout=0.2)
            except queue.Empty:
                continue
            if kind == "status":
                statuses.append(p)
            elif kind == "ack":
                acks.append(p)
        lg.stop()
        link.shutdown()
        time.sleep(0.3)
        with open(path) as f:
            rows = list(csv.reader(f))
        assert ["HOLD"] in acks
        assert any("reconnecting" in s for s in statuses)
        assert sum(s == "connected SIM" for s in statuses) >= 2
        assert len(rows) - 1 >= 50
        assert link.bad_lines == 0


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok ", name)
    print("all tests passed")
