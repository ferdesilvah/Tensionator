"""Multi-point load-cell calibration with discrete weight plates.

The firmware's own CAL is a single-point span: one known load, one scale factor. This
script hangs plates on one at a time (loading leg), takes them off again (unloading
leg), and fits the scale to every point. It also reports what a single point cannot
show: non-linearity, hysteresis, zero return and noise.

    python tools/calibrate_plates.py --plates 5,5,10,10,20 --hanger-kg 1.234
    python tools/calibrate_plates.py --plates 5,5,10,10,20 --hanger-kg 1.234 --check
    python tools/calibrate_plates.py --sim --plates 5,5,10,10,20      # no hardware

Modes
  (default)  calibrate: TARE, load + unload, fit, and (after you confirm) store the
             fitted scale on the board.
  --check    verify only: no TARE, nothing written to the board. Compares the board's
             force reading against the plates, and fails if any point is out by more
             than --tol-pct. Use it between tests to see if a re-calibration is needed.

How the fitted scale gets onto the board without a firmware change: the firmware only
accepts CAL <N>, which sets scale = (raw_now - offset) / N. At the top of the loading
leg, with all plates still hanging, the script works out the N that makes the firmware
land on the fitted scale and sends that. It then reads the scale back to confirm.

Close the dashboard first: only one program can hold the serial port.
Weigh every plate and the hanger on a scale you trust, and pass those masses. Stamped
plate masses are often 1-2 % off, which would be the biggest error in the whole job.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import random
import sys
import threading
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

import numpy as np
import serial
from serial.tools import list_ports

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "dashboard"))
from rig_protocol import ParseError, command, frame, parse_sample, split_line  # noqa: E402

G_STD = 9.80665           # m/s^2; pass --g for your site if you want the last 0.1 %
BAUD = 115200
ARDUINO_VID = 0x2341
BOOTLOADER_PID = 0x0369
OUT_DIR = ROOT / "dashboard" / "data" / "calibration"   # git-ignored with the rest of data/
SOFT_FMAX_N = 1100.0


class Abort(Exception):
    pass


# ----------------------------------------------------------------------------- serial link
class Link:
    """Background reader for the rig's line protocol."""

    def __init__(self, ser):
        self.ser = ser
        self.lock = threading.Lock()
        self.samples: list = []      # (host_monotonic, Sample)
        self.events: list = []       # (host_monotonic, [NAME, args...])
        self.params: dict = {}
        self.replies: list = []      # (host_monotonic, 'K'|'X', [CMD, reason])
        self.err: str | None = None
        self.alive = True
        self._buf = b""
        self._th = threading.Thread(target=self._run, daemon=True)
        self._th.start()

    def close(self):
        self.alive = False
        self._th.join(timeout=1)
        try:
            self.ser.close()
        except Exception:
            pass

    def _run(self):
        while self.alive:
            try:
                n = self.ser.in_waiting
                data = self.ser.read(n if n else 1)
            except (serial.SerialException, OSError) as e:
                self.err = str(e) or type(e).__name__
                self.alive = False
                return
            if not data:
                continue
            self._buf += data
            while b"\n" in self._buf:
                line, self._buf = self._buf.split(b"\n", 1)
                self._handle(line.decode("ascii", "replace"))

    def _handle(self, line: str):
        try:
            f = split_line(line)
        except ParseError:
            return
        now = time.monotonic()
        with self.lock:
            if f[0] == "D":
                try:
                    self.samples.append((now, parse_sample(f)))
                except ParseError:
                    return
                if len(self.samples) > 50_000:
                    del self.samples[:25_000]
            elif f[0] == "E":
                self.events.append((now, f[2:] if len(f) > 2 else []))
            elif f[0] == "P" and len(f) >= 3:
                self.params[f[1]] = f[2]
            elif f[0] in ("K", "X"):
                self.replies.append((now, f[0], f[1:]))

    def _check(self):
        if self.err:
            raise Abort(f"serial link lost: {self.err}")

    def send(self, text: str, timeout: float = 3.0) -> None:
        """Send a command and wait for its K; raise on X or timeout."""
        self._check()
        word = text.split()[0].upper()
        with self.lock:
            self.replies.clear()
        self.ser.write(command(text))
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            with self.lock:
                for _, typ, f in self.replies:
                    if f and f[0].upper() in (word, "?"):
                        if typ == "K":
                            return
                        raise Abort(f"board rejected {text!r}: {f[1] if len(f) > 1 else '?'}")
            self._check()
            time.sleep(0.01)
        raise Abort(f"no reply to {text!r}")

    def get_params(self) -> dict:
        with self.lock:
            self.params.clear()
        self.send("GET")
        with self.lock:
            return dict(self.params)

    def wait_event(self, names: tuple[str, ...], since: float, timeout: float) -> list[str]:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            with self.lock:
                for ts, f in self.events:
                    if ts >= since and f and f[0] in names:
                        return f
            self._check()
            time.sleep(0.02)
        raise Abort(f"timed out waiting for {'/'.join(names)}")

    def window(self, seconds: float):
        """Collect samples for `seconds`; abort on a fault or if the cell goes quiet."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            self._check()
            time.sleep(0.05)
        with self.lock:
            got = [s for ts, s in self.samples if ts >= t0]
        good = [s for s in got if not math.isnan(s.force)]
        if not good:
            raise Abort("no load-cell samples (HX711 unplugged? HX_TIMEOUT?)")
        bad = {s.fault for s in got if s.fault != "NONE"}
        if bad:
            raise Abort(f"board reports fault {', '.join(sorted(bad))}")
        return good


# ----------------------------------------------------------------------------- simulator
class PlateSim:
    """--sim: a load cell with plates hanging off it. Mimics TARE/CAL/GET and the D lines.

    True scale is negative on purpose (the README allows it) and the cell has a little
    non-linearity, hysteresis and swing after each plate change, so the report has
    something to show. Not a model of the real firmware's control logic.
    """

    def __init__(self, rate_hz: float = 80.0):
        self.period = 1.0 / rate_hz
        self.t0 = self.last = time.monotonic()
        self.out = bytearray()
        self.inbuf = b""
        self.seq = 0
        self.true_off, self.true_scale = 84_213.0, -2143.7
        self.load_N, self.prev_load_N, self.peak_N, self.changed_at = 0.0, 0.0, 0.0, self.t0
        self.off, self.scale, self.cal_valid = 80_000, -2100.0, 1
        self.mode, self.avg, self.cal_n = "IDLE", [], 0.0

    def set_load(self, newtons: float):
        self.prev_load_N, self.load_N, self.changed_at = self.load_N, newtons, time.monotonic()
        self.peak_N = max(self.peak_N, newtons)

    # serial.Serial-like API
    @property
    def in_waiting(self):
        self._advance()
        return len(self.out)

    def read(self, n=1):
        self._advance()
        if not self.out:
            time.sleep(0.005)
            self._advance()
        b, self.out = bytes(self.out[:n]), self.out[n:]
        return b

    def write(self, data):
        self.inbuf += data
        while b"\n" in self.inbuf:
            line, self.inbuf = self.inbuf.split(b"\n", 1)
            self._command(line.decode().strip())
        return len(data)

    def close(self):
        pass

    def _emit(self, body):
        self.out += frame(body).encode()

    def _raw(self, now):
        f = self.load_N
        dt = now - self.changed_at
        f += (self.load_N - self.prev_load_N) * 0.15 * math.exp(-dt / 0.6) * math.sin(7.5 * dt)  # swing
        f += 2e-6 * f * f                                       # 0.2 % non-linearity at 1 kN
        if self.load_N < self.peak_N:                           # unloading branch reads high
            f += 0.0008 * (self.peak_N - self.load_N) * (self.load_N / max(self.peak_N, 1))
        return int(round(self.true_off + self.true_scale * f + random.gauss(0, 60)))

    def _advance(self):
        now = time.monotonic()
        while now - self.last >= self.period:
            self.last += self.period
            self.seq += 1
            raw = self._raw(self.last)
            if self.mode in ("TARE", "CAL"):
                self.avg.append(raw)
                if len(self.avg) >= 20:
                    a = int(sum(self.avg) / len(self.avg))
                    ms = int((self.last - self.t0) * 1000)
                    if self.mode == "TARE":
                        self.off = a
                        self._emit(f"E,{ms},TARED,{a}")
                    else:
                        self.scale, self.cal_valid = (a - self.off) / self.cal_n, 1
                        self._emit(f"E,{ms},CALIBRATED,{self.scale:.3f}")
                    self.mode = "IDLE"
            force = (raw - self.off) / self.scale
            self._emit(f"D,{int((self.last - self.t0) * 1000)},{self.seq},{force:.2f},{raw},"
                       f"0.0000,0.0000,{self.mode},0.0,NONE")

    def _command(self, line):
        if "*" in line:
            line = ",".join(split_line(line))
        tok = line.replace(",", " ").upper().split()
        c = tok[0]
        if c == "GET":
            for k, v in (("fw", "platesim"), ("fmax", "1100.0"), ("offset", self.off),
                         ("scale", f"{self.scale:.3f}"), ("calvalid", self.cal_valid),
                         ("hz", "80.0"), ("mode", self.mode), ("fault", "NONE")):
                self._emit(f"P,{k},{v}")
        elif c == "TARE":
            self.mode, self.avg = "TARE", []
        elif c == "CAL":
            self.mode, self.avg, self.cal_n = "CAL", [], float(tok[1])
        self._emit(f"K,{c}")


# ----------------------------------------------------------------------------- measurement
@dataclass
class Point:
    leg: str            # zero / load / unload / return
    step: int
    mass_kg: float
    force_N: float      # known applied force
    raw_mean: float
    raw_std: float
    n: int
    stable: bool
    fw_force_N: float   # what the board itself read (its current offset/scale)


def measure(link: Link, settle_s: float, win_s: float,
            counts_per_N: float, tol_N: float, max_windows: int = 8):
    """Wait for the plates to stop swinging, then average.

    Takes consecutive windows until two in a row agree to within
    max(3 x their combined standard error, tol_N). Returns (mean, std, n, stable, fw_force).
    """
    time.sleep(settle_s)
    prev = None
    for _ in range(max_windows):
        win = link.window(win_s)
        raws = np.array([s.raw for s in win], dtype=float)
        fw = float(np.mean([s.force for s in win]))
        cur = (raws.mean(), raws.std(ddof=1) if len(raws) > 1 else 0.0, len(raws), raws, fw)
        if prev is not None:
            sem = math.hypot(prev[1] / math.sqrt(prev[2]), cur[1] / math.sqrt(cur[2]))
            tol = max(3 * sem, tol_N * abs(counts_per_N))
            if abs(cur[0] - prev[0]) <= tol:
                both = np.concatenate([prev[3], cur[3]])
                return both.mean(), both.std(ddof=1), len(both), True, (prev[4] + cur[4]) / 2
        prev = cur
    return cur[0], cur[1], cur[2], False, cur[4]


# ----------------------------------------------------------------------------- analysis
def fit(points: list[Point], offset: float) -> dict:
    """Least-squares scale (counts/N) through the tare offset, plus quality figures."""
    F = np.array([p.force_N for p in points])
    raw = np.array([p.raw_mean for p in points])
    d = raw - offset
    loaded = F > 0
    if loaded.sum() < 1:
        raise Abort("no loaded points to fit")
    scale = float(np.sum(F * d) / np.sum(F * F))
    resid = d / scale - F                                  # N, per point
    fmax = float(F.max())
    out = {"scale": scale, "offset": offset, "max_load_N": fmax,
           "resid_N": resid.tolist(),
           "max_error_N": float(np.max(np.abs(resid))),
           "max_error_pct_fs": float(100 * np.max(np.abs(resid)) / fmax)}
    if len(np.unique(F)) >= 3:                              # best straight line, free intercept
        b, a = np.polyfit(F, raw, 1)
        nl = (raw - (a + b * F)) / b
        out["nonlinearity_pct_fs"] = float(100 * np.max(np.abs(nl)) / fmax)
        out["free_intercept_N"] = float((a - offset) / b)
    hyst = []
    for p in points:
        if p.leg == "unload":
            ups = [q for q in points if q.leg == "load" and abs(q.mass_kg - p.mass_kg) < 1e-9]
            if ups:
                hyst.append((p.raw_mean - ups[0].raw_mean) / scale)
    if hyst:
        out["hysteresis_pct_fs"] = float(100 * max(abs(h) for h in hyst) / fmax)
    zeros = [p for p in points if p.force_N == 0]
    if len(zeros) >= 2:
        out["zero_return_N"] = float((zeros[-1].raw_mean - zeros[0].raw_mean) / scale)
    out["noise_N_rms"] = float(np.median([p.raw_std for p in points]) / abs(scale))
    return out


# ----------------------------------------------------------------------------- main flow
def find_port(explicit: str | None) -> str:
    if explicit:
        return explicit
    for p in list_ports.comports():
        if p.vid == ARDUINO_VID and p.pid != BOOTLOADER_PID:
            return p.device
    raise Abort("no Arduino found; pass --port COMx (and close the dashboard)")


def open_port(dev: str):
    s = serial.Serial()
    s.port, s.baudrate, s.timeout, s.write_timeout = dev, BAUD, 0.1, 1.0
    s.dtr, s.rts = True, False          # firmware only talks while DTR is asserted
    s.open()
    return s


def ask(prompt: str, auto: bool) -> str:
    print(prompt, end="", flush=True)
    if auto:
        print("  [sim: auto]")
        return ""
    try:
        return input().strip().lower()
    except EOFError:
        raise Abort("no input") from None


def parse_masses(s: str) -> list[float]:
    try:
        m = [float(x) for x in s.replace(";", ",").split(",") if x.strip()]
    except ValueError:
        raise argparse.ArgumentTypeError("plates must be a comma-separated list of kg") from None
    if not m or any(x <= 0 for x in m):
        raise argparse.ArgumentTypeError("plate masses must be positive")
    return m


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--plates", type=parse_masses, required=True,
                    help="measured plate masses in kg, in the order you will hang them, e.g. 5.02,5.01,10.03")
    ap.add_argument("--hanger-kg", type=float, default=0.0,
                    help="mass of the hanger/shackle/sling that carries the plates (hung first)")
    ap.add_argument("--g", type=float, default=G_STD, help=f"local gravity, m/s^2 (default {G_STD})")
    ap.add_argument("--port", help="serial port, e.g. COM5 (default: auto-detect)")
    ap.add_argument("--sim", action="store_true", help="run against a simulated cell")
    ap.add_argument("--check", action="store_true", help="verify the stored calibration only; change nothing")
    ap.add_argument("--no-unload", action="store_true", help="skip the unloading leg (no hysteresis figure)")
    ap.add_argument("--settle", type=float, default=5.0, help="seconds to wait after each change (default 5)")
    ap.add_argument("--window", type=float, default=3.0, help="averaging window, seconds (default 3)")
    ap.add_argument("--tol-n", type=float, default=0.05,
                    help="stability: two consecutive windows must agree within this many N (default 0.05)")
    ap.add_argument("--tol-pct", type=float, default=0.5,
                    help="--check pass limit, %% of full calibrated load (default 0.5)")
    ap.add_argument("--yes", action="store_true", help="apply the fitted scale without asking")
    ap.add_argument("--out", type=Path, default=OUT_DIR, help="where to write results")
    args = ap.parse_args(argv)

    if args.sim:
        args.settle, args.window = min(args.settle, 1.0), min(args.window, 0.6)
    masses = ([args.hanger_kg] if args.hanger_kg > 0 else []) + args.plates
    cum = list(np.cumsum(masses))
    top_N = cum[-1] * args.g
    if top_N > SOFT_FMAX_N:
        print(f"Total load {top_N:.0f} N exceeds the {SOFT_FMAX_N:.0f} N soft limit; remove plates.")
        return 2
    if top_N < 10:
        print("Total load must be at least 10 N (firmware CAL minimum). Aim for >= 20 % of your test loads.")
        return 2

    sim = PlateSim() if args.sim else None
    try:
        ser = sim if sim else open_port(find_port(args.port))
    except (Abort, serial.SerialException, OSError) as e:
        print(f"Cannot open the port: {e}")
        return 2
    link = Link(ser)
    auto = sim is not None
    place = (lambda kg: sim.set_load(kg * args.g)) if sim else (lambda kg: None)
    points: list[Point] = []
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    applied = None
    try:
        time.sleep(0.5)
        prm = link.get_params()
        if prm.get("mode", "IDLE") != "IDLE":
            raise Abort(f"board is in {prm.get('mode')} mode; STOP it first")
        if prm.get("fault", "NONE") != "NONE":
            raise Abort(f"fault {prm.get('fault')} latched; fix the cause and send CLR")
        if prm.get("nocell") == "1":
            raise Abort("board is in bench mode (NOCELL 1); send NOCELL 0")
        hz = float(prm.get("hz", "nan"))
        print(f"Board fw {prm.get('fw')}  sample rate {hz:.0f} Hz  stored offset {prm.get('offset')} "
              f"scale {prm.get('scale')}  calvalid {prm.get('calvalid')}")
        if hz < 40:
            print("  note: ~10 Hz HX711; windows will be noisier. 80 Hz (RATE pin high) is recommended.")
        old_scale, old_off = float(prm.get("scale", 0)), float(prm.get("offset", 0))
        if args.check and prm.get("calvalid") != "1":
            raise Abort("board is not calibrated; run without --check")

        print("\nBefore you start:\n"
              "  * Rope off the load cell (or fully slack): ONLY the plates may load the cell.\n"
              "  * Load along the same axis the rope pulls, through the same eye/shackle if possible.\n"
              "  * Motor energised and stopped. Nothing touching the hanger while it measures.")
        ask("Cell unloaded, hanger OFF. Press Enter to start (Ctrl+C aborts any time) ", auto)
        place(0.0)

        if not args.check:
            print("Taring...", flush=True)
            time.sleep(args.settle)
            t = time.monotonic()
            link.send("TARE")
            link.wait_event(("TARED",), t, 10)
            prm = link.get_params()
        offset = float(prm["offset"])
        counts_per_N = old_scale if abs(old_scale) > 1 else 2000.0

        def take(leg, step, kg):
            nonlocal counts_per_N
            m, sd, n, ok, fwf = measure(link, args.settle, args.window, counts_per_N, args.tol_n)
            F = kg * args.g
            p = Point(leg, step, kg, F, m, sd, n, ok, fwf)
            points.append(p)
            if F > 0 and leg == "load":                       # refine the stability tolerance
                counts_per_N = (m - offset) / F
            flag = "" if ok else "   ** did not settle; check for swing/rubbing **"
            print(f"  {leg:6s} {kg:8.3f} kg {F:8.2f} N   raw {m:12.1f} +/- {sd:6.1f} (n={n})"
                  f"   board reads {fwf:8.2f} N{flag}")
            return p

        print("\nLoading leg")
        take("zero", 0, 0.0)
        names = (["hanger"] if args.hanger_kg > 0 else []) + [f"plate {i + 1}" for i in range(len(args.plates))]
        for i, (name, kg, tot) in enumerate(zip(names, masses, cum), start=1):
            if ask(f"Hang {name} ({kg:g} kg) -> total {tot:g} kg = {tot * args.g:.1f} N. "
                   "Hands off, Enter to measure (q to quit) ", auto) == "q":
                raise Abort("stopped by user")
            place(tot)
            take("load", i, tot)

        if not args.check:
            loading = [p for p in points if p.leg in ("zero", "load")]
            res = fit(loading, offset)
            s = res["scale"]
            print(f"\nLoading-leg fit: scale {s:.3f} counts/N (stored was {old_scale:.3f}, "
                  f"{100 * (s - old_scale) / old_scale:+.2f} %), worst point {res['max_error_N']:.2f} N")
            go = args.yes or auto or ask("Store this scale on the board now, with all plates still hanging? [y/N] ",
                                         False) == "y"
            if go:
                now = link.window(1.0)
                raw_now = float(np.mean([x.raw for x in now]))
                n_cmd = (raw_now - offset) / s                  # makes firmware scale == s
                if n_cmd < 10:
                    raise Abort("load too small for CAL (needs the equivalent of >= 10 N)")
                t = time.monotonic()
                link.send(f"CAL {n_cmd:.4f}")
                ev = link.wait_event(("CALIBRATED", "CALFAIL"), t, 10)
                if ev[0] != "CALIBRATED":
                    raise Abort(f"CAL failed: {','.join(ev[1:])}")
                stored = float(link.get_params()["scale"])
                applied = {"target_scale": s, "stored_scale": stored,
                           "diff_pct": 100 * (stored - s) / s}
                print(f"  board scale now {stored:.3f} ({applied['diff_pct']:+.3f} % from the fit)")
            else:
                print("  not stored; the board keeps its old calibration")

        if not args.no_unload:
            print("\nUnloading leg")
            down = list(zip(names, masses, cum))[::-1]
            for j, (name, kg, tot) in enumerate(down):
                remaining = tot - kg
                if ask(f"Remove {name} ({kg:g} kg) -> total {remaining:g} kg. Hands off, Enter ", auto) == "q":
                    raise Abort("stopped by user")
                place(remaining)
                take("unload" if remaining > 1e-9 else "return", len(down) - j - 1, remaining)

    except (Abort, KeyboardInterrupt) as e:
        print(f"\nAborted: {e or 'Ctrl+C'}. Nothing further was sent to the board.")
        if applied:
            print("  (the new scale WAS stored before the abort)")
        link.close()
        return 1
    link.close()

    # ---- report -------------------------------------------------------------------------------
    final_scale = applied["stored_scale"] if applied else (old_scale if args.check else None)
    # --check does not tare, so judge the span from the zero measured now; the stored
    # offset's error is reported on its own (a TARE fixes it, a re-calibration is not needed).
    zero_raw = points[0].raw_mean
    ref = zero_raw if args.check else offset
    res = fit(points, ref)
    print("\n================ RESULT ================")
    print(f"fitted scale (all points)   {res['scale']:.3f} counts/N   zero {ref:.0f}")
    if args.check:
        zshift = (zero_raw - offset) / old_scale
        print(f"stored zero is off by       {zshift:+.3f} N" +
              ("   -> TARE with the rope slack before testing" if abs(zshift) > 0.5 else ""))
    print(f"max load                    {res['max_load_N']:.1f} N")
    print(f"worst point vs fit          {res['max_error_N']:.3f} N  ({res['max_error_pct_fs']:.3f} % FS)")
    for k, label in (("nonlinearity_pct_fs", "non-linearity"), ("hysteresis_pct_fs", "hysteresis")):
        if k in res:
            print(f"{label:28s}{res[k]:.3f} % FS")
    if "zero_return_N" in res:
        print(f"zero return                 {res['zero_return_N']:+.3f} N")
    print(f"noise (per sample, rms)     {res['noise_N_rms']:.3f} N")

    verdict = None
    if final_scale:
        err = [p.raw_mean - ref for p in points]
        fw_err = [e / final_scale - p.force_N for e, p in zip(err, points)]
        worst = max(abs(x) for x in fw_err)
        pct = 100 * worst / res["max_load_N"]
        print(f"board span error, worst pt  {worst:.3f} N  ({pct:.3f} % FS) with scale {final_scale:.3f}")
        if args.check:
            verdict = "PASS" if pct <= args.tol_pct else "FAIL"
            print(f"CHECK {verdict} (limit {args.tol_pct} % FS)" +
                  ("" if verdict == "PASS" else "  -> run a full calibration"))
    if applied and abs(res["scale"] - applied["stored_scale"]) / abs(applied["stored_scale"]) > 0.002:
        print("note: the unloading leg moved the fit by > 0.2 %; hysteresis or a disturbed point. Consider a re-run.")
    if any(not p.stable for p in points):
        print("note: some points never settled (marked). Re-run with a longer --settle if they look off.")

    # ---- files --------------------------------------------------------------------------------
    args.out.mkdir(parents=True, exist_ok=True)
    tag = "check" if args.check else ("sim" if args.sim else "cal")
    pts_path = args.out / f"{stamp}_plate{tag}.csv"
    with open(pts_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(list(asdict(points[0]).keys()) + ["fit_resid_N"])
        for p, r in zip(points, res["resid_N"]):
            w.writerow(list(asdict(p).values()) + [f"{r:.4f}"])
    summary = {"time": stamp, "mode": tag, "g": args.g, "hanger_kg": args.hanger_kg, "plates_kg": args.plates,
               "old_scale": old_scale, "old_offset": old_off, "applied": applied, "verdict": verdict,
               **{k: v for k, v in res.items() if k != "resid_N"}}
    (args.out / f"{stamp}_plate{tag}.json").write_text(json.dumps(summary, indent=2))
    hist = args.out / "cal_history.csv"
    new = not hist.exists()
    with open(hist, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["time", "mode", "offset", "fitted_scale", "stored_scale", "max_load_N",
                        "max_error_pct_fs", "nonlinearity_pct_fs", "hysteresis_pct_fs", "zero_return_N", "verdict"])
        w.writerow([stamp, tag, f"{offset:.0f}", f"{res['scale']:.3f}",
                    f"{final_scale:.3f}" if final_scale else "", f"{res['max_load_N']:.1f}",
                    f"{res['max_error_pct_fs']:.3f}", f"{res.get('nonlinearity_pct_fs', float('nan')):.3f}",
                    f"{res.get('hysteresis_pct_fs', float('nan')):.3f}",
                    f"{res.get('zero_return_N', float('nan')):.3f}", verdict or ""])
    print(f"\nwrote {pts_path.relative_to(ROOT) if pts_path.is_relative_to(ROOT) else pts_path}")
    print(f"history appended to {hist.relative_to(ROOT) if hist.is_relative_to(ROOT) else hist}")
    return 0 if verdict != "FAIL" else 3


if __name__ == "__main__":
    sys.exit(main())
