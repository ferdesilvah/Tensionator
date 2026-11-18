"""Simulated rig: a serial-port look-alike for testing the dashboard without hardware.

Models the rope as a linear spring with slack plus slow logarithmic creep, and mimics
the firmware's commands and line formats closely enough to exercise the GUI, logging
and reconnect paths. It is NOT a validation of the firmware's control logic.
"""
from __future__ import annotations

import math
import random
import time

import serial

from rig_protocol import frame, split_line, ParseError


class SimDevice:
    def __init__(self, drop_every_s: float = 0.0, rate_hz: float = 10.0):
        self.drop_every_s = drop_every_s
        self.period = 1.0 / rate_hz
        self.t0 = time.monotonic()
        self.opened_at = self.t0
        self.out = bytearray()
        self.inbuf = b""
        self.last = self.t0
        self.seq = 0
        self.pos = 0.0          # mm, + = tighten
        self.vel = 0.0
        self.slack = 5.0        # mm of slack before the rope loads
        self.k = 120.0          # N/mm
        self.creep = 0.0
        self.break_at = random.uniform(850, 1000)
        self.mode = "IDLE"
        self.fault = "NONE"
        self.sp = 0.0
        self.target = 0.0
        self.rate = 0.0
        self.move_to = 0.0
        self.cy = None
        self.broken = False
        self._emit("E", f"{self._ms()},BOOT,sim,0,1")

    # ---- serial.Serial-like API ------------------------------------------------
    @property
    def in_waiting(self) -> int:
        self._advance()
        return len(self.out)

    def read(self, n: int = 1) -> bytes:
        self._advance()
        if not self.out:
            time.sleep(0.02)
            self._advance()
        b, self.out = bytes(self.out[:n]), self.out[n:]
        return b

    def write(self, data: bytes) -> int:
        self._check_drop()
        self.inbuf += data
        while b"\n" in self.inbuf:
            line, self.inbuf = self.inbuf.split(b"\n", 1)
            self._command(line.decode("ascii", "replace").strip())
        return len(data)

    def close(self) -> None:
        pass

    # ---- internals -------------------------------------------------------------
    def _check_drop(self):
        if self.drop_every_s and time.monotonic() - self.opened_at > self.drop_every_s:
            raise serial.SerialException("simulated cable unplug")

    def _ms(self) -> int:
        return int((time.monotonic() - self.t0) * 1000)

    def _emit(self, typ: str, rest: str):
        self.out += frame(f"{typ},{rest}").encode()

    def _force(self) -> float:
        if self.broken:
            return random.gauss(0, 0.3)
        ext = self.pos - self.slack - self.creep
        return max(0.0, self.k * ext) + random.gauss(0, 0.3)

    def _end(self, reason: str, peak: float = 0.0):
        self._emit("E", f"{self._ms()},END,{self.mode},{reason},{peak:.2f},{self.pos:.4f}")
        self.mode, self.vel = "IDLE", 0.0

    def _advance(self):
        self._check_drop()
        now = time.monotonic()
        while now - self.last >= self.period:
            self.last += self.period
            self._tick(self.period)

    def _tick(self, dt: float):
        self.pos += self.vel * dt
        f = self._force()
        if f > 50:
            self.creep += 2e-4 * dt * f / 100.0   # crude creep
        if f > self.break_at and not self.broken:
            self.broken = True
            self._emit("E", f"{self._ms()},BREAK,{self.mode},{f:.2f},{self.pos:.4f},0.00")
            self._end("BREAK", f)
            f = self._force()
        if f > 1200 and self.fault == "NONE":
            self.fault = "OVERFORCE"
            self._emit("E", f"{self._ms()},FAULT,OVERFORCE,{self.mode},{f:.2f},{self.pos:.4f}")
            self.mode, self.vel = "IDLE", 0.0
        m = self.mode
        if m == "HOLD":
            step = self.rate * dt
            self.sp = min(self.sp + step, self.target) if self.sp < self.target else max(self.sp - step, self.target)
            self.vel = max(-2.0, min(2.0, 0.005 * (self.sp - f)))
        elif m == "MOVE":
            rem = self.move_to - self.pos
            if abs(rem) < 0.01:
                self._end("DONE")
            else:
                self.vel = math.copysign(min(2.0, abs(rem) * 5), rem)
        elif m == "CYCLE":
            lo, hi, v, n = self.cy["lo"], self.cy["hi"], self.cy["v"], self.cy["n"]
            if self.cy["up"]:
                self.cy["pk"] = max(self.cy["pk"], f)
                self.vel = v
                if f >= hi:
                    self.cy["up"], self.cy["vl"] = False, f
            else:
                self.cy["vl"] = min(self.cy["vl"], f)
                self.vel = -v
                if f <= lo:
                    self.cy["done"] += 1
                    self._emit("E", f"{self._ms()},CYCLE,{self.cy['done']},{self.cy['pk']:.2f},{self.cy['vl']:.2f},{self.pos:.4f}")
                    self.cy["up"], self.cy["pk"] = True, f
                    if n and self.cy["done"] >= n:
                        self._end("DONE")
        self.seq += 1
        if m == "HOLD":
            sp = self.sp
        elif m == "CYCLE":
            sp = self.cy["hi"] if self.cy["up"] else self.cy["lo"]
        else:
            sp = 0.0
        self._emit("D", f"{self._ms()},{self.seq},{f:.2f},{int(f * 2000)},{self.pos:.4f},{self.vel:.4f},{m},{sp:.1f},{self.fault}")

    def _command(self, line: str):
        if "*" in line:
            try:
                line = ",".join(split_line(line))
            except ParseError:
                self._emit("X", "?,checksum")
                return
        tok = line.replace(",", " ").upper().split()
        if not tok:
            return
        c, a = tok[0], [float(x) for x in tok[1:] if _isnum(x)]
        f = self._force()
        if c in ("STOP", "ESTOP", "OFF"):
            if self.mode != "IDLE":
                self._end("STOPPED")
        elif c == "CLR":
            self.fault = "NONE"
        elif c == "HOLD" and a:
            if self.mode != "HOLD":
                self.sp = f
            self.mode, self.target, self.rate = "HOLD", a[0], a[1] if len(a) > 1 else 20.0
        elif c == "RAMP" and a:
            self.mode, self.vel = "RAMP", a[0] / 60.0
        elif c == "MOVE" and a:
            self.mode, self.move_to = "MOVE", self.pos + a[0]
        elif c == "CYCLE" and len(a) >= 4:
            self.mode = "CYCLE"
            self.cy = dict(lo=a[0], hi=a[1], v=a[2] / 60.0, n=int(a[3]), up=True, pk=f, vl=f, done=0)
        elif c == "ZERO":
            self.pos = 0.0
        elif c == "GET":
            for k, v in (("fw", "sim"), ("hardfmax", "1200.0"), ("fmax", "1100.0"), ("hz", "10.0")):
                self._emit("P", f"{k},{v}")
        elif c == "RESETSIM":                    # sim-only: re-tie a fresh rope
            self.broken, self.pos, self.creep = False, 0.0, 0.0
        elif c not in ("PING", "ON", "TARE", "CAL", "SET", "SAVE", "RAMPF", "DEFAULTS"):
            self._emit("X", f"{c},unknown command")
            return
        self._emit("K", c)


def _isnum(s: str) -> bool:
    try:
        float(s)
        return True
    except ValueError:
        return False
