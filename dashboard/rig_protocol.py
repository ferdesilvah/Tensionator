"""Line protocol shared by the dashboard, simulator and tests.

Device -> host lines:  <TYPE>,<fields...>*HH\n   (HH = XOR of all chars before '*')
  D  data   : ms, seq, force_N, raw, pos_mm, vel_mm_s, mode, setpoint_N, fault
  E  event  : ms, NAME, args...
  K  ack    : CMD
  X  error  : CMD, reason
  P  param  : key, value
Host -> device: "CMD arg arg*HH\n" (checksum optional for hand-typed commands).
"""
from __future__ import annotations

from dataclasses import dataclass

DATA_FIELDS = ("dev_ms", "seq", "force_N", "raw", "pos_mm", "vel_mm_s", "mode", "setpoint_N", "fault")


def checksum(body: str) -> str:
    cs = 0
    for ch in body.encode("ascii", "replace"):
        cs ^= ch
    return f"{cs:02X}"


def frame(body: str) -> str:
    return f"{body}*{checksum(body)}\n"


def command(text: str) -> bytes:
    """Host command with checksum appended."""
    return frame(text.strip()).encode("ascii")


@dataclass
class Sample:
    dev_ms: int
    seq: int
    force: float
    raw: int
    pos: float
    vel: float
    mode: str
    setpoint: float
    fault: str


class ParseError(ValueError):
    pass


def split_line(line: str) -> list[str]:
    """Validate checksum and return comma-separated fields (first = type)."""
    line = line.strip()
    star = line.rfind("*")
    if star < 1 or len(line) != star + 3:
        raise ParseError("no checksum")
    body, cs = line[:star], line[star + 1:]
    if checksum(body) != cs.upper():
        raise ParseError("bad checksum")
    return body.split(",")


def parse_sample(fields: list[str]) -> Sample:
    if fields[0] != "D" or len(fields) != 10:
        raise ParseError("not a data line")
    try:
        return Sample(int(fields[1]), int(fields[2]), float(fields[3]), int(fields[4]),
                      float(fields[5]), float(fields[6]), fields[7], float(fields[8]), fields[9])
    except ValueError as e:
        raise ParseError(str(e)) from e
