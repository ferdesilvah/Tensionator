"""Dyneema capstan rig dashboard.

    python rig_dashboard.py            # pick the COM port in the GUI
    python rig_dashboard.py --sim      # simulated rig (no hardware)
    python rig_dashboard.py --sim --sim-drop 20   # sim that "unplugs" every 20 s

Architecture
  Link thread  : owns the serial port, reconnects automatically, validates checksums,
                 and writes the CSV log itself — logging never waits on the GUI.
  GUI (Tk)     : drains a queue every 100 ms, redraws plots every 300 ms.
Dependencies: pyserial, numpy, matplotlib (tkinter ships with Python).
"""
from __future__ import annotations

import argparse
import csv
import os
import queue
import threading
import time
import tkinter as tk
from datetime import datetime, timezone
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import numpy as np
import serial
from serial.tools import list_ports
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402

from rig_protocol import DATA_FIELDS, ParseError, Sample, command, parse_sample, split_line

BAUD = 115200
SIM_PORT = "SIM"
STALL_PING_S = 2.0       # no data this long -> send PING
STALL_RECONNECT_S = 5.0  # no data this long -> close and reopen the port
TEST_MODES = {"HOLD", "RAMP", "RAMPF", "CYCLE"}


def iso_now(t: float) -> str:
    return datetime.fromtimestamp(t, timezone.utc).isoformat(timespec="milliseconds")


# =============================================================================== logging
class CsvLogger:
    """Thread-safe CSV writer: <stem>.csv (samples) and <stem>_events.csv."""

    def __init__(self):
        self.lock = threading.Lock()
        self.f = self.ef = None
        self.w = self.ew = None
        self.path: Path | None = None
        self.rows = 0
        self.t_start = 0.0
        self._last_flush = self._last_sync = 0.0

    @property
    def active(self) -> bool:
        return self.f is not None

    def start(self, folder: Path, tag: str, meta: dict[str, str]) -> Path:
        with self.lock:
            self._close_locked()
            folder.mkdir(parents=True, exist_ok=True)
            stem = datetime.now().strftime("%Y%m%d_%H%M%S") + (f"_{tag}" if tag else "")
            self.path = folder / f"{stem}.csv"
            self.f = open(self.path, "w", newline="", encoding="utf-8")
            self.ef = open(folder / f"{stem}_events.csv", "w", newline="", encoding="utf-8")
            self.w, self.ew = csv.writer(self.f), csv.writer(self.ef)
            self.w.writerow(["host_time_utc", "host_unix_s", "elapsed_s", *DATA_FIELDS])
            self.ew.writerow(["host_time_utc", "host_unix_s", "kind", "detail"])
            self.t_start = time.time()
            for k, v in meta.items():
                self.ew.writerow([iso_now(self.t_start), f"{self.t_start:.3f}", "META", f"{k}={v}"])
            self.rows = 0
            return self.path

    def sample(self, t: float, s: Sample) -> None:
        with self.lock:
            if not self.w:
                return
            self.w.writerow([iso_now(t), f"{t:.3f}", f"{t - self.t_start:.3f}", s.dev_ms, s.seq,
                             f"{s.force:.2f}", s.raw, f"{s.pos:.4f}", f"{s.vel:.4f}", s.mode,
                             f"{s.setpoint:.1f}", s.fault])
            self.rows += 1
            self._maybe_flush(t)

    def event(self, t: float, kind: str, detail: str) -> None:
        with self.lock:
            if not self.ew:
                return
            self.ew.writerow([iso_now(t), f"{t:.3f}", kind, detail])
            self.ef.flush()

    def _maybe_flush(self, t: float) -> None:
        if t - self._last_flush > 1.0:
            self.f.flush()
            self._last_flush = t
        if t - self._last_sync > 10.0:   # survive a PC crash / power cut mid-creep
            os.fsync(self.f.fileno())
            os.fsync(self.ef.fileno())
            self._last_sync = t

    def stop(self) -> None:
        with self.lock:
            self._close_locked()

    def _close_locked(self) -> None:
        for fh in (self.f, self.ef):
            if fh:
                fh.flush()
                os.fsync(fh.fileno())
                fh.close()
        self.f = self.ef = self.w = self.ew = None


# =============================================================================== serial link
class Link(threading.Thread):
    """Owns the port. Posts (kind, host_time, payload) tuples to `inbox`."""

    def __init__(self, inbox: queue.Queue, logger: CsvLogger, sim_drop: float = 0.0):
        super().__init__(daemon=True)
        self.inbox, self.logger, self.sim_drop = inbox, logger, sim_drop
        self.port: str | None = None
        self.serial_number: str | None = None   # lets us find the board if its COM number changes
        self.want = False
        self.connected = False
        self.cmds: queue.Queue[str] = queue.Queue()
        self.ser = None
        self.bad_lines = 0
        self.seq_gaps = 0
        self._last_seq: int | None = None
        self._last_ms: int | None = None
        self._halt = threading.Event()

    # ---- called from the GUI thread ----
    def connect(self, port: str) -> None:
        self.port, self.serial_number, self.want = port, None, True

    def disconnect(self) -> None:
        self.want = False

    def send(self, text: str) -> bool:
        if not self.connected:
            return False            # never queue commands for "later": stale commands are unsafe
        self.cmds.put(text)
        return True

    def shutdown(self) -> None:
        self._halt.set()

    # ---- thread body ----
    def _post(self, kind: str, payload) -> None:
        self.inbox.put((kind, time.time(), payload))

    def _status(self, text: str) -> None:
        self._post("status", text)

    def _resolve_port(self) -> str | None:
        if self.port == SIM_PORT:
            return SIM_PORT
        ports = list(list_ports.comports())
        for p in ports:
            if p.device == self.port:
                if p.serial_number:
                    self.serial_number = p.serial_number
                if not self.serial_number or p.serial_number == self.serial_number:
                    return p.device
        if self.serial_number:                       # re-enumerated under another COM number
            for p in ports:
                if p.serial_number == self.serial_number:
                    return p.device
        return None

    def _open(self) -> bool:
        dev = self._resolve_port()
        if dev is None:
            self._status(f"waiting for {self.port}")
            return False
        try:
            if dev == SIM_PORT:
                from rig_sim import SimDevice
                self.ser = SimDevice(drop_every_s=self.sim_drop)
            else:
                s = serial.Serial()
                s.port, s.baudrate, s.timeout, s.write_timeout = dev, BAUD, 0.1, 1.0
                # UNO R4 (native USB): opening the port never resets it (only a 1200-baud
                # "touch" does, which we never do). Assert DTR so the USB CDC link is "open".
                s.dtr = True
                s.rts = False
                s.open()
                self.ser = s
        except (serial.SerialException, OSError) as e:
            self._status(f"open {dev} failed: {e}")
            self.ser = None
            return False
        self.connected = True
        self._last_seq = None
        self._status(f"connected {dev}")
        self.logger.event(time.time(), "LINK", f"connected {dev}")
        self.cmds.put("GET")
        return True

    def _close(self, why: str) -> None:
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:  # noqa: BLE001 — closing a dead port can raise anything
                pass
        was = self.connected
        self.ser, self.connected = None, False
        while not self.cmds.empty():                 # drop anything not yet sent
            self._post("log", f"not sent (link down): {self.cmds.get_nowait()}")
        if was:
            self.logger.event(time.time(), "LINK", f"lost: {why}")
        self._status(f"disconnected: {why}" if not self.want else f"reconnecting ({why})")

    def run(self) -> None:
        buf = b""
        last_rx = time.monotonic()
        pinged = False
        while not self._halt.is_set():
            if not self.want:
                if self.ser is not None:
                    self._close("by user")
                time.sleep(0.1)
                continue
            if self.ser is None:
                if not self._open():
                    time.sleep(1.0)
                    continue
                buf, last_rx, pinged = b"", time.monotonic(), False
            try:
                while not self.cmds.empty():
                    txt = self.cmds.get_nowait()
                    self.ser.write(command(txt))
                    self._post("sent", txt)
                    self.logger.event(time.time(), "CMD", txt)
                n = self.ser.in_waiting
                data = self.ser.read(n if n else 1)
            except (serial.SerialException, OSError) as e:
                self._close(str(e).splitlines()[0] if str(e) else type(e).__name__)
                time.sleep(0.5)
                continue
            now = time.monotonic()
            if data:
                last_rx, pinged = now, False
                buf += data
                *lines, buf = buf.split(b"\n")
                for ln in lines:
                    try:
                        self._on_line(ln.decode("ascii", "replace"))
                    except Exception as e:  # noqa: BLE001 — one bad line must never kill the link thread
                        self.bad_lines += 1
                        self._post("log", f"line handler error: {e!r}")
                if len(buf) > 4096:
                    buf = b""
            elif now - last_rx > STALL_RECONNECT_S:
                self._close(f"no data for {STALL_RECONNECT_S:.0f} s")
            elif now - last_rx > STALL_PING_S and not pinged:
                self.cmds.put("PING")
                pinged = True
        self._close("shutdown")

    def _on_line(self, line: str) -> None:
        line = line.strip()
        if not line:
            return
        t = time.time()
        try:
            f = split_line(line)
        except ParseError:
            self.bad_lines += 1          # partial line after (re)connect, or corruption
            return
        typ = f[0]
        if typ == "D":
            try:
                s = parse_sample(f)
            except ParseError:
                self.bad_lines += 1
                return
            if self._last_ms is not None and s.dev_ms + 1000 < self._last_ms:
                # device clock went backwards: it rebooted (its BOOT line may have been
                # sent before we reopened the port)
                self.logger.event(t, "DEV", f"REBOOT_DETECTED,uptime_ms={s.dev_ms}")
                self._post("event", ["REBOOT_DETECTED", f"uptime {s.dev_ms} ms"])
                self._last_seq = None
            self._last_ms = s.dev_ms
            if self._last_seq is not None and s.seq > self._last_seq + 1:
                self.seq_gaps += s.seq - self._last_seq - 1
            self._last_seq = s.seq
            self.logger.sample(t, s)
            self._post("sample", s)
        elif typ == "E":
            self._last_seq = None if len(f) > 2 and f[2] == "BOOT" else self._last_seq
            self.logger.event(t, "DEV", ",".join(f[2:]))
            self._post("event", f[2:])
        elif typ == "K":
            self._post("ack", f[1:])
        elif typ == "X":
            self.logger.event(t, "ERR", ",".join(f[1:]))
            self._post("err", f[1:])
        elif typ == "P" and len(f) >= 3:
            self._post("param", (f[1], f[2]))


# =============================================================================== plot data
class Series:
    """Growable numpy columns: host time (s since t0), force, position, setpoint."""

    def __init__(self):
        self.clear()

    def clear(self) -> None:
        self.n = 0
        self.a = np.empty((4, 4096))

    def add(self, t: float, f: float, p: float, sp: float) -> None:
        if self.n == self.a.shape[1]:
            self.a = np.concatenate([self.a, np.empty_like(self.a)], axis=1)
        self.a[:, self.n] = (t, f, p, sp)
        self.n += 1

    def view(self) -> np.ndarray:
        return self.a[:, :self.n]


def envelope(x: np.ndarray, ys: list[np.ndarray], max_pts: int = 3000):
    """Min/max decimation so short spikes (e.g. a break peak) survive downsampling."""
    n = len(x)
    if n <= max_pts:
        return x, ys
    b = int(np.ceil(n / (max_pts // 2)))
    m = (n // b) * b
    xo = np.repeat(x[:m:b], 2)
    out = []
    for y in ys:
        r = y[:m].reshape(-1, b)
        out.append(np.column_stack([r.min(axis=1), r.max(axis=1)]).ravel())
    return xo, out


# =============================================================================== GUI
class App:
    def __init__(self, root: tk.Tk, sim: bool, sim_drop: float):
        self.root = root
        root.title("Dyneema rig")
        root.geometry("1400x860")
        self.inbox: queue.Queue = queue.Queue()
        self.logger = CsvLogger()
        self.link = Link(self.inbox, self.logger, sim_drop)
        self.link.start()
        self.data = Series()
        self.t0 = time.time()
        self.last: Sample | None = None
        self.params: dict[str, str] = {}
        self.hold_deadline: float | None = None
        self.rate_t, self.rate_n, self.rate_hz = time.monotonic(), 0, 0.0

        self.out_dir = tk.StringVar(value=str(Path(__file__).resolve().parent / "data"))
        self.auto_rec = tk.BooleanVar(value=True)
        self._build()
        if sim:
            self.port_var.set(SIM_PORT)
            self._connect()
        root.protocol("WM_DELETE_WINDOW", self._on_close)
        root.after(100, self._poll)
        root.after(300, self._redraw)

    # ---------------------------------------------------------------- layout
    def _build(self) -> None:
        top = ttk.Frame(self.root, padding=4)
        top.pack(fill="x")
        ttk.Label(top, text="Port").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(top, textvariable=self.port_var, width=28)
        self.port_cb.pack(side="left", padx=2)
        ttk.Button(top, text="↻", width=3, command=self._refresh_ports).pack(side="left")
        self.conn_btn = ttk.Button(top, text="Connect", command=self._toggle_connect)
        self.conn_btn.pack(side="left", padx=4)
        self.link_lbl = ttk.Label(top, text="disconnected", width=40)
        self.link_lbl.pack(side="left", padx=6)
        self.stats_lbl = ttk.Label(top, text="")
        self.stats_lbl.pack(side="right")
        self._refresh_ports()

        # readouts
        ro = ttk.Frame(self.root, padding=(6, 0))
        ro.pack(fill="x")
        big = ("Segoe UI", 22, "bold")
        self.f_lbl = ttk.Label(ro, text="---.-- N", font=big, width=12)
        self.f_lbl.pack(side="left")
        self.p_lbl = ttk.Label(ro, text="---.---- mm", font=("Segoe UI", 16), width=14)
        self.p_lbl.pack(side="left", padx=10)
        self.mode_lbl = ttk.Label(ro, text="mode: -", font=("Segoe UI", 14), width=16)
        self.mode_lbl.pack(side="left")
        self.fault_lbl = tk.Label(ro, text="", font=("Segoe UI", 14, "bold"), fg="white")
        self.fault_lbl.pack(side="left", padx=6)
        self.hold_lbl = ttk.Label(ro, text="", font=("Segoe UI", 12))
        self.hold_lbl.pack(side="left", padx=10)
        self.rec_lbl = ttk.Label(ro, text="not recording", font=("Segoe UI", 11))
        self.rec_lbl.pack(side="right")

        body = ttk.PanedWindow(self.root, orient="horizontal")
        body.pack(fill="both", expand=True)
        left = ttk.Frame(body, padding=4, width=380)
        right = ttk.Frame(body)
        body.add(left, weight=0)
        body.add(right, weight=1)
        self._build_controls(left)
        self._build_plot(right)

    def _entry(self, parent, label: str, default: str, row: int, width: int = 9) -> tk.StringVar:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=1)
        v = tk.StringVar(value=default)
        ttk.Entry(parent, textvariable=v, width=width).grid(row=row, column=1, sticky="w", pady=1)
        return v

    def _build_controls(self, left: ttk.Frame) -> None:
        safety = ttk.LabelFrame(left, text="Safety", padding=4)
        safety.pack(fill="x")
        tk.Button(safety, text="STOP", bg="#c62828", fg="white", font=("Segoe UI", 14, "bold"),
                  command=lambda: self._send("STOP")).grid(row=0, column=0, rowspan=2, sticky="nsew", padx=2)
        ttk.Button(safety, text="E-stop (no decel)", command=lambda: self._send("ESTOP")).grid(row=0, column=1, sticky="ew")
        ttk.Button(safety, text="Clear fault", command=lambda: self._send("CLR")).grid(row=1, column=1, sticky="ew")
        ttk.Button(safety, text="Motor ON", command=lambda: self._send("ON")).grid(row=0, column=2, sticky="ew")
        ttk.Button(safety, text="Motor OFF…", command=self._motor_off).grid(row=1, column=2, sticky="ew")
        safety.columnconfigure(0, weight=1)

        nb = ttk.Notebook(left)
        nb.pack(fill="x", pady=4)

        # --- creep hold
        h = ttk.Frame(nb, padding=6)
        nb.add(h, text="Hold")
        self.h_sp = self._entry(h, "Setpoint (N)", "500", 0)
        self.h_rate = self._entry(h, "Approach rate (N/s)", "20", 1)
        self.h_dur = self._entry(h, "Duration (min, 0 = ∞)", "60", 2)
        ttk.Button(h, text="Start / update hold", command=self._start_hold).grid(row=3, column=0, columnspan=2, sticky="ew", pady=4)
        ttk.Label(h, text="Duration is timed by this PC. If the PC or link\n"
                          "drops, the rig keeps holding until STOP.", foreground="#666").grid(row=4, column=0, columnspan=2, sticky="w")

        # --- ramp
        r = ttk.Frame(nb, padding=6)
        nb.add(r, text="Ramp")
        self.r_rate = self._entry(r, "Displacement rate (mm/min)", "50", 0)
        ttk.Button(r, text="Start displacement ramp", command=self._start_ramp).grid(row=1, column=0, columnspan=2, sticky="ew", pady=4)
        self.rf_rate = self._entry(r, "Force rate (N/s)", "10", 2)
        ttk.Button(r, text="Start force-rate ramp", command=self._start_rampf).grid(row=3, column=0, columnspan=2, sticky="ew", pady=4)
        ttk.Label(r, text="Ends on break (drop below brkdrop × peak)\nor at fmax.", foreground="#666").grid(row=4, column=0, columnspan=2, sticky="w")

        # --- cycle
        c = ttk.Frame(nb, padding=6)
        nb.add(c, text="Cycle")
        self.c_lo = self._entry(c, "Low force (N)", "100", 0)
        self.c_hi = self._entry(c, "High force (N)", "600", 1)
        self.c_rate = self._entry(c, "Rate (mm/min)", "60", 2)
        self.c_n = self._entry(c, "Cycles (0 = ∞)", "100", 3)
        self.c_dwell = self._entry(c, "Dwell at limits (s)", "0", 4)
        ttk.Button(c, text="Start cycling", command=self._start_cycle).grid(row=5, column=0, columnspan=2, sticky="ew", pady=4)

        # --- setup
        s = ttk.Frame(nb, padding=6)
        nb.add(s, text="Setup")
        self.m_dist = self._entry(s, "Move (mm, + tightens)", "5", 0)
        self.m_speed = self._entry(s, "Speed (mm/s)", "2", 1)
        bf = ttk.Frame(s)
        bf.grid(row=2, column=0, columnspan=2, sticky="ew")
        ttk.Button(bf, text="Tighten", command=lambda: self._move(+1)).pack(side="left", expand=True, fill="x")
        ttk.Button(bf, text="Loosen", command=lambda: self._move(-1)).pack(side="left", expand=True, fill="x")
        ttk.Button(bf, text="Zero pos", command=lambda: self._send("ZERO")).pack(side="left", expand=True, fill="x")
        ttk.Separator(s).grid(row=3, column=0, columnspan=2, sticky="ew", pady=6)
        ttk.Button(s, text="Tare (rope slack!)", command=self._tare).grid(row=4, column=0, columnspan=2, sticky="ew")
        self.cal_n = self._entry(s, "Known load (N)", "98.07", 5)
        ttk.Button(s, text="Calibrate with known load", command=self._cal).grid(row=6, column=0, columnspan=2, sticky="ew")
        ttk.Separator(s).grid(row=7, column=0, columnspan=2, sticky="ew", pady=6)
        ttk.Label(s, text="Parameter").grid(row=8, column=0, sticky="w")
        self.p_key = ttk.Combobox(s, width=9, values=["fmax", "kp", "ki", "vhold", "vload", "accel", "travel",
                                                       "brkdrop", "brkmin", "reff", "dirinv", "stream"])
        self.p_key.grid(row=8, column=1, sticky="w")
        self.p_val = self._entry(s, "Value", "", 9)
        pf = ttk.Frame(s)
        pf.grid(row=10, column=0, columnspan=2, sticky="ew")
        ttk.Button(pf, text="Set", command=self._set_param).pack(side="left", expand=True, fill="x")
        ttk.Button(pf, text="Get all", command=lambda: self._send("GET")).pack(side="left", expand=True, fill="x")
        ttk.Button(pf, text="Save to EEPROM", command=lambda: self._send("SAVE")).pack(side="left", expand=True, fill="x")
        ttk.Separator(s).grid(row=11, column=0, columnspan=2, sticky="ew", pady=6)
        bm = ttk.Frame(s)
        bm.grid(row=12, column=0, columnspan=2, sticky="ew")
        ttk.Button(bm, text="Bench mode (no load cell)…", command=self._bench_on).pack(side="left", expand=True, fill="x")
        ttk.Button(bm, text="Exit bench mode", command=lambda: self._send("NOCELL 0")).pack(side="left", expand=True, fill="x")

        # --- logging
        lg = ttk.LabelFrame(left, text="Logging", padding=4)
        lg.pack(fill="x", pady=4)
        ttk.Entry(lg, textvariable=self.out_dir, width=34).grid(row=0, column=0, sticky="ew")
        ttk.Button(lg, text="…", width=3, command=self._browse).grid(row=0, column=1)
        self.tag_var = tk.StringVar(value="")
        ttk.Label(lg, text="Sample tag (e.g. bowline_3mm_s1)").grid(row=1, column=0, sticky="w")
        ttk.Entry(lg, textvariable=self.tag_var, width=34).grid(row=2, column=0, columnspan=2, sticky="ew")
        self.rec_btn = ttk.Button(lg, text="Start recording", command=self._toggle_rec)
        self.rec_btn.grid(row=3, column=0, columnspan=2, sticky="ew", pady=2)
        ttk.Checkbutton(lg, text="Auto-start recording when a test starts", variable=self.auto_rec).grid(row=4, column=0, columnspan=2, sticky="w")

        # --- console
        con = ttk.LabelFrame(left, text="Console", padding=4)
        con.pack(fill="both", expand=True)
        self.console = tk.Text(con, height=12, width=46, font=("Consolas", 9), state="disabled", wrap="word")
        self.console.pack(fill="both", expand=True)
        self.console.tag_configure("err", foreground="#c62828")
        self.console.tag_configure("ev", foreground="#1565c0")
        self.cmd_var = tk.StringVar()
        e = ttk.Entry(con, textvariable=self.cmd_var)
        e.pack(fill="x")
        e.bind("<Return>", lambda _e: (self._send(self.cmd_var.get()), self.cmd_var.set("")))

    def _build_plot(self, right: ttk.Frame) -> None:
        bar = ttk.Frame(right)
        bar.pack(fill="x")
        ttk.Label(bar, text="Window").pack(side="left")
        self.win_var = tk.StringVar(value="10 min")
        ttk.Combobox(bar, textvariable=self.win_var, width=8, state="readonly",
                     values=["1 min", "10 min", "60 min", "All"]).pack(side="left", padx=4)
        ttk.Label(bar, text="View").pack(side="left")
        self.view_var = tk.StringVar(value="vs time")
        ttk.Combobox(bar, textvariable=self.view_var, width=16, state="readonly",
                     values=["vs time", "force vs position"]).pack(side="left", padx=4)
        self.pause_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(bar, text="Freeze", variable=self.pause_var).pack(side="left", padx=6)
        ttk.Button(bar, text="Clear plot", command=self._clear_plot).pack(side="left")

        self.fig = Figure(figsize=(8, 6), dpi=100, layout="constrained")
        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self._make_axes("vs time")

    def _make_axes(self, view: str) -> None:
        self._view_mode = view
        self.fig.clear()
        if view == "vs time":
            self.ax_f = self.fig.add_subplot(211)
            self.ax_p = self.fig.add_subplot(212, sharex=self.ax_f)
            self.ax_p.set_ylabel("line travel (mm)")
            self.ax_p.set_xlabel("time (s)")
            self.axes = (self.ax_f, self.ax_p)
        else:
            self.ax_f = self.fig.add_subplot(111)
            self.ax_f.set_xlabel("line travel at drum (mm)")
            self.ax_p = None
            self.axes = (self.ax_f,)
        self.ln_f, = self.ax_f.plot([], [], lw=1.0, color="#1f77b4", label="force")
        self.ln_sp, = self.ax_f.plot([], [], lw=0.8, color="#ff7f0e", ls="--", label="setpoint")
        self.ln_p = self.ax_p.plot([], [], lw=1.0, color="#2ca02c")[0] if self.ax_p else None
        self.ax_f.set_ylabel("force (N)")
        self.ax_f.legend(loc="upper left", fontsize=8)
        for ax in self.axes:
            ax.grid(True, alpha=0.3)

    # ---------------------------------------------------------------- actions
    def _log(self, text: str, tag: str = "") -> None:
        self.console.configure(state="normal")
        self.console.insert("end", time.strftime("%H:%M:%S ") + text + "\n", tag)
        if int(self.console.index("end-1c").split(".")[0]) > 2000:
            self.console.delete("1.0", "500.0")
        self.console.see("end")
        self.console.configure(state="disabled")

    def _send(self, text: str) -> bool:
        text = text.strip()
        if not text:
            return False
        if not self.link.send(text):
            self._log(f"not connected — '{text}' not sent", "err")
            return False
        return True

    def _num(self, var: tk.StringVar, name: str) -> float | None:
        try:
            return float(var.get())
        except ValueError:
            messagebox.showerror("Input", f"{name}: not a number")
            return None

    def _refresh_ports(self) -> None:
        ports = [p.device for p in list_ports.comports()]
        self.port_cb["values"] = ports + [SIM_PORT]
        if not self.port_var.get() and ports:
            self.port_var.set(ports[0])

    def _toggle_connect(self) -> None:
        if self.link.want:
            self.link.disconnect()
            self.conn_btn.configure(text="Connect")
        else:
            self._connect()

    def _connect(self) -> None:
        port = self.port_var.get().split(" ")[0]
        if not port:
            return
        self.link.connect(port)
        self.conn_btn.configure(text="Disconnect")

    def _motor_off(self) -> None:
        if self.last and abs(self.last.force) > 20 and not messagebox.askyesno(
                "Motor OFF", f"Rope is loaded ({self.last.force:.0f} N).\nThe gearbox will back-drive and "
                             "dump the load uncontrolled.\nUse 'Loosen' to unload instead.\n\nDe-energise anyway?"):
            return
        self._send("OFF")

    def _bench_on(self) -> None:
        if messagebox.askyesno(
                "Bench mode", "Bench mode runs the motor with NO force sensing and NO overload protection.\n"
                              "Only MOVE is allowed. Detach the rope (or leave it slack and free).\n\n"
                              "Enter bench mode?"):
            self._send("NOCELL 1")

    def _maybe_autorec(self, test: str) -> None:
        if self.auto_rec.get() and not self.logger.active:
            self._start_rec(test)

    def _start_hold(self) -> None:
        sp, rate, dur = self._num(self.h_sp, "setpoint"), self._num(self.h_rate, "rate"), self._num(self.h_dur, "duration")
        if None in (sp, rate, dur):
            return
        if self._send(f"HOLD {sp:g} {rate:g}"):
            self._maybe_autorec("hold")
            self.hold_deadline = time.time() + dur * 60 if dur and dur > 0 else None
            if self.logger.active:
                self.logger.event(time.time(), "UI", f"hold {sp:g} N, {rate:g} N/s, {dur:g} min")

    def _start_ramp(self) -> None:
        v = self._num(self.r_rate, "rate")
        if v is not None and self._send(f"RAMP {v:g}"):
            self._maybe_autorec("ramp")

    def _start_rampf(self) -> None:
        v = self._num(self.rf_rate, "rate")
        if v is not None and self._send(f"RAMPF {v:g}"):
            self._maybe_autorec("rampf")

    def _start_cycle(self) -> None:
        vals = [self._num(v, n) for v, n in ((self.c_lo, "low"), (self.c_hi, "high"), (self.c_rate, "rate"),
                                             (self.c_n, "cycles"), (self.c_dwell, "dwell"))]
        if None in vals:
            return
        lo, hi, rate, n, dwell = vals
        if self._send(f"CYCLE {lo:g} {hi:g} {rate:g} {int(n)} {dwell:g}"):
            self._maybe_autorec("cycle")

    def _move(self, sign: int) -> None:
        d, v = self._num(self.m_dist, "distance"), self._num(self.m_speed, "speed")
        if d is not None and v is not None:
            self._send(f"MOVE {sign * abs(d):g} {v:g}")

    def _tare(self) -> None:
        if messagebox.askyesno("Tare", "Tare sets the current reading as 0 N and stores it.\n"
                                       "Is the rope completely slack / unloaded?"):
            self._send("TARE")

    def _cal(self) -> None:
        n = self._num(self.cal_n, "known load")
        if n is not None and messagebox.askyesno("Calibrate", f"Is exactly {n:g} N applied to the load cell now?"):
            self._send(f"CAL {n:g}")

    def _set_param(self) -> None:
        k, v = self.p_key.get().strip(), self.p_val.get().strip()
        if k and v:
            self._send(f"SET {k} {v}")

    def _browse(self) -> None:
        d = filedialog.askdirectory(initialdir=self.out_dir.get())
        if d:
            self.out_dir.set(d)

    def _toggle_rec(self) -> None:
        if self.logger.active:
            self.logger.stop()
            self._log(f"recording stopped ({self.logger.rows} rows)")
        else:
            self._start_rec("")

    def _start_rec(self, test: str) -> None:
        tag = "_".join(x for x in (self.tag_var.get().strip().replace(" ", "_"), test) if x)
        meta = {"port": self.port_var.get(), "tag": self.tag_var.get(), "test": test,
                **{f"param.{k}": v for k, v in self.params.items()}}
        try:
            path = self.logger.start(Path(self.out_dir.get()), tag, meta)
        except OSError as e:
            messagebox.showerror("Logging", f"Cannot open log file:\n{e}")
            return
        self._log(f"recording → {path}")

    def _clear_plot(self) -> None:
        self.data.clear()
        self.t0 = time.time()

    def _on_close(self) -> None:
        running = self.last is not None and self.last.mode in TEST_MODES and self.link.connected
        if running and not messagebox.askyesno(
                "Quit", f"A {self.last.mode} test is running on the rig.\nIt will keep running without logging.\n\nQuit anyway?"):
            return
        self.logger.stop()
        self.link.shutdown()
        self.root.after(300, self.root.destroy)

    # ---------------------------------------------------------------- periodic
    def _poll(self) -> None:
        try:
            for _ in range(5000):
                kind, t, payload = self.inbox.get_nowait()
                self._dispatch(kind, t, payload)
        except queue.Empty:
            pass
        now = time.time()
        if self.hold_deadline is not None:
            if self.last is None or self.last.mode != "HOLD":
                if self.last is not None and self.last.mode == "IDLE":
                    self.hold_deadline = None
                    self.hold_lbl.configure(text="")
            else:
                rem = self.hold_deadline - now
                if rem <= 0:
                    if self._send("STOP"):
                        self._log("hold duration elapsed → STOP", "ev")
                        self.logger.event(now, "UI", "hold duration elapsed, STOP sent")
                        self.hold_deadline = None
                else:
                    self.hold_lbl.configure(text=f"hold ends in {int(rem // 60)}:{int(rem % 60):02d}")
        mon = time.monotonic()
        if mon - self.rate_t >= 2.0:
            self.rate_hz = self.rate_n / (mon - self.rate_t)
            self.rate_t, self.rate_n = mon, 0
        self.stats_lbl.configure(text=f"{self.rate_hz:4.1f} Hz | bad lines {self.link.bad_lines} | "
                                      f"missed samples {self.link.seq_gaps}")
        if self.logger.active:
            self.rec_lbl.configure(text=f"● REC {self.logger.path.name}  {self.logger.rows} rows", foreground="#c62828")
            self.rec_btn.configure(text="Stop recording")
        else:
            self.rec_lbl.configure(text="not recording", foreground="#666")
            self.rec_btn.configure(text="Start recording")
        self.root.after(100, self._poll)

    def _dispatch(self, kind: str, t: float, payload) -> None:
        if kind == "sample":
            s: Sample = payload
            self.last = s
            self.rate_n += 1
            self.data.add(t - self.t0, s.force, s.pos, s.setpoint if s.mode in ("HOLD", "RAMPF", "CYCLE") else np.nan)
            self.f_lbl.configure(text=f"{s.force:8.2f} N")
            self.p_lbl.configure(text=f"{s.pos:9.4f} mm")
            self.mode_lbl.configure(text=f"mode: {s.mode}")
            if s.fault != "NONE":
                self.fault_lbl.configure(text=f" FAULT: {s.fault} ", bg="#c62828")
            else:
                self.fault_lbl.configure(text="", bg=self.root.cget("bg"))
        elif kind == "status":
            self.link_lbl.configure(text=payload)
            self._log(f"link: {payload}")
        elif kind == "event":
            name = payload[0] if payload else "?"
            self._log("EVENT " + ",".join(payload),
                      "err" if name in ("FAULT", "BREAK", "BOOT", "NORESUME", "REBOOT_DETECTED") else "ev")
            if name in ("BOOT", "REBOOT_DETECTED") and self.last is not None and self.last.mode in TEST_MODES:
                self._log(f"device rebooted during {self.last.mode}! (HOLD resumes automatically if load was kept)", "err")
        elif kind == "ack":
            self._log("ok " + ",".join(payload))
        elif kind == "err":
            self._log("ERROR " + ",".join(payload), "err")
        elif kind == "param":
            k, v = payload
            self.params[k] = v
            self._log(f"  {k} = {v}")
        elif kind == "sent":
            pass
        elif kind == "log":
            self._log(payload, "err")

    def _redraw(self) -> None:
        try:
            if not self.pause_var.get() and self.data.n:
                self._draw()
        finally:
            self.root.after(300, self._redraw)

    def _draw(self) -> None:
        a = self.data.view()
        t = a[0]
        win = {"1 min": 60, "10 min": 600, "60 min": 3600}.get(self.win_var.get())
        i0 = int(np.searchsorted(t, t[-1] - win)) if win else 0
        t, f, p, sp = a[0, i0:], a[1, i0:], a[2, i0:], a[3, i0:]
        view = self.view_var.get()
        if view != self._view_mode:
            self._make_axes(view)
        if view == "vs time":
            x, (fy, py, spy) = envelope(t, [f, p, sp])
            self.ln_f.set_data(x, fy)
            self.ln_sp.set_data(x, spy)
            self.ln_p.set_data(x, py)
        else:
            step = max(1, len(p) // 6000)
            self.ln_f.set_data(p[::step], f[::step])
            self.ln_sp.set_data([], [])
        for ax in self.axes:
            ax.relim()
            ax.autoscale_view()
        self.canvas.draw_idle()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", action="store_true", help="connect to a simulated rig")
    ap.add_argument("--sim-drop", type=float, default=0.0, help="sim: simulate an unplug every N seconds")
    args = ap.parse_args()
    root = tk.Tk()
    App(root, args.sim, args.sim_drop)
    root.mainloop()


if __name__ == "__main__":
    main()
