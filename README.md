# Tensionator

Firmware and a PC dashboard for a benchtop rope test rig that characterises **Dyneema (UHMWPE)
rope terminations** (knots, splices, clamps) under tension-to-break, cyclic loading, and
long-duration creep.

A stepper drives a 22.5 mm capstan drum; the rope wraps 4–6 turns and runs to the termination
under test at a fixed anchor, which is mounted through a 200 kgf S-type load cell. The firmware
closes the loop on that load cell: it holds constant tension, ramps to break, or cycles between
force limits, with hard safety limits. The dashboard plots live force and line travel, sends
commands, and logs everything to CSV.

---

## 1. Boot the firmware

**You need:** an Arduino **UNO R4 Minima**, and the Arduino IDE 2.x or `arduino-cli` with the
*Arduino UNO R4 Boards* core (`arduino:renesas_uno`) installed. No extra libraries are needed.

### Option A: Arduino IDE
1. Open `dyneema_rig/dyneema_rig.ino`.
2. *Tools → Board →* **Arduino UNO R4 Minima**, and select its port.
3. Click **Upload**.

### Option B: helper script (recommended on Windows)
```
pip install -r requirements.txt
python tools/upload.py            # auto-detects the board
python tools/upload.py --port COM5
```
The script compiles the sketch, puts the board into its bootloader and uploads. If it asks you
to, **double-tap the RESET button** (the LED starts pulsing) and it finishes on its own.

> **Why the helper exists:** on Windows the R4's automatic "jump to bootloader" step often fails
> with `Lost device after RESET?` / `exit status 74`. Double-tapping RESET puts the board in the
> bootloader by hand, and the upload then works. The script handles the waiting for you.

Close the dashboard before uploading, because only one program can hold the serial port at a time.

### Check that it booted
Start the dashboard (next section) and connect. You should see live data lines. Under
**Setup → Get all**, `fw` shows the firmware version and `rst` shows the last reset cause.
With no load cell connected, a red **`HX_TIMEOUT`** fault is expected and correct.

---

## 2. Run the dashboard

```
pip install -r requirements.txt     # pyserial, numpy, matplotlib (tkinter ships with Python)
python dashboard/rig_dashboard.py
```
Pick the board's COM port and click **Connect**. To try everything without hardware:
`python dashboard/rig_dashboard.py --sim` (add `--sim-drop 20` to simulate a cable pull every 20 s).

---

## 3. First-time setup

Do these steps in order before any real test.

### 3.1 Motor check (no load cell needed)
With the **rope off the drum** (or slack with its end free):
1. **Setup → Bench mode (no load cell)…** → *Yes*. The mode shows `BENCH`. In this mode only
   moves are allowed, and there is **no force protection**.
2. **Direction:** Move `20` mm, Speed `2` mm/s → **Tighten**. The drum must turn the way that
   winds rope *onto* it. If it turns the other way, send `SET dirinv 1` in the console, then `SAVE`.
3. **Distance:** **Zero pos**, then Move `70.686` mm (one drum revolution at r = 11.25 mm) →
   **Tighten**. A tape mark on the drum should return to its start. If it doesn't, check that the
   DM542T is set to 1600 pulses/rev.
4. **Stop and enable:** start a long move and press **STOP**. Then try **Motor OFF…**: the drum
   should turn by hand. **Motor ON** makes it stiff again.
5. **Exit bench mode**. Bench mode is also cleared on every reboot.

### 3.2 Load cell
1. Wire the HX711: VCC → 5V, GND → GND, **DT → D5**, **SCK → D6**. The load cell goes to
   E+/E−/A+/A−. **Get all** → `hz` should read about 10, or about 80 if the HX711's RATE pin is
   tied high. **80 Hz is strongly recommended**: it tightens the force cutoff and break detection.
2. **Tare:** rope slack, nothing pulling on the cell → **Setup → Tare** → *Yes*. The reading goes to about 0 N.
3. **Calibrate:** hang a known mass from the cell (N = kg × 9.807, e.g. 10 kg = 98.07 N; use
   the heaviest you can safely apply) → enter it in **Known load (N)** → **Calibrate**. Force
   must read **positive** in tension; a negative internal scale is fine.
   Tare and calibration are stored on the board and are **never** redone automatically at boot.
   Re-tare with the rope slack before each test day.

### 3.3 Geometry and tuning
1. **Effective radius:** line travel per step depends on the rope **centreline** radius,
   11.25 mm + rope diameter / 2. Set it with `SET reff 12.25` (for a 2 mm rope), then `SAVE`.
   For accurate absolute travel, measure one known move and adjust `reff` to match.
2. **Hold gain:** run `HOLD 100` and watch the force trace. If it oscillates, lower `kp`; if it
   is sluggish, raise it (default `kp = 0.005 (mm/s)/N`). Then `SAVE`.

---

## 4. Running tests

| Test | Dashboard tab | Firmware command |
|---|---|---|
| Creep (constant tension) | **Hold**: setpoint, approach rate, duration | `HOLD <N> [N/s]` |
| Tensile to break, constant displacement rate | **Ramp** | `RAMP <mm/min>` (≤ 120) |
| Tensile to break, constant force rate | **Ramp** | `RAMPF <N/s>` |
| Cyclic between force limits | **Cycle** | `CYCLE <lo> <hi> <mm/min> <count> [dwell_s]` |
| Jog / unload | **Setup**: Tighten / Loosen | `MOVE <mm> [mm/s]` (+ = tighten) |

- Recording starts automatically when a test starts (*Auto-start recording*). Set a **Sample tag**
  first, e.g. `bowline_2mm_s1`.
- To **unload**, use **Loosen**. Do not use *Motor OFF*: the 50:1 planetary back-drives under
  load and dumps the tension uncontrolled.
- A hold's *duration* is timed by the PC. If the PC or the link dies, the rig keeps holding
  (the safe default) until someone sends `STOP`.

### What "line travel" means (important for creep)
`pos_mm` is **not measured**. It is the motor step count converted to rope length:
`steps × 2π·reff / (1600 × 50)` ≈ 0.88 µm per step. Drum rotation equals the total lengthening
of everything between the drum and the anchor: specimen, termination slip, rope creeping on the
drum wraps, and frame, cell and gearbox flex.

- In a **ramp**, it is *not* specimen elongation. Use the camera marks for that.
- In a **HOLD**, the elastic parts stay at a fixed deflection because the force is constant, so
  **Δ line travel after the setpoint is reached is total system creep** (specimen + termination +
  drum wraps). Camera marks on the free span isolate material creep, and the difference
  approximates termination and drum creep.

---

## 5. Safety behaviour

| Layer | Behaviour |
|---|---|
| Hard cutoff | Above **1200 N** (compile-time `HARD_FMAX_N`, not settable over serial) motion stops instantly and an `OVERFORCE` fault latches. |
| Soft limit `fmax` | Default 1100 N. Ramps end normally here (`END …,FLIMIT`); HOLD/CYCLE setpoints above it are rejected. |
| Loaded speed cap | Above 100 N, tightening speed ≤ `vload` (2 mm/s), so overshoot between samples stays bounded. |
| Load cell | No valid sample for 500 ms → `HX_TIMEOUT`. Railed ADC → `HX_SATURATED`. Force < −50 N during a test → `NEG_FORCE`. Reads from a disconnected HX711 (stuck data line, frozen value) are rejected, not treated as 0 N. |
| Travel | More than `travel` (150 mm) from the test start → `TRAVEL`. This catches drum slip, a broken rope, or a dead cell. |
| Watchdog | 500 ms hardware watchdog. |
| Fault state | The motor stays **energised and holding**. Only *loosening* moves are accepted until `CLR`. |
| Reset under load | The motor stays energised through a board reset. An active HOLD **resumes automatically** if the force is still within ±20 % of its setpoint; otherwise the board reports `NORESUME`. Ramps and cycles are not resumed. |

Firmware limits do not replace a **hardware E-stop**. Make sure something physical can stop the motor.

**Recommended hardware backstop:** 1200 N needs only about 0.29 N·m at the motor. Lowering the
DM542T current until the motor stalls at about 1400 N (measure it with the load cell) makes it
physically unable to reach the load cell's 1961 N or the gearbox rating, even if the firmware fails.

**R4 pin drive:** the RA4M1 is rated for 8 mA per pin (the firmware sets medium drive), while the
DM542T opto inputs draw roughly 7–16 mA. It usually works, but it is out of spec. For unattended
runs, add a 74HCT244/245 buffer (or one transistor per line) on PUL+/DIR+/ENA+.

---

## 6. Hardware and wiring

| Arduino pin | Signal | Notes |
|---|---|---|
| D2 | DM542T PUL+ | step pulses from a 20 kHz timer interrupt (max about 8.8 mm/s of line) |
| D3 | DM542T DIR+ | |
| D4 | DM542T ENA+ | HIGH = motor de-energised |
| D5 | HX711 DT | |
| D6 | HX711 SCK | |
| GND | PUL−, DIR−, ENA−, HX711 GND | |

Drive train: StepperOnline 23HS30-2904S-EGS50 (NEMA 23, 50:1 planetary), DM542T at 1600
pulses/rev on 36 V. That gives 0.884 µm of line per microstep at r = 11.25 mm.

---

## 7. Serial protocol (115200 8N1, USB)

The host must assert **DTR** (the dashboard and the Arduino serial monitor do). Without DTR the
board neither sends nor accepts anything; this avoids a deadlock in the R4 core's USB stack.

Every device line ends in `*HH`, where HH is the XOR of all characters before `*`, in hex. The
dashboard drops lines that fail the check. Host commands may carry the same suffix (the
dashboard always adds it). Commands are case-insensitive.

**Device → host**

| Line | Fields |
|---|---|
| `D` | `ms, seq, force_N, raw, pos_mm, vel_mm_s, mode, setpoint_N, fault`, one per load-cell sample (÷ `stream`), or a 10 Hz heartbeat with `force = nan` while the load cell is silent |
| `E` | `ms, NAME, …`: `BOOT version,reset_cause,cal_valid` · `BREAK mode,peak,peak_pos,force` · `END mode,reason,peak,peak_pos` · `FAULT code,mode,force,pos` · `CYCLE n,peak,valley,pos` · `TARED` · `CALIBRATED` · `CALFAIL` · `RESUME` · `NORESUME` · `NOCELL` |
| `K` | `CMD`: command accepted |
| `X` | `CMD, reason`: command rejected |
| `P` | `key, value`: reply to `GET` |

**Host → device**

| Command | Meaning |
|---|---|
| `HOLD <N> [N/s]` | Constant-tension hold. Re-issue to change the target. Runs until `STOP`. |
| `RAMP <mm/min>` / `RAMPF <N/s>` | Ramp to break or `fmax` |
| `CYCLE <lo> <hi> <mm/min> <count> [dwell_s]` | Cycle between force limits (count 0 = run indefinitely) |
| `MOVE <mm> [mm/s]` | Relative move, + = tighten (≤ 8 mm/s) |
| `STOP` / `ESTOP` | Stop with / without deceleration. The motor keeps holding. |
| `CLR` | Clear a latched fault |
| `ON` / `OFF` | Energise / de-energise the driver |
| `NOCELL 1` / `NOCELL 0` | Bench mode without a load cell (MOVE only) / back to normal |
| `TARE` · `CAL <N>` · `ZERO` | Zero force · span calibration · zero position |
| `SET <key> <value>` | `fmax kp ki vhold vload accel travel brkdrop brkmin reff dirinv stream` |
| `GET` · `SAVE` · `DEFAULTS` · `PING` | List parameters · store them on the board · restore defaults (keeps calibration) · liveness check |

**Break detection:** a break is declared when force drops below `(1 − brkdrop) × peak` (default:
a 50 % drop) after the peak exceeded `brkmin` (50 N). In HOLD the reference is
min(peak, setpoint). In CYCLE it applies within each loading stroke; a rupture at the bottom of
a cycle shows up as `STALL` or a travel fault. If knot slip events cause false triggers, tune `brkdrop`.

---

## 8. Data files

The dashboard writes to `dashboard/data/` (git-ignored):

- `<timestamp>_<tag>_<test>.csv`: one row per sample with host UTC time, device ms, sequence
  number, force, raw ADC counts, line travel, velocity, mode, setpoint and fault.
- `…_events.csv`: link drops and reconnects, every command sent, device events (breaks, cycles,
  faults, reboots) and the parameters in force at the start.

The serial thread writes the CSV directly, so a busy or frozen GUI cannot lose data. Files are
flushed every second and fsync'd every 10 s. Gaps show up as missing sequence numbers ("missed
samples" in the status bar).

---

## 9. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Upload fails with `exit status 74` | Double-tap RESET and retry, or use `tools/upload.py`. |
| Dashboard says "not connected" or the port keeps disappearing | Check **Get all** → `rst`, `boots`, `prevcrumb`. A repeating `WDT` reset means the firmware is hanging somewhere; `prevcrumb` records where the previous run stopped (see `CRUMB(...)` in the sketch). |
| Force frozen at 0 / `HX_TIMEOUT` | HX711 unpowered or DT/SCK swapped or loose. `hxbad` counts rejected reads. |
| Force reads negative in tension | Redo calibration with the load applied in tension. |
| Motor turns the wrong way | `SET dirinv 1`, then `SAVE`. |

---

## Repository layout

```
dyneema_rig/dyneema_rig.ino   firmware (UNO R4 Minima; core libraries only: WDT, EEPROM, FspTimer)
dyneema_rig/rig_types.h       enums (kept separate for the Arduino prototype generator)
dashboard/rig_dashboard.py    PC dashboard: live plots, commands, CSV logging, auto-reconnect
dashboard/rig_protocol.py     line protocol (checksums, parsing)
dashboard/rig_sim.py          simulated rig for testing the dashboard without hardware
tools/upload.py               compile + upload helper that handles the R4 bootloader
tests/test_dashboard.py       headless tests: protocol, plot decimation, reconnect + logging (simulator)
```

Run the tests with `python tests/test_dashboard.py`. They take about 10 s and need no hardware.
