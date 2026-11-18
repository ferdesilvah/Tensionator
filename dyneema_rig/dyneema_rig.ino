/*
 * dyneema_rig.ino — capstan rope test rig controller (Arduino UNO R4 Minima, RA4M1)
 *
 * Hardware (all five signals are on RA4M1 port 1; direct register access is used)
 *   D2  PUL+  (DM542T)  P105    D5  HX711 DT   P102
 *   D3  DIR+  (DM542T)  P104    D6  HX711 SCK  P106
 *   D4  ENA+  (HIGH = motor de-energised)  P103
 *   Signal minus terminals -> Arduino GND. DM542T 1600 pulse/rev, 50:1 gearbox,
 *   drum r = 11.25 mm (set the *rope centreline* radius with `SET reff`).
 *
 * Design notes
 *   - Steps come from a 20 kHz GPT timer interrupt running a 32-bit phase accumulator, so
 *     step timing never depends on loop(), serial traffic or HX711 reads.
 *     Max step rate 10 kHz (~8.8 mm/s of line). No AccelStepper / HX711 libraries.
 *   - The HX711 is polled (non-blocking): it is read only when DT is low (data ready).
 *     All force-dependent logic runs once per new sample.
 *   - Safety (every sample, every mode): hard force cutoff (HARD_FMAX_N, compile-time,
 *     not changeable over serial), load-cell timeout / saturation, negative-force
 *     sanity check, travel limit per test, reduced tightening speed under load,
 *     setpoint validation, hardware watchdog.
 *   - On a fault the motor stops instantly and STAYS ENERGISED (holding). The 50:1
 *     planetary will back-drive under load if de-energised, so `OFF` releases the
 *     rope uncontrollably — use `MOVE -x` to unload instead.
 *   - Tare/calibration live in EEPROM and are NEVER re-done at boot (a reset under
 *     load must not zero a loaded cell). An active HOLD is also stored, and resumed
 *     after a reset if the measured force shows the motor kept holding the load.
 *
 * Serial protocol: 115200 8N1, '\n' terminated ASCII. See PROTOCOL.md.
 *   Device lines end with "*HH" = XOR of all chars before '*', two hex digits.
 *   Host commands may optionally end with "*HH" too (verified if present).
 */

#include <Arduino.h>
#include <EEPROM.h>
#include <WDT.h>
#include <FspTimer.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "rig_types.h"

// ----------------------------------------------------------------- constants
#define FW_VERSION "1.1.0-r4"

static const float HARD_FMAX_N     = 1200.0f;  // absolute cutoff — compile-time only
static const float STEPS_PER_REV   = 1600.0f * 50.0f; // driver microsteps * gearbox
static const uint32_t ISR_HZ       = 20000UL;  // step ISR rate (max step rate = ISR_HZ/2)
static const float MAX_SPS         = 9990.0f;  // must stay < ISR_HZ/2
static const float PHASE_PER_SPS   = 4294967296.0f / (float)ISR_HZ;
static const uint16_t HX_TIMEOUT_MS = 500;     // no sample for this long -> fault
static const float LOADED_N        = 100.0f;   // above this, tightening speed <= vload
static const float NEG_FAULT_N     = -50.0f;   // force below this while active -> fault
static const uint8_t AVG_SAMPLES   = 20;       // samples averaged by TARE / CAL

// Pins. Arduino numbers for pinMode; port-1 bit masks for fast access.
#define PUL_PIN 2
#define DIR_PIN 3
#define ENA_PIN 4
#define HX_DT_PIN 5
#define HX_SCK_PIN 6
static const uint32_t PUL_M = 1u << 5;   // P105
static const uint32_t DIR_M = 1u << 4;   // P104
static const uint32_t DT_M  = 1u << 2;   // P102
static const uint32_t SCK_M = 1u << 6;   // P106
// PCNTR3: writing bits [15:0] sets outputs (POSR), bits [31:16] clears them (PORR).
// One atomic store, no read-modify-write, so the ISR and main loop never race.
static inline void p1Set(uint32_t m) { R_PORT1->PCNTR3 = m; }
static inline void p1Clr(uint32_t m) { R_PORT1->PCNTR3 = m << 16; }
static inline bool p1In(uint32_t m)  { return (R_PORT1->PCNTR2 & m) != 0; }  // PIDR = bits [15:0]

// ----------------------------------------------------------------- persistent config
struct Config {
  uint16_t magic;
  uint8_t  ver;
  uint8_t  calValid;    // 1 once TARE + CAL have both been done
  int32_t  offset;      // HX711 raw counts at zero load
  float    scale;       // raw counts per newton (sign handles cell orientation)
  float    reff;        // effective winding radius of rope centreline, mm
  uint8_t  dirInv;      // 1 = flip which motor direction tightens
  uint8_t  streamDiv;   // emit a data line every N load-cell samples
  float    fmax;        // soft force limit, N (<= HARD_FMAX_N); tests end here
  float    kp;          // hold gain, (mm/s) per N of error
  float    ki;          // hold integral gain, (mm/s) per (N*s)
  float    vhold;       // max |speed| in HOLD / RAMPF, mm/s
  float    vload;       // max tightening speed when F > LOADED_N, mm/s
  float    accel;       // mm/s^2
  float    maxTravel;   // max line travel from test start, mm
  float    brkDrop;     // break when F < peak*(1-brkDrop)
  float    brkMin;      // ...and peak above this, N
};
static const uint16_t CFG_MAGIC = 0xD7E1;
static const uint8_t  CFG_VER   = 2;
static const int CFG_ADDR = 0;

struct ResumeRec {      // active HOLD, persisted so a reset doesn't end a creep test
  uint16_t magic;
  uint8_t  active;
  float    target;      // N
  float    rate;        // N/s
};
static const uint16_t RES_MAGIC = 0x4E1D;
static const int RES_ADDR = 128;

Config cfg;

static void cfgDefaults() {
  cfg.magic = CFG_MAGIC; cfg.ver = CFG_VER; cfg.calValid = 0;
  cfg.offset = 0; cfg.scale = 2000.0f; cfg.reff = 11.25f; cfg.dirInv = 0;
  cfg.streamDiv = 1; cfg.fmax = 1100.0f;
  cfg.kp = 0.005f; cfg.ki = 0.0f; cfg.vhold = 2.0f; cfg.vload = 2.0f;
  cfg.accel = 20.0f; cfg.maxTravel = 150.0f; cfg.brkDrop = 0.5f; cfg.brkMin = 50.0f;
}

// Reset diagnostics state (see diagBoot below). In .noinit RAM: survives resets.
struct Diag {
  uint32_t magic, boots, crumb, upMs, prevCrumb, prevUpMs;
};
__attribute__((section(".noinit"))) static volatile Diag diag;
static const uint32_t DIAG_MAGIC = 0x0D1A6001;
static const char* rstName = "?";
// Breadcrumb + uptime are mirrored into VBATT backup registers (bytes 16..23), which,
// unlike RAM, are not touched by the R4 bootloader that runs on every reset.
static inline void vbtWrite32(int at, uint32_t v) {
  uint16_t old = R_SYSTEM->PRCR;
  R_SYSTEM->PRCR = (uint16_t)(0xA500 | old | 0x02);        // unlock PRC1
  R_SYSTEM->VBTBKR[at] = (uint8_t)v;         R_SYSTEM->VBTBKR[at + 1] = (uint8_t)(v >> 8);
  R_SYSTEM->VBTBKR[at + 2] = (uint8_t)(v >> 16); R_SYSTEM->VBTBKR[at + 3] = (uint8_t)(v >> 24);
  R_SYSTEM->PRCR = (uint16_t)(0xA500 | (old & 0xFF));
}
static inline uint32_t vbtRead32(int at) {
  return (uint32_t)R_SYSTEM->VBTBKR[at] | ((uint32_t)R_SYSTEM->VBTBKR[at + 1] << 8) |
         ((uint32_t)R_SYSTEM->VBTBKR[at + 2] << 16) | ((uint32_t)R_SYSTEM->VBTBKR[at + 3] << 24);
}
static inline void crumbSet(uint32_t n) { diag.crumb = n; vbtWrite32(16, n); }
#define CRUMB(n) crumbSet(n)

// ----------------------------------------------------------------- step generator (ISR)
volatile uint32_t isrInc = 0;      // phase increment per tick
volatile int8_t   isrDirReq = 1;   // +1 tighten, -1 loosen (logical)
volatile int32_t  isrPos = 0;      // logical steps, + = line wound in
static uint32_t isrPhase = 0;
static volatile int8_t isrDirCur = 1;
static bool     isrPulHigh = false;
static FspTimer stepTimer;

static inline void writeDirPin(int8_t logicalDir) {
  if ((logicalDir > 0) ^ (cfg.dirInv != 0)) p1Set(DIR_M); else p1Clr(DIR_M);
}

static void stepTick(timer_callback_args_t*) {
  if (isrPulHigh) {                // finish the pulse started last tick (50 us wide)
    p1Clr(PUL_M);
    isrPulHigh = false;
    return;                        // a carry can never occur on the tick after a carry
  }
  int8_t d = isrDirReq;
  if (d != isrDirCur) {            // change DIR, then wait >= 1 tick (50 us setup) to step
    writeDirPin(d);
    isrDirCur = d;
    return;
  }
  uint32_t p = isrPhase + isrInc;
  if (p < isrPhase) {              // accumulator overflow -> one step
    p1Set(PUL_M);
    isrPulHigh = true;
    isrPos += isrDirCur;
  }
  isrPhase = p;
}

// Driver inputs are opto LEDs (~7-16 mA). Use the RA4M1's medium drive strength
// (Arduino's pinMode uses low drive). Output starts LOW.
static void driverPinInit(uint8_t pin) {
  R_IOPORT_PinCfg(NULL, g_pin_cfg[pin].pin,
                  IOPORT_CFG_PORT_DIRECTION_OUTPUT | IOPORT_CFG_PORT_OUTPUT_LOW | IOPORT_CFG_DRIVE_MID);
}

static bool stepperInit() {
  driverPinInit(PUL_PIN);
  driverPinInit(DIR_PIN);
  writeDirPin(1);
  uint8_t type = GPT_TIMER;
  int8_t ch = FspTimer::get_available_timer(type);
  if (ch < 0) return false;
  if (!stepTimer.begin(TIMER_MODE_PERIODIC, type, ch, (float)ISR_HZ, 0.0f, stepTick)) return false;
  if (!stepTimer.setup_overflow_irq(8)) return false;     // above the default (12) priority
  if (!stepTimer.open() || !stepTimer.start()) return false;
  return true;
}

static int32_t readPos() { return isrPos; }               // 32-bit loads are atomic on Cortex-M4
static void writePos(int32_t p) { noInterrupts(); isrPos = p; interrupts(); }

// ----------------------------------------------------------------- motion state (main loop)
static float vCur = 0;      // commanded speed now, steps/s (signed)
static float vTgt = 0;      // target speed, steps/s (signed)
static uint32_t lastMotionUs = 0;

static float stepsPerMm() { return STEPS_PER_REV / (2.0f * (float)M_PI * cfg.reff); }

static void applySpeed(float sps) {
  if (sps > MAX_SPS) sps = MAX_SPS;
  if (sps < -MAX_SPS) sps = -MAX_SPS;
  uint32_t inc = (uint32_t)(fabs(sps) * PHASE_PER_SPS);
  int8_t d = (sps > 0) ? 1 : (sps < 0 ? -1 : isrDirReq);
  noInterrupts(); isrInc = inc; isrDirReq = d; interrupts();
}

static void hardStop() { vCur = vTgt = 0; applySpeed(0); }

// ----------------------------------------------------------------- HX711 (non-blocking poll)
static inline bool hxReady() { return !p1In(DT_M); }

static int32_t hxRead() {
  // 24 data bits + 1 pulse => channel A, gain 128. SCK high time kept ~1-2 us with
  // interrupts masked per bit (HX711 powers down if SCK stays high > 60 us).
  uint32_t v = 0;
  for (uint8_t i = 0; i < 25; i++) {
    noInterrupts();
    p1Set(SCK_M);
    delayMicroseconds(1);
    uint8_t b = p1In(DT_M) ? 1 : 0;
    p1Clr(SCK_M);
    interrupts();
    if (i < 24) v = (v << 1) | b;
    delayMicroseconds(1);
  }
  if (v & 0x800000UL) v |= 0xFF000000UL;   // sign-extend 24 -> 32
  return (int32_t)v;
}

// ----------------------------------------------------------------- modes / test state
// Mode / Fault enums live in rig_types.h (Arduino's prototype generator needs them early)
static const char* const MODE_NAMES[] = {"IDLE","MOVE","HOLD","RAMP","RAMPF","CYCLE","TARE","CAL"};
static const char* const FAULT_NAMES[] = {"NONE","OVERFORCE","HX_TIMEOUT","HX_SATURATED","NEG_FORCE","TRAVEL"};

static Mode mode = M_IDLE;
static Fault faultCode = F_NONE;     // latched until CLR
static bool motorEnabled = true;

static float force = 0;              // latest force, N
static int32_t rawLast = 0;
static bool haveSample = false;
static uint32_t lastSampleMs = 0;
static uint32_t sampleSeq = 0;
static uint16_t ratecount = 0;
static uint32_t rateT0 = 0;
static float sampleHz = 0;

static int32_t testStartPos = 0;     // travel limit reference
static float peakF = 0;              // break detection
static float peakPosMm = 0;

// MOVE
static int32_t moveTarget = 0;
static float moveSps = 0;
// HOLD / RAMPF (force control)
static float spTarget = 0, spCur = 0, spRate = 0, holdI = 0;
static uint32_t lastCtrlMs = 0;
// RAMP
static float rampSps = 0;
// CYCLE
static float cyLo = 0, cyHi = 0, cySps = 0, cyDwellS = 0;
static uint16_t cyN = 0, cyDone = 0;
static bool cyUp = true, cyDwelling = false;
static uint32_t cyDwellT0 = 0;
static float cyPeak = 0, cyValley = 0, cyUpPeak = 0;
static int32_t cyPhaseStartPos = 0, cyLastUpTravel = 0;
// TARE / CAL averaging
static int64_t avgSum = 0;
static uint8_t avgN = 0;
static float calKnownN = 0;

static uint8_t streamCnt = 0;
static uint32_t lastDataMs = 0;
// Bench mode: run the motor with no load cell fitted (MOVE only; never persisted, so a
// reboot always returns to normal load-cell supervision).
static bool noCell = false;
static void emitData(float f, int32_t raw);
static bool resumePending = false;   // decide on HOLD resume once samples arrive
static uint8_t resumeWait = 0;
static ResumeRec resumeRec;

// ----------------------------------------------------------------- output (checksummed lines)
static char obuf[112];
static uint32_t txDropped = 0;
static uint32_t hxBad = 0;            // rejected HX711 reads (stuck DT line, frozen value)
static uint8_t olen = 0;

static void oBegin(char type) { olen = 0; obuf[olen++] = type; }
static void oStr(const char* s) {
  if (olen < sizeof(obuf) - 5) obuf[olen++] = ',';
  while (*s && olen < sizeof(obuf) - 5) obuf[olen++] = *s++;
}
static void oStrP(const __FlashStringHelper* fs) {
  const char* p = (const char*)fs;
  if (olen < sizeof(obuf) - 5) obuf[olen++] = ',';
  char c;
  while ((c = pgm_read_byte(p++)) && olen < sizeof(obuf) - 5) obuf[olen++] = c;
}
static void oLong(long v) { char t[12]; ltoa(v, t, 10); oStr(t); }
static void oULong(unsigned long v) { char t[12]; ultoa(v, t, 10); oStr(t); }
static void oFloat(float v, uint8_t dec) {
  char t[16];
  if (isnan(v) || isinf(v)) { oStr("nan"); return; }
  dtostrf(v, 1, dec, t);
  oStr(t);
}
static void oEnd() {
  uint8_t cs = 0;
  for (uint8_t i = 0; i < olen; i++) cs ^= (uint8_t)obuf[i];
  static const char hex[] = "0123456789ABCDEF";
  obuf[olen++] = '*';
  obuf[olen++] = hex[cs >> 4];
  obuf[olen++] = hex[cs & 15];
  obuf[olen++] = '\n';
  // Never block on USB: if the host isn't draining the port (closed, frozen, unplugged)
  // drop the line. Gaps show up host-side as missing sequence numbers.
  uint32_t c = diag.crumb;
  CRUMB(c | 0x100);                    // 0x1xx: inside availableForWrite
  int room = Serial.availableForWrite();
  CRUMB(c | 0x200);                    // 0x2xx: inside Serial.write
  if (room >= (int)olen) Serial.write((const uint8_t*)obuf, olen);
  else txDropped++;
  CRUMB(c);
}

static void ev(const __FlashStringHelper* name) { oBegin('E'); oULong(millis()); oStrP(name); }

static void sendOk(const char* cmd) { oBegin('K'); oStr(cmd); oEnd(); }
static void sendErr(const char* cmd, const __FlashStringHelper* why) {
  oBegin('X'); oStr(cmd); oStrP(why); oEnd();
}

// ----------------------------------------------------------------- persistence helpers
static void saveCfg() { EEPROM.put(CFG_ADDR, cfg); }
static void setResume(bool active, float target, float rate) {
  ResumeRec r; r.magic = RES_MAGIC; r.active = active ? 1 : 0; r.target = target; r.rate = rate;
  ResumeRec old; EEPROM.get(RES_ADDR, old);
  if (memcmp(&old, &r, sizeof r) != 0) EEPROM.put(RES_ADDR, r);   // avoid needless wear
}

// ----------------------------------------------------------------- mode transitions
static void enterIdle(const __FlashStringHelper* reason) {
  vTgt = 0;                                // decelerate to stop
  if (mode == M_HOLD || mode == M_RAMPF) setResume(false, 0, 0);
  Mode prev = mode;
  mode = M_IDLE;
  if (reason) {
    ev(F("END")); oStr(MODE_NAMES[prev]); oStrP(reason);
    oFloat(peakF, 2); oFloat(peakPosMm, 4); oEnd();
  }
}

static void fault(Fault f) {
  hardStop();
  if (mode == M_HOLD || mode == M_RAMPF) setResume(false, 0, 0);
  Mode prev = mode;
  mode = M_IDLE;
  faultCode = f;
  ev(F("FAULT")); oStr(FAULT_NAMES[f]); oStr(MODE_NAMES[prev]);
  oFloat(force, 2); oFloat(readPos() / stepsPerMm(), 4); oEnd();
}

static void beginTest() {
  testStartPos = readPos();
  peakF = force; peakPosMm = testStartPos / stepsPerMm();
}

// ----------------------------------------------------------------- control, per load-cell sample
static void onSample(int32_t raw) {
  uint32_t now = millis();
  rawLast = raw;
  lastSampleMs = now;
  haveSample = true;
  sampleSeq++;
  ratecount++;
  if (now - rateT0 >= 2000) { sampleHz = ratecount * 1000.0f / (now - rateT0); ratecount = 0; rateT0 = now; }

  force = (float)(raw - cfg.offset) / cfg.scale;
  int32_t pos = readPos();
  float spm = stepsPerMm();

  // ---- safety checks (all modes) -------------------------------------------------
  if (raw >= 0x7FFFFF || raw <= -0x800000L) { if (faultCode != F_HX_SAT) fault(F_HX_SAT); }
  else if (force > HARD_FMAX_N && (faultCode == F_NONE || vCur > 0)) fault(F_OVERFORCE);
  else if (mode >= M_HOLD && mode <= M_CYCLE) {
    if (force < NEG_FAULT_N) fault(F_NEGFORCE);
    else if (labs(pos - testStartPos) > (int32_t)(cfg.maxTravel * spm)) fault(F_TRAVEL);
  }

  // ---- peak tracking & break detection for force tests --------------------------
  if (mode == M_RAMP || mode == M_RAMPF || mode == M_HOLD) {
    if (force > peakF) { peakF = force; peakPosMm = pos / spm; }
    // Force-controlled modes compare against min(peak, setpoint) so that deliberately
    // lowering a HOLD setpoint is not mistaken for a rupture.
    float ref = (mode == M_RAMP) ? peakF : min(peakF, spCur);
    if (ref > cfg.brkMin && force < ref * (1.0f - cfg.brkDrop)) {
      hardStop();
      ev(F("BREAK")); oStr(MODE_NAMES[mode]); oFloat(peakF, 2); oFloat(peakPosMm, 4);
      oFloat(force, 2); oEnd();
      enterIdle(F("BREAK"));
    }
  }

  float dt = (now - lastCtrlMs) * 0.001f;
  if (dt > 0.5f) dt = 0.5f;
  lastCtrlMs = now;

  switch (mode) {
    case M_HOLD:
    case M_RAMPF: {
      // setpoint slews toward target at spRate (RAMPF: target is fmax)
      float step = spRate * dt;
      if (spCur < spTarget) spCur = min(spCur + step, spTarget);
      else if (spCur > spTarget) spCur = max(spCur - step, spTarget);
      float e = spCur - force;
      float vmax = cfg.vhold;
      float v = cfg.kp * e + holdI;
      bool sat = (v > vmax) || (v < -vmax);
      if (!sat || (v > 0 && e < 0) || (v < 0 && e > 0)) holdI += cfg.ki * e * dt;   // anti-windup
      holdI = constrain(holdI, -vmax, vmax);
      v = constrain(cfg.kp * e + holdI, -vmax, vmax);
      vTgt = v * spm;
      if (mode == M_RAMPF && force >= cfg.fmax) { hardStop(); enterIdle(F("FLIMIT")); }
      break;
    }
    case M_RAMP:
      vTgt = rampSps;
      if (force >= cfg.fmax) { hardStop(); enterIdle(F("FLIMIT")); }
      break;
    case M_CYCLE: {
      if (cyDwelling) {
        vTgt = 0;
        if (now - cyDwellT0 >= (uint32_t)(cyDwellS * 1000.0f)) cyDwelling = false;
        else break;
      }
      if (cyUp) {
        if (force > cyUpPeak) cyUpPeak = force;
        if (force > cyPeak) cyPeak = force;
        // rupture while loading
        if (cyUpPeak > cfg.brkMin && force < cyUpPeak * (1.0f - cfg.brkDrop)) {
          hardStop();
          ev(F("BREAK")); oStr("CYCLE"); oFloat(cyUpPeak, 2); oFloat(pos / spm, 4);
          oFloat(force, 2); oEnd();
          enterIdle(F("BREAK"));
          break;
        }
        // rope gone slack/broken at the bottom: this up-stroke has travelled far more than the last
        int32_t trav = pos - cyPhaseStartPos;
        if (cyLastUpTravel > 0 && trav > 3 * cyLastUpTravel + (int32_t)(2.0f * spm)) {
          hardStop(); enterIdle(F("STALL")); break;
        }
        if (force >= cyHi) {
          cyLastUpTravel = trav;
          cyUp = false; cyValley = force; cyPhaseStartPos = pos;
          if (cyDwellS > 0) { cyDwelling = true; cyDwellT0 = now; vTgt = 0; break; }
        }
        vTgt = cySps;
      } else {
        if (force < cyValley) cyValley = force;
        if (force <= cyLo) {
          cyDone++;
          ev(F("CYCLE")); oULong(cyDone); oFloat(cyPeak, 2); oFloat(cyValley, 2);
          oFloat(pos / spm, 4); oEnd();
          if (cyN > 0 && cyDone >= cyN) { hardStop(); enterIdle(F("DONE")); break; }
          cyUp = true; cyPeak = force; cyUpPeak = force; cyPhaseStartPos = pos;
          if (cyDwellS > 0) { cyDwelling = true; cyDwellT0 = now; vTgt = 0; break; }
          vTgt = cySps;
        } else {
          vTgt = -cySps;
        }
      }
      break;
    }
    case M_TARE:
    case M_CAL:
      avgSum += raw;
      if (++avgN >= AVG_SAMPLES) {
        int32_t avg = (int32_t)(avgSum / avgN);
        if (mode == M_TARE) {
          cfg.offset = avg;
          saveCfg();
          ev(F("TARED")); oLong(avg); oEnd();
        } else {
          float s = (float)(avg - cfg.offset) / calKnownN;
          if (fabs(s) < 1.0f) {
            ev(F("CALFAIL")); oStr("signal too small"); oEnd();
          } else {
            cfg.scale = s; cfg.calValid = 1; saveCfg();
            ev(F("CALIBRATED")); oFloat(s, 3); oEnd();
          }
        }
        mode = M_IDLE;
        force = (float)(raw - cfg.offset) / cfg.scale;
      }
      break;
    default: break;
  }

  // ---- HOLD resume after reset ---------------------------------------------------
  if (resumePending && ++resumeWait >= 5) {
    resumePending = false;
    float t = resumeRec.target;
    if (cfg.calValid && faultCode == F_NONE && t > cfg.brkMin && fabs(force - t) < 0.2f * t) {
      mode = M_HOLD; spTarget = t; spRate = resumeRec.rate; spCur = force; holdI = 0;
      beginTest(); peakF = max(force, t);
      ev(F("RESUME")); oFloat(t, 2); oFloat(force, 2); oEnd();
    } else {
      setResume(false, 0, 0);
      ev(F("NORESUME")); oFloat(t, 2); oFloat(force, 2); oEnd();
    }
  }

  // ---- data line --------------------------------------------------------------
  if (++streamCnt >= cfg.streamDiv) {
    streamCnt = 0;
    emitData(force, raw);
  }
}

// One data line. Also used as a 10 Hz heartbeat (force = nan) while no load-cell
// samples arrive, so the host still sees position, mode and faults.
static void emitData(float f, int32_t raw) {
  float spm = stepsPerMm();
  float sp = (mode == M_HOLD || mode == M_RAMPF) ? spCur
           : (mode == M_CYCLE ? (cyUp ? cyHi : cyLo) : 0.0f);
  lastDataMs = millis();
  oBegin('D');
  oULong(lastDataMs); oULong(sampleSeq); oFloat(f, 2); oLong(raw);
  oFloat(readPos() / spm, 4); oFloat(vCur / spm, 4);
  oStr(noCell ? "BENCH" : MODE_NAMES[mode]); oFloat(sp, 1); oStr(FAULT_NAMES[faultCode]);
  oEnd();
}

// ----------------------------------------------------------------- motion update (1 kHz)
static void motionUpdate() {
  uint32_t nowUs = micros();
  uint32_t dus = nowUs - lastMotionUs;
  if (dus < 1000) return;
  lastMotionUs = nowUs;
  float dt = dus * 1e-6f;
  if (dt > 0.01f) dt = 0.01f;
  float spm = stepsPerMm();

  if (mode == M_MOVE) {
    int32_t rem = moveTarget - readPos();
    if (rem == 0) { vTgt = 0; if (fabs(vCur) < 1.0f) { hardStop(); mode = M_IDLE; ev(F("END")); oStr("MOVE"); oStr("DONE"); oEnd(); } }
    else {
      float vStop = sqrtf(2.0f * cfg.accel * spm * fabs((float)rem));
      float v = min(moveSps, vStop);
      if (v < 20.0f) v = 20.0f;                  // creep in the last few steps
      vTgt = (rem > 0) ? v : -v;
    }
  }

  float t = vTgt;
  if (!motorEnabled) t = 0;
  if (faultCode != F_NONE && t > 0) t = 0;                       // latched fault: loosen only
  if (t > 0 && force > LOADED_N) t = min(t, cfg.vload * spm);    // gentle under load
  if (t > MAX_SPS) t = MAX_SPS;
  if (t < -MAX_SPS) t = -MAX_SPS;

  float dv = cfg.accel * spm * dt;
  if (vCur < t) vCur = min(vCur + dv, t);
  else if (vCur > t) vCur = max(vCur - dv, t);
  applySpeed(vCur);
}

// ----------------------------------------------------------------- commands
static char ibuf[64];
static uint8_t ilen = 0;
static bool iOverflow = false;

static uint8_t hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return 255;
}

static bool needCal(const char* cmd) {
  if (!cfg.calValid) { sendErr(cmd, F("not calibrated (TARE then CAL)")); return false; }
  return true;
}
static bool needReady(const char* cmd) {
  if (faultCode != F_NONE) { sendErr(cmd, F("fault latched, send CLR")); return false; }
  if (!motorEnabled) { sendErr(cmd, F("motor off, send ON")); return false; }
  if (noCell) { sendErr(cmd, F("bench mode (no load cell): only MOVE allowed, send NOCELL 0")); return false; }
  if (!haveSample) { sendErr(cmd, F("no load cell data")); return false; }
  if (mode == M_TARE || mode == M_CAL) { sendErr(cmd, F("busy")); return false; }
  return true;
}

static void printParams() {
  oBegin('P'); oStr("fw"); oStr(FW_VERSION); oEnd();
  oBegin('P'); oStr("hardfmax"); oFloat(HARD_FMAX_N, 1); oEnd();
  oBegin('P'); oStr("fmax"); oFloat(cfg.fmax, 1); oEnd();
  oBegin('P'); oStr("kp"); oFloat(cfg.kp, 5); oEnd();
  oBegin('P'); oStr("ki"); oFloat(cfg.ki, 5); oEnd();
  oBegin('P'); oStr("vhold"); oFloat(cfg.vhold, 3); oEnd();
  oBegin('P'); oStr("vload"); oFloat(cfg.vload, 3); oEnd();
  oBegin('P'); oStr("accel"); oFloat(cfg.accel, 2); oEnd();
  oBegin('P'); oStr("travel"); oFloat(cfg.maxTravel, 1); oEnd();
  oBegin('P'); oStr("brkdrop"); oFloat(cfg.brkDrop, 3); oEnd();
  oBegin('P'); oStr("brkmin"); oFloat(cfg.brkMin, 1); oEnd();
  oBegin('P'); oStr("reff"); oFloat(cfg.reff, 4); oEnd();
  oBegin('P'); oStr("dirinv"); oLong(cfg.dirInv); oEnd();
  oBegin('P'); oStr("stream"); oLong(cfg.streamDiv); oEnd();
  oBegin('P'); oStr("offset"); oLong(cfg.offset); oEnd();
  oBegin('P'); oStr("scale"); oFloat(cfg.scale, 3); oEnd();
  oBegin('P'); oStr("calvalid"); oLong(cfg.calValid); oEnd();
  oBegin('P'); oStr("stepspermm"); oFloat(stepsPerMm(), 3); oEnd();
  oBegin('P'); oStr("hz"); oFloat(sampleHz, 1); oEnd();
  oBegin('P'); oStr("txdrop"); oULong(txDropped); oEnd();
  oBegin('P'); oStr("hxbad"); oULong(hxBad); oEnd();
  oBegin('P'); oStr("rst"); oStr(rstName); oEnd();
  oBegin('P'); oStr("boots"); oULong(diag.boots); oEnd();
  oBegin('P'); oStr("prevcrumb"); oULong(diag.prevCrumb); oEnd();
  oBegin('P'); oStr("prevupms"); oULong(diag.prevUpMs); oEnd();
  oBegin('P'); oStr("mode"); oStr(MODE_NAMES[mode]); oEnd();
  oBegin('P'); oStr("fault"); oStr(FAULT_NAMES[faultCode]); oEnd();
  oBegin('P'); oStr("motor"); oStr(motorEnabled ? "ON" : "OFF"); oEnd();
  oBegin('P'); oStr("nocell"); oLong(noCell ? 1 : 0); oEnd();
}

static bool setParam(const char* key, float v) {
  bool idle = (mode == M_IDLE);
  if (!strcmp(key, "FMAX"))   { if (v < 10 || v > HARD_FMAX_N) return false; cfg.fmax = v; return true; }
  if (!strcmp(key, "KP"))     { if (v < 0 || v > 1) return false; cfg.kp = v; return true; }
  if (!strcmp(key, "KI"))     { if (v < 0 || v > 1) return false; cfg.ki = v; return true; }
  if (!strcmp(key, "VHOLD"))  { if (v <= 0 || v > 8) return false; cfg.vhold = v; return true; }
  if (!strcmp(key, "VLOAD"))  { if (v <= 0 || v > 8) return false; cfg.vload = v; return true; }
  if (!strcmp(key, "ACCEL"))  { if (v < 0.5f || v > 500) return false; cfg.accel = v; return true; }
  if (!strcmp(key, "TRAVEL")) { if (v < 1 || v > 2000) return false; cfg.maxTravel = v; return true; }
  if (!strcmp(key, "BRKDROP")){ if (v < 0.05f || v > 0.95f) return false; cfg.brkDrop = v; return true; }
  if (!strcmp(key, "BRKMIN")) { if (v < 1 || v > HARD_FMAX_N) return false; cfg.brkMin = v; return true; }
  if (!strcmp(key, "STREAM")) { if (v < 1 || v > 100) return false; cfg.streamDiv = (uint8_t)v; return true; }
  // geometry/direction only while stopped
  if (!strcmp(key, "REFF"))   { if (!idle || v < 5 || v > 50) return false; cfg.reff = v; return true; }
  if (!strcmp(key, "DIRINV")) {
    if (!idle || fabs(vCur) > 0) return false;
    noInterrupts();
    cfg.dirInv = v != 0;
    writeDirPin(isrDirCur);
    interrupts();
    return true;
  }
  return false;
}

static void handleCommand(char* line) {
  // optional "*HH" checksum
  char* star = strrchr(line, '*');
  if (star) {
    uint8_t cs = 0;
    for (char* p = line; p < star; p++) cs ^= (uint8_t)*p;
    uint8_t h = hexVal(star[1]), l = hexVal(star[2]);
    if (h == 255 || l == 255 || ((h << 4) | l) != cs) { sendErr("?", F("checksum")); return; }
    *star = 0;
  }
  // uppercase + tokenise on spaces/commas
  for (char* p = line; *p; p++) *p = toupper(*p);
  char* tok[7]; uint8_t n = 0;
  char* save;
  for (char* t = strtok_r(line, " ,\t\r", &save); t && n < 7; t = strtok_r(NULL, " ,\t\r", &save)) tok[n++] = t;
  if (n == 0) return;
  const char* c = tok[0];
  float a1 = n > 1 ? atof(tok[1]) : 0, a2 = n > 2 ? atof(tok[2]) : 0;
  float a3 = n > 3 ? atof(tok[3]) : 0, a4 = n > 4 ? atof(tok[4]) : 0, a5 = n > 5 ? atof(tok[5]) : 0;
  float spm = stepsPerMm();

  if (!strcmp(c, "PING")) { sendOk(c); return; }
  if (!strcmp(c, "STOP")) {          // stop any motion/test, keep holding position
    if (mode == M_TARE || mode == M_CAL) mode = M_IDLE;
    if (mode != M_IDLE) enterIdle(F("STOPPED"));
    vTgt = 0;
    sendOk(c); return;
  }
  if (!strcmp(c, "ESTOP")) {         // instant stop, no deceleration
    hardStop();
    if (mode != M_IDLE) enterIdle(F("ESTOP"));
    sendOk(c); return;
  }
  if (!strcmp(c, "OFF")) {           // de-energise: gearbox WILL back-drive under load
    hardStop();
    if (mode != M_IDLE) enterIdle(F("MOTOR_OFF"));
    digitalWrite(ENA_PIN, HIGH); motorEnabled = false;
    sendOk(c); return;
  }
  if (!strcmp(c, "ON")) { hardStop(); digitalWrite(ENA_PIN, LOW); motorEnabled = true; sendOk(c); return; }
  if (!strcmp(c, "CLR")) {
    faultCode = F_NONE;
    sendOk(c); return;
  }
  if (!strcmp(c, "NOCELL")) {        // NOCELL 1|0 — bench mode without a load cell
    if (mode != M_IDLE || fabs(vCur) > 0) { sendErr(c, F("stop first")); return; }
    noCell = (n > 1 && a1 != 0);
    if (noCell && faultCode == F_HX_TIMEOUT) faultCode = F_NONE;
    if (!noCell) lastSampleMs = millis();      // restart the load-cell timeout cleanly
    ev(F("NOCELL")); oLong(noCell ? 1 : 0); oEnd();
    sendOk(c); return;
  }
  if (!strcmp(c, "GET")) { printParams(); sendOk(c); return; }
  if (!strcmp(c, "SAVE")) {          // EEPROM write blocks the loop briefly: idle only
    if (mode != M_IDLE) { sendErr(c, F("stop first")); return; }
    saveCfg(); sendOk(c); return;
  }
  if (!strcmp(c, "DEFAULTS")) {      // keeps tare/calibration
    if (mode != M_IDLE) { sendErr(c, F("stop first")); return; }
    int32_t o = cfg.offset; float s = cfg.scale; uint8_t cv = cfg.calValid;
    cfgDefaults(); cfg.offset = o; cfg.scale = s; cfg.calValid = cv;
    sendOk(c); return;
  }
  if (!strcmp(c, "SET")) {
    if (n < 3 || !setParam(tok[1], a2)) { sendErr(c, F("bad key/value or not idle")); return; }
    sendOk(c); return;
  }
  if (!strcmp(c, "ZERO")) {
    if (mode != M_IDLE || fabs(vCur) > 0) { sendErr(c, F("stop first")); return; }
    writePos(0); sendOk(c); return;
  }
  if (!strcmp(c, "TARE")) {
    if (mode != M_IDLE || fabs(vCur) > 0) { sendErr(c, F("stop first")); return; }
    if (!haveSample) { sendErr(c, F("no load cell data")); return; }
    avgSum = 0; avgN = 0; mode = M_TARE; sendOk(c); return;
  }
  if (!strcmp(c, "CAL")) {
    if (mode != M_IDLE || fabs(vCur) > 0) { sendErr(c, F("stop first")); return; }
    if (!haveSample) { sendErr(c, F("no load cell data")); return; }
    if (n < 2 || a1 < 10) { sendErr(c, F("usage: CAL <known_N >= 10>")); return; }
    calKnownN = a1; avgSum = 0; avgN = 0; mode = M_CAL; sendOk(c); return;
  }
  if (!strcmp(c, "MOVE")) {          // MOVE <mm> [mm/s]  (+ = tighten)
    if (n < 2) { sendErr(c, F("usage: MOVE <mm> [mm/s]")); return; }
    if (!motorEnabled) { sendErr(c, F("motor off, send ON")); return; }
    if (!haveSample && !noCell) { sendErr(c, F("no load cell data (NOCELL 1 for bench mode)")); return; }
    if (mode == M_TARE || mode == M_CAL) { sendErr(c, F("busy")); return; }
    if (faultCode != F_NONE && a1 > 0) { sendErr(c, F("fault latched: only loosening (negative) moves")); return; }
    if (mode != M_IDLE && mode != M_MOVE) { sendErr(c, F("test running, STOP first")); return; }
    float v = n > 2 ? a2 : 2.0f;
    if (v <= 0 || v > 8.0f) { sendErr(c, F("speed 0..8 mm/s")); return; }
    if (fabs(a1) > cfg.maxTravel) { sendErr(c, F("exceeds travel limit")); return; }
    moveTarget = readPos() + (int32_t)lround(a1 * spm);
    moveSps = v * spm;
    mode = M_MOVE; sendOk(c); return;
  }
  if (!strcmp(c, "HOLD")) {          // HOLD <N> [N/s]
    if (!needReady(c) || !needCal(c)) return;
    if (mode != M_IDLE && mode != M_HOLD) { sendErr(c, F("test running, STOP first")); return; }
    if (n < 2 || a1 < 1 || a1 > cfg.fmax) { sendErr(c, F("setpoint out of range (1..fmax)")); return; }
    float r = n > 2 ? a2 : 20.0f;
    if (r <= 0 || r > 500) { sendErr(c, F("rate 0..500 N/s")); return; }
    bool fresh = (mode != M_HOLD);
    spTarget = a1; spRate = r;
    if (fresh) { spCur = force; holdI = 0; beginTest(); }
    peakF = force;                     // a new (possibly lower) target resets the break reference
    mode = M_HOLD;
    setResume(true, spTarget, spRate);
    sendOk(c); return;
  }
  if (!strcmp(c, "RAMP")) {          // RAMP <mm/min>  constant displacement rate to break / fmax
    if (!needReady(c) || !needCal(c)) return;
    if (mode != M_IDLE) { sendErr(c, F("test running, STOP first")); return; }
    if (n < 2 || a1 <= 0 || a1 > 120) { sendErr(c, F("rate 0..120 mm/min")); return; }
    rampSps = a1 / 60.0f * spm;
    beginTest(); mode = M_RAMP; sendOk(c); return;
  }
  if (!strcmp(c, "RAMPF")) {         // RAMPF <N/s>  constant force rate to break / fmax
    if (!needReady(c) || !needCal(c)) return;
    if (mode != M_IDLE) { sendErr(c, F("test running, STOP first")); return; }
    if (n < 2 || a1 <= 0 || a1 > 200) { sendErr(c, F("rate 0..200 N/s")); return; }
    spTarget = cfg.fmax + 1.0f; spRate = a1; spCur = force; holdI = 0;
    beginTest(); mode = M_RAMPF; sendOk(c); return;
  }
  if (!strcmp(c, "CYCLE")) {         // CYCLE <lo N> <hi N> <mm/min> <count, 0=inf> [dwell s]
    if (!needReady(c) || !needCal(c)) return;
    if (mode != M_IDLE) { sendErr(c, F("test running, STOP first")); return; }
    if (n < 5) { sendErr(c, F("usage: CYCLE lo hi mm/min count [dwell_s]")); return; }
    if (a1 < 0 || a2 <= a1 + 5 || a2 > cfg.fmax) { sendErr(c, F("need 0 <= lo < hi-5, hi <= fmax")); return; }
    if (a3 <= 0 || a3 > 120) { sendErr(c, F("rate 0..120 mm/min")); return; }
    if (a4 < 0 || a4 > 60000) { sendErr(c, F("count 0..60000")); return; }
    cyLo = a1; cyHi = a2; cySps = a3 / 60.0f * spm; cyN = (uint16_t)a4; cyDwellS = n > 5 ? max(a5, 0.0f) : 0;
    cyDone = 0; cyUp = true; cyDwelling = false; cyPeak = force; cyValley = force; cyUpPeak = force;
    cyLastUpTravel = 0;
    beginTest(); cyPhaseStartPos = testStartPos;
    mode = M_CYCLE; sendOk(c); return;
  }
  sendErr(c, F("unknown command"));
}

static void serialPoll() {
  uint8_t budget = 32;                       // bound time spent per loop
  while (Serial.available() && budget--) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (iOverflow) { sendErr("?", F("line too long")); }
      else if (ilen > 0) { ibuf[ilen] = 0; handleCommand(ibuf); }
      ilen = 0; iOverflow = false;
    } else if (ilen < sizeof(ibuf) - 1) {
      ibuf[ilen++] = ch;
    } else {
      iOverflow = true;
    }
  }
}

// ----------------------------------------------------------------- reset diagnostics
// Lives in .noinit RAM, which survives watchdog/software resets (not power-off). Each
// section of loop() leaves a "breadcrumb"; after a reset, GET reports where the previous
// run was and how long it had been up. A HardFault sets bit 15 and waits for the watchdog.

extern "C" void HardFault_Handler(void) {
  crumbSet(diag.crumb | 0x8000);
  for (;;) {}                                  // watchdog resets us
}

static void diagBoot() {
  if (vbtRead32(28) != DIAG_MAGIC) {      // first boot ever: initialise the record
    vbtWrite32(16, 0); vbtWrite32(20, 0); vbtWrite32(24, 0); vbtWrite32(28, DIAG_MAGIC);
  }
  diag.prevCrumb = vbtRead32(16);
  diag.prevUpMs = vbtRead32(20);
  diag.boots = vbtRead32(24) + 1;
  vbtWrite32(24, diag.boots);
  vbtWrite32(20, 0);
  CRUMB(100);
}

// ----------------------------------------------------------------- setup / loop
void setup() {
  diagBoot();
  // Reset cause: RSTSR0 bit0 = power-on, bits1-3 = low-voltage detect;
  // RSTSR1 bit0 = IWDT, bit1 = WDT, bit2 = software. No flag = reset pin / USB re-enumeration.
  uint16_t rst = (uint16_t)((R_SYSTEM->RSTSR1 << 8) | R_SYSTEM->RSTSR0);
  R_SYSTEM->RSTSR0 = 0; R_SYSTEM->RSTSR1 = 0;            // clear flags for the next boot
  rstName = (rst & 0x0200) ? "WDT" : (rst & 0x0001) ? "POWERON"
          : (rst & 0x000E) ? "BROWNOUT" : (rst & 0x0400) ? "SOFTWARE"
          : (rst & 0x0100) ? "IWDT" : "PIN";

  // Motor stays ENERGISED through boot: if we reset under load, holding is safer than
  // letting the gearbox back-drive and dump the load.
  driverPinInit(ENA_PIN);                                 // LOW = enabled
  pinMode(HX_SCK_PIN, OUTPUT); p1Clr(SCK_M);
  pinMode(HX_DT_PIN, INPUT);

  CRUMB(101);
  EEPROM.get(CFG_ADDR, cfg);
  if (cfg.magic != CFG_MAGIC || cfg.ver != CFG_VER) { CRUMB(102); cfgDefaults(); saveCfg(); }
  CRUMB(103);
  if (cfg.fmax > HARD_FMAX_N) cfg.fmax = HARD_FMAX_N;

  bool stepOk = stepperInit();
  Serial.begin(115200);                  // native USB: the baud rate is nominal
  // Do NOT call Serial.dtr() here: it makes the core ignore DTR, and then Serial.write()
  // deadlocks when no host has the port open (watchdog reset every ~1.6 s). With DTR
  // respected, output is discarded instantly while nobody is listening. Hosts must assert
  // DTR (the dashboard and Arduino serial monitor do). No waiting for a host either:
  // after a reset mid-test we must resume control immediately. BOOT may therefore be
  // missed by the host; `GET` reports rst/boots, and the dashboard notices dev_ms reset.
  CRUMB(104);

  CRUMB(105);
  EEPROM.get(RES_ADDR, resumeRec);
  resumePending = (resumeRec.magic == RES_MAGIC && resumeRec.active == 1);

  ev(F("BOOT")); oStr(FW_VERSION); oStr(rstName); oLong(cfg.calValid); oEnd();
  if (!stepOk) {                         // can't generate steps: refuse to run, keep reporting
    for (;;) { ev(F("FATAL")); oStr("step timer init failed"); oEnd(); delay(1000); }
  }
  lastSampleMs = millis(); rateT0 = millis(); lastCtrlMs = millis(); lastMotionUs = micros();
  CRUMB(106);
  WDT.begin(500);
  CRUMB(107);
}

void loop() {
  CRUMB(1);
  static uint32_t lastUpSave = 0;
  if (millis() - lastUpSave >= 50) { lastUpSave = millis(); vbtWrite32(20, lastUpSave); }
  WDT.refresh();
  CRUMB(2);
  serialPoll();
  CRUMB(3);
  // A real HX711 converts at <= 80 Hz and drives DOUT high right after the 25th clock.
  // A disconnected/unpowered module with DT stuck low would otherwise "read" 0 N forever
  // and never trip the timeout, so such reads are rejected (and counted) instead.
  static uint32_t lastTryUs = 0;
  static int32_t prevRaw = 0;
  static uint16_t sameCount = 0;
  if (hxReady() && micros() - lastTryUs >= 2000) {
    lastTryUs = micros();
    int32_t raw = hxRead();
    bool doutReleased = p1In(DT_M);
    sameCount = (raw == prevRaw) ? sameCount + 1 : 0;
    prevRaw = raw;
    CRUMB(4);
    if (doutReleased && sameCount < 50) onSample(raw);   // real noise never repeats 50x
    else hxBad++;
  }
  CRUMB(5);
  motionUpdate();
  CRUMB(6);
  if (!noCell && millis() - lastSampleMs > HX_TIMEOUT_MS && faultCode != F_HX_TIMEOUT) {
    fault(F_HX_TIMEOUT);
    lastSampleMs = millis();                 // re-report at most every HX_TIMEOUT_MS
  }
  CRUMB(7);
  if (millis() - lastDataMs >= 100 && millis() - lastSampleMs > 150) {
    emitData(NAN, 0);                        // heartbeat while the load cell is silent
  }
  CRUMB(8);
}
