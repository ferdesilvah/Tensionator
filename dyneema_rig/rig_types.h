#pragma once
#include <stdint.h>
// Kept in a header so the Arduino IDE's auto-generated prototypes can see them.
enum Mode : uint8_t { M_IDLE, M_MOVE, M_HOLD, M_RAMP, M_RAMPF, M_CYCLE, M_TARE, M_CAL };
enum Fault : uint8_t { F_NONE, F_OVERFORCE, F_HX_TIMEOUT, F_HX_SAT, F_NEGFORCE, F_TRAVEL };
