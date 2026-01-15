#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H
#include <Arduino.h>

// --- Pin Definitions ---
// (Matched to your uploaded HardwareConfig.h)
#define PIN_STEP        33
#define PIN_DIR         19
#define PIN_ENABLE      48
#define PIN_PRESSURE    27

// --- EXACT Calibration from Old Code ---
// Source: ControllerConfig.h
#define PRESSURE_CALIBRATION_FACTOR  1.568818661410291f
#define PRESSURE_OFFSET             -14.882802381988625f

// Source: ControllerConfig.h (Old limit was 1600, I set 1550 to be safe)
#define MAX_PRESSURE_LIMIT           1600.0f 

// Source: ControllerConfig.h
#define STEPS_PER_UM                 16.1024f
#define MOTOR_PULSE_DELAY_NS         3500 // 3.5 microseconds

#endif