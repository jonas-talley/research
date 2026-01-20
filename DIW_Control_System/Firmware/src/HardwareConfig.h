#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H
#include <Arduino.h>

// --- Pin Definitions ---
#define PIN_STEP        33
#define PIN_DIR         19
#define PIN_ENABLE      48
#define PIN_PRESSURE    27

// --- Calibration ---

//Following values calibrated using ESI tool.
#define PRESSURE_CALIBRATION_FACTOR  1.568818661410291f
#define PRESSURE_OFFSET             -14.882802381988625f
#define MAX_PRESSURE_LIMIT           1550.0f //kPa

//Calibrated using LVDT
#define STEPS_PER_UM                 16.1024f

//Anything below this pulse width and the motor driver will lose the steps
#define MOTOR_PULSE_WIDTH_NS         3000 

// Watchdog Timeout (ms)
#define WDT_TIMEOUT_MS               100

#endif