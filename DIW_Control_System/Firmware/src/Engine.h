#ifndef ENGINE_H
#define ENGINE_H

#include <Arduino.h>
#include "HardwareConfig.h"
#include "protocol_constants.h"

class Engine {
public:
    static void setup();
    static void run(); // Call in loop() for watchdog/safety checks

    // Command Interface
    static void set_velocity(float vel_um_s);
    static void move_relative(int32_t steps, float vel_um_s);
    static bool enqueue_move(float vel_um_s, uint32_t duration_ms);
    static void start_queue(); // The "Fire" command
    static void clear_queue();
    static void emergency_stop();
    static void zero_position();

    // Telemetry Accessors
    static TelemetryPacket get_telemetry();

private:
    // Internal ISRs
    static void control_loop_isr();
    static void step_timer_isr();
};

#endif