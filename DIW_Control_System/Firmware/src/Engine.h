#ifndef ENGINE_H
#define ENGINE_H

#include <Arduino.h>
#include "HardwareConfig.h"
#include "protocol_constants.h"

class Engine {
public:
    static void setup();
    static void run();

    // Command Interface
    static void set_velocity(float vel_um_s); //Start moving at this velocity
    static void move_relative(int32_t steps, float vel_um_s); //Moves a specific number of steps, at given velocity
    static bool enqueue_move(float vel_um_s, uint32_t duration_ms); //Adds moves to the queue
    static void start_queue(); //Starts running moves from the queue
    static void clear_queue(); //Clears the queue
    static void emergency_stop(); //Velocity to 0, mode cleared
    static void zero_position(); //Sets current step position to 0

    // Telemetry Accessors
    static TelemetryPacket get_telemetry();

private:
    // Internal ISRs
    static void control_loop_isr();
    static void step_timer_isr();
};

#endif