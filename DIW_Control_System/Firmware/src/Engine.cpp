#include "Engine.h"
#include <IntervalTimer.h>
#include "Watchdog_t4.h" // Teensy 4.x Watchdog Library

// --- Globals (Encapsulated in cpp) ---
IntervalTimer controlTimer;
IntervalTimer stepTimer;
WDT_T4<WDT1> wdt;

// Queue
struct MoveCmd {
    float    velocity;
    uint32_t duration_ticks; 
};
#define QUEUE_SIZE 512 
volatile MoveCmd move_queue[QUEUE_SIZE];

volatile uint16_t queue_head = 0;
volatile uint16_t queue_tail = 0;
volatile uint16_t queue_count = 0;

// State Variables
volatile float    current_velocity_um_s = 0.0f;
volatile float    target_velocity_um_s = 0.0f;
volatile float    current_pressure_kpa = 0.0f;
volatile int32_t  global_position_steps = 0;
volatile int32_t  target_position_steps = 0;

// Modes
enum EngineMode { MODE_STOP, MODE_VELOCITY, MODE_QUEUE, MODE_POSITION };
volatile EngineMode current_mode = MODE_STOP;
volatile bool       is_queue_running = false; 
volatile uint32_t   current_move_ticks_remaining = 0;
volatile float      position_move_max_vel = 0.0f;

// Error & Telemetry
volatile uint32_t system_error_flags = 0;
volatile uint32_t last_step_period_us = 0;

// Constants
#define CONTROL_FREQ_HZ 4000
#define RAMP_ACCEL      2.0f 

// --- Helper: Atomic Float Write ---
void atomic_set_target(float val) {
    noInterrupts();
    target_velocity_um_s = val;
    interrupts();
}

// --- SETUP ---
void Engine::setup() {
    pinMode(PIN_STEP, OUTPUT);
    pinMode(PIN_DIR, OUTPUT);
    pinMode(PIN_ENABLE, OUTPUT);
    pinMode(PIN_PRESSURE, INPUT);
    
    // RESTORED: Onboard LED
    pinMode(LED_BUILTIN, OUTPUT);
    
    analogReadResolution(10);
    
    digitalWriteFast(PIN_ENABLE, HIGH); // Disable init

    // Watchdog Config
    WDT_timings_t config;
    config.trigger = 1; // ms
    config.timeout = WDT_TIMEOUT_MS; // ms
    wdt.begin(config);

    // Timers
    controlTimer.begin(control_loop_isr, 250); // 4kHz
}

// --- MAIN LOOP TASKS ---
void Engine::run() {
    wdt.feed(); 
    
    // Check Pressure (Redundant Safety in loop)
    if (current_pressure_kpa > MAX_PRESSURE_LIMIT) {
        emergency_stop();
    }

    // --- LED STATUS MONITOR ---
    // Uses non-blocking millis() to blink the LED based on state
    static uint32_t last_led_toggle = 0;
    static bool led_state = false;
    uint32_t now = millis();
    uint32_t interval = 2000; // Default: Idle (Slow Heartbeat)


    if (is_queue_running || current_mode == MODE_VELOCITY || current_mode == MODE_POSITION) {
        interval = 1000; // Active: Fast Blink
    } else {
        interval = 2000; // Idle: Slow Blink
    }

    if (now - last_led_toggle > interval) {
        led_state = !led_state;
        digitalWriteFast(LED_BUILTIN, led_state);
        last_led_toggle = now;
    }
}

// --- COMMANDS ---
void Engine::set_velocity(float vel_um_s) {
    current_mode = MODE_VELOCITY;
    is_queue_running = false;
    atomic_set_target(vel_um_s);
    digitalWriteFast(PIN_ENABLE, LOW);
}

void Engine::move_relative(int32_t steps, float vel_um_s) {
    noInterrupts();
    current_mode = MODE_POSITION;
    is_queue_running = false;
    target_position_steps = global_position_steps + steps;
    position_move_max_vel = abs(vel_um_s);
    if (position_move_max_vel < 1.0f) position_move_max_vel = 100.0f;
    digitalWriteFast(PIN_ENABLE, LOW);
    interrupts();
}

bool Engine::enqueue_move(float vel_um_s, uint32_t duration_ms) {
    noInterrupts();
    if (queue_count >= QUEUE_SIZE) {
        interrupts();
        system_error_flags |= 0x02; // Buffer Full
        return false;
    }
    
    uint32_t ticks = duration_ms * 4; 
    if (ticks == 0 && duration_ms > 0) ticks = 1;

    move_queue[queue_head].velocity = vel_um_s;
    move_queue[queue_head].duration_ticks = ticks;
    queue_head = (queue_head + 1) % QUEUE_SIZE;
    queue_count++;
    interrupts();
    return true;
}

void Engine::start_queue() {
    noInterrupts();
    if (queue_count > 0) {
        current_mode = MODE_QUEUE;
        is_queue_running = true;
        current_move_ticks_remaining = 0; 
        digitalWriteFast(PIN_ENABLE, LOW);
    }
    interrupts();
}

void Engine::clear_queue() {
    noInterrupts();
    queue_count = 0;
    queue_head = 0;
    queue_tail = 0;
    is_queue_running = false;
    target_velocity_um_s = 0.0f;
    interrupts();
}

void Engine::emergency_stop() {
    current_mode = MODE_STOP;
    is_queue_running = false;
    atomic_set_target(0.0f);
    digitalWriteFast(PIN_ENABLE, HIGH); 
    system_error_flags |= 0x01;
}

void Engine::zero_position() {
    noInterrupts();
    global_position_steps = 0;
    target_position_steps = 0;
    interrupts();
}

// --- TELEMETRY ---
TelemetryPacket Engine::get_telemetry() {
    TelemetryPacket t;
    noInterrupts();
    t.timestamp_us = micros();
    t.pressure_raw = current_pressure_kpa;
    t.current_velocity = current_velocity_um_s;
    t.target_velocity = target_velocity_um_s;
    t.total_steps = global_position_steps;
    t.error_flags = system_error_flags;
    
    // Explicit cast, though uint16 to uint16 is safe
    t.buffer_fill = queue_count; 
    
    interrupts();
    return t;
}

// --- ISRs ---
void Engine::step_timer_isr() {
    digitalWriteFast(PIN_STEP, HIGH);
    delayNanoseconds(MOTOR_PULSE_WIDTH_NS); 
    digitalWriteFast(PIN_STEP, LOW);

    if (current_velocity_um_s > 0) global_position_steps++;
    else global_position_steps--;
}

void Engine::control_loop_isr() {
    int raw = analogRead(PIN_PRESSURE);
    current_pressure_kpa = (raw * PRESSURE_CALIBRATION_FACTOR) + PRESSURE_OFFSET;

    if (current_pressure_kpa > MAX_PRESSURE_LIMIT) {
        current_mode = MODE_STOP;
        system_error_flags |= 0x01;
        digitalWriteFast(PIN_ENABLE, HIGH);
        target_velocity_um_s = 0.0f;
    }

    if (current_mode == MODE_STOP) {
        stepTimer.end();
        current_velocity_um_s = 0.0f;
        return;
    }

    if (current_mode == MODE_QUEUE && is_queue_running) {
        if (current_move_ticks_remaining > 0) {
            current_move_ticks_remaining--;
        } else {
            if (queue_count > 0) {
                target_velocity_um_s = move_queue[queue_tail].velocity;
                current_move_ticks_remaining = move_queue[queue_tail].duration_ticks;
                queue_tail = (queue_tail + 1) % QUEUE_SIZE;
                queue_count--;
            } else {
                target_velocity_um_s = 0.0f;
                is_queue_running = false; 
            }
        }
    } 
    else if (current_mode == MODE_POSITION) {
        int32_t error = target_position_steps - global_position_steps;
        if (error == 0) {
            target_velocity_um_s = 0.0f;
            if (abs(current_velocity_um_s) < 1.0f) current_mode = MODE_STOP; 
        } else {
            float dist_um = abs(error) / STEPS_PER_UM;
            float max_safe = sqrtf(2.0f * (RAMP_ACCEL * CONTROL_FREQ_HZ) * dist_um);
            float desired = position_move_max_vel;
            if (max_safe < desired) desired = max_safe;
            
            target_velocity_um_s = (error > 0) ? desired : -desired;
        }
    }

    if (current_velocity_um_s < target_velocity_um_s) {
        current_velocity_um_s += RAMP_ACCEL;
        if (current_velocity_um_s > target_velocity_um_s) current_velocity_um_s = target_velocity_um_s;
    } else if (current_velocity_um_s > target_velocity_um_s) {
        current_velocity_um_s -= RAMP_ACCEL;
        if (current_velocity_um_s < target_velocity_um_s) current_velocity_um_s = target_velocity_um_s;
    }

    if (abs(current_velocity_um_s) < 0.1f) {
        stepTimer.end();
        last_step_period_us = 0;
    } else {
        digitalWriteFast(PIN_DIR, (current_velocity_um_s > 0));
        float freq = abs(current_velocity_um_s) * STEPS_PER_UM;
        if (freq < 1.0f) freq = 1.0f; 

        uint32_t period_us = (uint32_t)(1000000.0f / freq);
        
        if (abs((int)period_us - (int)last_step_period_us) > 2) {
            stepTimer.begin(step_timer_isr, period_us);
            last_step_period_us = period_us;
        }
    }
}