//
#include <Arduino.h>
#include <IntervalTimer.h>
#include "HardwareConfig.h"
#include "protocol_constants.h" 

// ==========================================
// 1. RING BUFFER & GLOBALS
// ==========================================
struct MoveCmd {
    float    velocity;
    uint32_t duration_ticks; 
};

#define QUEUE_SIZE 64 
volatile MoveCmd move_queue[QUEUE_SIZE];
volatile uint8_t queue_head = 0;
volatile uint8_t queue_tail = 0;
volatile uint8_t queue_count = 0;

// State Tracking
volatile uint32_t current_move_ticks_remaining = 0;
volatile bool     is_executing_queue = false;

// Position Control Globals
volatile int32_t  global_position_steps = 0;
volatile int32_t  target_position_steps = 0;
volatile bool     is_position_mode = false;
volatile float    position_move_max_vel = 0.0f;

IntervalTimer controlLoopTimer;

volatile uint8_t  current_mode = CMD_EMERGENCY_STOP;
volatile float    target_velocity_um_s = 0.0f; 
volatile float    current_velocity_um_s = 0.0f; 
volatile float    current_pressure_kpa = 0.0f;
volatile uint32_t system_error_flags = 0;
volatile float    step_accumulator = 0.0f; 

// Loop Config
#define CONTROL_FREQ_HZ 4000
#define RAMP_ACCEL      2.0f // um/s per tick

// Debug
volatile uint32_t isr_counter = 0;
bool led_state = false;

// ==========================================
// 2. HARDWARE HELPERS
// ==========================================
void generate_step_pulse(bool dir) {
    digitalWriteFast(PIN_DIR, dir);
    // Setup hold time
    delayNanoseconds(500); 
    digitalWriteFast(PIN_STEP, HIGH);
    delayNanoseconds(MOTOR_PULSE_DELAY_NS); 
    digitalWriteFast(PIN_STEP, LOW);
    delayNanoseconds(MOTOR_PULSE_DELAY_NS);
    
    // Update Global Counter
    if (dir) global_position_steps++;
    else     global_position_steps--;
}

// ==========================================
// 3. THE NERVOUS SYSTEM (ISR)
// ==========================================
void control_isr() {
    isr_counter++;
    
    // Heartbeat (0.5s)
    if (isr_counter % 2000 == 0) {
        led_state = !led_state;
        digitalWriteFast(LED_BUILTIN, led_state);
    }

    // 1. Read Sensors
    int raw_adc = analogRead(PIN_PRESSURE);
    current_pressure_kpa = (raw_adc * PRESSURE_CALIBRATION_FACTOR) + PRESSURE_OFFSET;

    // 2. Safety
    if (current_pressure_kpa > MAX_PRESSURE_LIMIT) { 
        current_mode = CMD_EMERGENCY_STOP;
        system_error_flags |= 0x01; 
    }

    // 3. Mode Handling
    if (current_mode == CMD_EMERGENCY_STOP) {
        digitalWriteFast(PIN_ENABLE, HIGH); 
        target_velocity_um_s = 0.0f;
        current_velocity_um_s = 0.0f;
        is_executing_queue = false;
        is_position_mode = false;
        return;
    }

    digitalWriteFast(PIN_ENABLE, LOW); 

    // --- A. Queue Mode ---
    if (is_executing_queue) {
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
                system_error_flags |= 0x04; 
            }
        }
    }
    // --- B. Position Mode (Move Relative) ---
    else if (is_position_mode) {
        int32_t error = target_position_steps - global_position_steps;
        
        // Simple P-like Brake Logic
        // Calculate stopping distance: s = (v^2) / (2*a)
        // Convert velocity (um/s) to steps/s for this math
        float vel_steps = abs(current_velocity_um_s) / (1000.0f / STEPS_PER_UM); // Rough approx
        // Actually, simpler heuristic: 
        // If we are close, slow down. 
        // 1 tick = 2.0 um/s change. 
        
        float dist_um = abs(error) / STEPS_PER_UM;
        
        // Target Velocity Logic
        if (error == 0) {
            target_velocity_um_s = 0.0f;
            if (abs(current_velocity_um_s) < 1.0f) is_position_mode = false; // Done
        } else {
            // Deceleration Ramp Check
            // v^2 = 2 * a * d
            // max_safe_vel = sqrt(2 * acc * dist)
            // acc in um/s^2 = RAMP_ACCEL * CONTROL_FREQ_HZ = 2.0 * 4000 = 8000 um/s^2
            float max_safe_vel = sqrt(2.0f * (RAMP_ACCEL * CONTROL_FREQ_HZ) * dist_um);
            
            float desired_vel = position_move_max_vel;
            if (max_safe_vel < desired_vel) desired_vel = max_safe_vel;
            
            if (error > 0) target_velocity_um_s = desired_vel;
            else           target_velocity_um_s = -desired_vel;
        }
    }

    // 4. Ramp Generation
    if (current_velocity_um_s < target_velocity_um_s) {
        current_velocity_um_s += RAMP_ACCEL;
        if (current_velocity_um_s > target_velocity_um_s) current_velocity_um_s = target_velocity_um_s;
    } else if (current_velocity_um_s > target_velocity_um_s) {
        current_velocity_um_s -= RAMP_ACCEL;
        if (current_velocity_um_s < target_velocity_um_s) current_velocity_um_s = target_velocity_um_s;
    }

    // 5. Step Generation
    float steps_needed = (current_velocity_um_s / CONTROL_FREQ_HZ) * STEPS_PER_UM;
    
    bool dir = (steps_needed > 0);
    if (!dir) steps_needed = -steps_needed;
    
    step_accumulator += steps_needed;

    int max_steps = 10; 
    while (step_accumulator >= 1.0f && max_steps > 0) {
        generate_step_pulse(dir);
        step_accumulator -= 1.0f;
        max_steps--;
    }
}

// ==========================================
// 4. MAIN LOOP
// ==========================================
void setup() {
    Serial.begin(115200);
    pinMode(PIN_STEP, OUTPUT);
    pinMode(PIN_DIR, OUTPUT);
    pinMode(PIN_ENABLE, OUTPUT);
    pinMode(PIN_PRESSURE, INPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    analogReadResolution(10); 
    digitalWriteFast(PIN_ENABLE, HIGH); 
    controlLoopTimer.begin(control_isr, 250);
}

void loop() {
    if (Serial.available() >= sizeof(ControlPacket)) {
        ControlPacket packet;
        Serial.readBytes((char*)&packet, sizeof(packet));

        if (packet.sync == PACKET_SYNC_BYTE && packet.footer == PACKET_FOOTER_BYTE) {
            noInterrupts();
            
            if (packet.opcode == CMD_EMERGENCY_STOP) {
                current_mode = CMD_EMERGENCY_STOP;
                queue_count = 0;
            
            } else if (packet.opcode == CMD_SET_VELOCITY) {
                current_mode = CMD_SET_VELOCITY;
                target_velocity_um_s = packet.value_a;
                is_executing_queue = false;
                is_position_mode = false;
                queue_count = 0;

            } else if (packet.opcode == CMD_QUEUE_MOVE) {
                if (queue_count < QUEUE_SIZE) {
                    current_mode = CMD_QUEUE_MOVE; 
                    is_executing_queue = true; 
                    is_position_mode = false;    

                    uint32_t dur = (uint32_t)(packet.value_b * 4.0f); // 4 ticks/ms
                    if (dur == 0) dur = 1;

                    move_queue[queue_head].velocity = packet.value_a;
                    move_queue[queue_head].duration_ticks = dur;
                    queue_head = (queue_head + 1) % QUEUE_SIZE;
                    queue_count++;
                } else {
                    system_error_flags |= 0x02; 
                }

            } else if (packet.opcode == CMD_MOVE_RELATIVE) {
                // value_a = Steps, value_b = Velocity
                current_mode = CMD_MOVE_RELATIVE;
                is_executing_queue = false;
                is_position_mode = true;
                queue_count = 0;
                
                target_position_steps = global_position_steps + (int32_t)packet.value_a;
                position_move_max_vel = abs(packet.value_b);
                if (position_move_max_vel == 0) position_move_max_vel = 100.0f; // Safety

            } else if (packet.opcode == CMD_ZERO_POSITION) {
                global_position_steps = 0;
                target_position_steps = 0;
            } else if (packet.opcode == CMD_CLEAR_QUEUE) {
                queue_count = 0;
                target_velocity_um_s = 0.0f;
            }

            interrupts();
        } else {
            while(Serial.available()) Serial.read();
        }
    }

    static uint32_t last_telemetry_time = 0;
    if (millis() - last_telemetry_time > 20) {
        TelemetryPacket telem;
        telem.timestamp_us = micros();
        telem.pressure_raw = current_pressure_kpa;
        telem.current_velocity = current_velocity_um_s; 
        telem.target_velocity = target_velocity_um_s; 
        telem.error_flags = system_error_flags;
        
        noInterrupts();
        telem.buffer_fill = queue_count; 
        telem.total_steps = global_position_steps; // Send Position
        interrupts();
        
        Serial.write((uint8_t*)&telem, sizeof(telem));
        last_telemetry_time = millis();
    }
}