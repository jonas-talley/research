#include <Arduino.h>
#include "Engine.h"
#include "protocol_constants.h"

void setup() {
    Serial.begin(115200);
    Engine::setup();
}

void loop() {
    Engine::run(); // Watchdog feed & safety

    // --- 1. Serial Command Parsing ---
    while (Serial.available() >= sizeof(ControlPacket)) {
        ControlPacket packet;
        Serial.readBytes((char*)&packet, sizeof(packet));

        if (packet.sync == PACKET_SYNC_BYTE && packet.footer == PACKET_FOOTER_BYTE) {
            
            switch (packet.opcode) {
                case CMD_EMERGENCY_STOP:
                    Engine::emergency_stop();
                    break;
                
                case CMD_SET_VELOCITY:
                    Engine::set_velocity(packet.value_a);
                    break;

                case CMD_QUEUE_MOVE:
                    // Value A = Velocity, Value B = Duration (ms)
                    Engine::enqueue_move(packet.value_a, (uint32_t)packet.value_b);
                    break;

                case CMD_START_QUEUE:
                    Engine::start_queue(); // FIRE!
                    break;

                case CMD_CLEAR_QUEUE:
                    Engine::clear_queue();
                    break;

                case CMD_MOVE_RELATIVE:
                    // Value A = Steps, Value B = Velocity
                    Engine::move_relative((int32_t)packet.value_a, packet.value_b);
                    break;

                case CMD_ZERO_POSITION:
                    Engine::zero_position();
                    break;
            }
        } else {
            // Flush garbage
            while(Serial.available()) Serial.read();
        }
    }

    // --- 2. Telemetry (50Hz) ---
    static uint32_t last_telemetry = 0;
    if (millis() - last_telemetry > 20) {
        TelemetryPacket telem = Engine::get_telemetry();
        Serial.write((uint8_t*)&telem, sizeof(telem));
        last_telemetry = millis();
    }
}