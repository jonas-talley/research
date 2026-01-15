//
#ifndef PROTOCOL_CONSTANTS_H
#define PROTOCOL_CONSTANTS_H

#define PACKET_SYNC_BYTE    0xAA
#define PACKET_FOOTER_BYTE  0xFF

enum Opcode : uint8_t {
    CMD_EMERGENCY_STOP  = 0x00,
    CMD_SET_VELOCITY    = 0x01, // Continuous Jog
    CMD_QUEUE_MOVE      = 0x10, // Buffered Profile Move
    CMD_CLEAR_QUEUE     = 0x11,
    CMD_MOVE_RELATIVE   = 0x20, // <--- NEW: Move X steps
    CMD_ZERO_POSITION   = 0x21, // <--- NEW: Reset counter
    CMD_PING            = 0x99
};

// Control Packet (Unchanged size, reused fields)
struct __attribute__((packed)) ControlPacket {
    uint8_t sync;       
    uint8_t opcode;     
    float   value_a;    // Vel / Steps / Param
    float   value_b;    // Dur / Speed / Param
    uint8_t crc;        
    uint8_t footer;     
};

// Telemetry (Expanded for Position)
// New Size: 28 Bytes
struct __attribute__((packed)) TelemetryPacket {
    uint32_t timestamp_us;
    float    pressure_raw;
    float    current_velocity; 
    float    target_velocity;
    int32_t  total_steps;      // <--- NEW: Position Tracking
    uint32_t error_flags;
    uint8_t  buffer_fill;      
    uint8_t  padding[3];       // Alignment 
};

#endif