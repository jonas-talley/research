#ifndef PROTOCOL_CONSTANTS_H
#define PROTOCOL_CONSTANTS_H

#define PACKET_SYNC_BYTE    0xAA
#define PACKET_FOOTER_BYTE  0xFF

enum Opcode : uint8_t {
    CMD_EMERGENCY_STOP  = 0x00,
    CMD_SET_VELOCITY    = 0x01, 
    CMD_QUEUE_MOVE      = 0x10, 
    CMD_CLEAR_QUEUE     = 0x11,
    CMD_START_QUEUE     = 0x12, 
    CMD_MOVE_RELATIVE   = 0x20, 
    CMD_ZERO_POSITION   = 0x21, 
    CMD_PING            = 0x99
};

struct __attribute__((packed)) ControlPacket {
    uint8_t sync;       
    uint8_t opcode;     
    float   value_a;    
    float   value_b;    
    uint8_t crc;        
    uint8_t footer;     
};

// CRITICAL FIX: Changed buffer_fill to uint16_t (2 bytes)
// Reduced padding from 3 bytes to 2 bytes to keep struct aligned
struct __attribute__((packed)) TelemetryPacket {
    uint32_t timestamp_us;
    float    pressure_raw;
    float    current_velocity; 
    float    target_velocity;
    int32_t  total_steps;      
    uint32_t error_flags;
    uint16_t buffer_fill;      
    uint8_t  padding[2];       
};

#endif