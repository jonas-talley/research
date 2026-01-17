import struct
import re
import os

def load_constants():
    """Parses C++ headers to define Python constants."""
    constants = {}
    
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    paths = [
        os.path.join(current_dir, '../Shared/protocol_constants.h'),
        os.path.join(current_dir, '../Firmware/src/HardwareConfig.h') 
    ]
    
    define_pattern = re.compile(r'#define\s+(\w+)\s+(0x[0-9A-Fa-f]+|\d+\.?\d*)')
    enum_pattern = re.compile(r'^\s*(\w+)\s*=\s*(0x[0-9A-Fa-f]+|\d+)', re.MULTILINE)

    for header_path in paths:
        if not os.path.exists(header_path):
            print(f"Warning: Could not find header at {header_path}")
            continue

        with open(header_path, 'r') as f:
            content = f.read()

        for match in define_pattern.finditer(content):
            key, val_str = match.groups()
            try:
                constants[key] = int(val_str, 0)
            except ValueError:
                try:
                    constants[key] = float(val_str)
                except ValueError:
                    pass 

        for match in enum_pattern.finditer(content):
            key, val_str = match.groups()
            constants[key] = int(val_str, 0)

    return constants

CONSTANTS = load_constants()
globals().update(CONSTANTS)

# ==========================================
# 2. STRUCT DEFINITIONS
# ==========================================
CTRL_STRUCT = struct.Struct('<BBffBB')

# CRITICAL FIX: 'B' (byte) -> 'H' (uint16), '3x' -> '2x'
TELEM_STRUCT = struct.Struct('<IfffiIH2x') 

def create_packet(opcode, val_a=0.0, val_b=0.0):
    return CTRL_STRUCT.pack(
        CONSTANTS.get('PACKET_SYNC_BYTE', 0xAA),
        opcode,
        float(val_a),
        float(val_b),
        0, 
        CONSTANTS.get('PACKET_FOOTER_BYTE', 0xFF)
    )