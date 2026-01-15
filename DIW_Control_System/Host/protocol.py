import struct
import re
import os

def load_constants():
    """Parses C++ headers to define Python constants."""
    constants = {}
    
    # Define paths relative to this script
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 1. Load Protocol Constants
    paths = [
        os.path.join(current_dir, '../Shared/protocol_constants.h'),
        os.path.join(current_dir, '../Firmware/src/HardwareConfig.h') # <--- NEW
    ]
    
    # Regex parsers
    define_pattern = re.compile(r'#define\s+(\w+)\s+(0x[0-9A-Fa-f]+|\d+\.?\d*)')
    enum_pattern = re.compile(r'^\s*(\w+)\s*=\s*(0x[0-9A-Fa-f]+|\d+)', re.MULTILINE)

    for header_path in paths:
        if not os.path.exists(header_path):
            print(f"Warning: Could not find header at {header_path}")
            continue

        with open(header_path, 'r') as f:
            content = f.read()

        # Parse #defines
        for match in define_pattern.finditer(content):
            key, val_str = match.groups()
            try:
                # Try integer
                constants[key] = int(val_str, 0)
            except ValueError:
                try:
                    # Try float
                    constants[key] = float(val_str)
                except ValueError:
                    pass # Skip complex defines

        # Parse Enums
        for match in enum_pattern.finditer(content):
            key, val_str = match.groups()
            constants[key] = int(val_str, 0)

    return constants

# Load Constants
CONSTANTS = load_constants()
globals().update(CONSTANTS)

# ==========================================
# 2. STRUCT DEFINITIONS
# ==========================================
CTRL_STRUCT = struct.Struct('<BBffBB')
TELEM_STRUCT = struct.Struct('<IfffiIB3x')

def create_packet(opcode, val_a=0.0, val_b=0.0):
    return CTRL_STRUCT.pack(
        CONSTANTS.get('PACKET_SYNC_BYTE', 0xAA),
        opcode,
        float(val_a),
        float(val_b),
        0, 
        CONSTANTS.get('PACKET_FOOTER_BYTE', 0xFF)
    )