import pandas as pd
import math
import os

def parse_csv_trajectory(fname):
    """
    Parses a CSV file for Teensy-only extrusion.
    Expected columns: 'Time', 'Velocity'
    Returns: list of (velocity, duration_ms) tuples
    """
    df = pd.read_csv(fname)
    # Ensure columns exist (basic check)
    if 'Time' not in df.columns or 'Velocity' not in df.columns:
        raise ValueError("CSV must contain 'Time' and 'Velocity' columns")

    times = df['Time'].values
    vels = df['Velocity'].values
    moves = []
    
    for i in range(len(times)-1):
        dt = (times[i+1] - times[i]) * 1000.0 # Convert to ms
        if dt > 0: 
            moves.append((float(vels[i]), float(dt)))
            
    return moves

def parse_gcode_to_dual_streams(gcode_filepath):
    """
    Parses G-Code into:
    1. Aerotech program lines (List[str])
    2. Teensy extrusion moves (List[(velocity, duration_ms)])
    """
    aerotech_lines = [
        "ENABLE X", "ENABLE Y", "ENABLE Z",
        "INCREMENTAL", "METRIC", 
        "VELOCITY ON", 
        "WAIT MODE AUTO"
    ]
    
    teensy_moves = []
    last_f_rate = 1.0 
    
    with open(gcode_filepath, 'r') as f:
        lines = f.readlines()

    for line in lines:
        line = line.strip().upper()
        if not line or line.startswith(';') or line.startswith('('): 
            continue
        
        if line.startswith('G1'):
            dx, dy, dz, de = 0.0, 0.0, 0.0, 0.0
            current_f = None
            
            parts = line.split()
            for p in parts:
                if p.startswith('X'): dx = float(p[1:])
                if p.startswith('Y'): dy = float(p[1:])
                if p.startswith('Z'): dz = float(p[1:])
                if p.startswith('E'): de = float(p[1:])
                if p.startswith('F'): current_f = float(p[1:])
            
            if current_f is not None:
                last_f_rate = current_f
            
            # Geometry & Timing
            distance = math.sqrt(dx**2 + dy**2 + dz**2)
            if distance <= 0.0001: continue

            # Duration in Seconds
            duration_sec = distance / last_f_rate 
            
            # 1. Aerotech Command
            aero_cmd = f"G1 X{dx:.4f} Y{dy:.4f} Z{dz:.4f} F{last_f_rate:.4f}"
            aerotech_lines.append(aero_cmd)
            
            # 2. Teensy Move (Convert to ms for Protocol)
            duration_ms = duration_sec * 1000.0
            teensy_moves.append((de, duration_ms))

    aerotech_lines.append("END PROGRAM")
    return aerotech_lines, teensy_moves