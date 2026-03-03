import math
import os

def parse_csv_trajectory(fname):
    """
    Parses a CSV file for Teensy-only extrusion.
    Expected columns: 'Time', 'Velocity'
    Returns: list of (velocity, duration_ms) tuples
    """
    import pandas as pd
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
        
        if line.startswith(('G1', 'G2', 'G3')):
            cmd_type = line.split()[0]
            dx, dy, dz, de = 0.0, 0.0, 0.0, 0.0
            di, dj = 0.0, 0.0
            current_f = None
            
            parts = line.split()
            for p in parts:
                if p.startswith('X'): dx = float(p[1:])
                if p.startswith('Y'): dy = float(p[1:])
                if p.startswith('Z'): dz = float(p[1:])
                if p.startswith('E'): de = float(p[1:])
                if p.startswith('I'): di = float(p[1:])
                if p.startswith('J'): dj = float(p[1:])
                if p.startswith('F'): current_f = float(p[1:])
            
            if current_f is not None:
                last_f_rate = current_f
            
            distance = 0.0
            if cmd_type == 'G1':
                distance = math.sqrt(dx**2 + dy**2 + dz**2)
                aero_cmd = f"G1 X{dx:.4f} Y{dy:.4f} Z{dz:.4f} F{last_f_rate:.4f}"
            elif cmd_type in ['G2', 'G3']:
                # Circular interpolation in XY plane (G17 assumed)
                r = math.sqrt(di**2 + dj**2)
                if r <= 0.0001:
                    distance = math.sqrt(dx**2 + dy**2 + dz**2) # Fallback to linear
                else:
                    # Start angle from center to (0,0)
                    angle_start = math.atan2(-dj, -di)
                    # End angle from center to (dx, dy)
                    angle_end = math.atan2(dy - dj, dx - di)
                    
                    if cmd_type == 'G2': # Clockwise
                        diff = angle_start - angle_end
                    else: # G3 Counter-Clockwise
                        diff = angle_end - angle_start
                    
                    # Normalize angle
                    while diff <= 0: diff += 2 * math.pi
                    while diff > 2 * math.pi: diff -= 2 * math.pi
                    
                    # Full circle check
                    if abs(dx) < 0.0001 and abs(dy) < 0.0001:
                        diff = 2 * math.pi
                    
                    arc_dist = r * diff
                    distance = math.sqrt(arc_dist**2 + dz**2)
                
                aero_cmd = f"{cmd_type} X{dx:.4f} Y{dy:.4f} I{di:.4f} J{dj:.4f} F{last_f_rate:.4f}"
            
            if distance <= 0.0001: 
                # If it's just an extrusion move with no motion, it needs a small duration or it's a "set"
                if abs(de) > 0:
                    teensy_moves.append((de, 10.0)) # Tiny 10ms burst for static extrude?
                continue

            # Duration in Seconds
            duration_sec = distance / last_f_rate 
            
            # 1. Aerotech Command
            aerotech_lines.append(aero_cmd)
            
            # 2. Teensy Move (Convert to ms for Protocol)
            duration_ms = duration_sec * 1000.0
            teensy_moves.append((de, duration_ms))

    aerotech_lines.append("ABSOLUTE")
    aerotech_lines.append("END PROGRAM")
    return aerotech_lines, teensy_moves