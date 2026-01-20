import json
import os

CONFIG_FILE = "config.json"

#DO NOT Change these defaults, particularly aerotech things unless you are getting pretty deep in the weeds.
DEFAULT_CONFIG = {
    "aerotech_ip": "127.0.0.1",
    "aerotech_port": 8000,
    "aerotech_path": r"C:\Users\Public\Public Documents\Aerotech\A3200\User Files",
    "teensy_port": "COM3",
    "defaults": {
        "velocity_um_s": 500.0,
        "move_steps": 1000,
        "log_folder": "logs"
    }
}

def load_config():
    # If config doesn't exist, create it with defaults
    if not os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, 'w') as f:
                json.dump(DEFAULT_CONFIG, f, indent=4)
        except Exception:
            pass # Use memory defaults if write fails
        return DEFAULT_CONFIG
    
    try:
        with open(CONFIG_FILE, 'r') as f:
            return json.load(f)
    except Exception as e:
        print(f"Error loading config: {e}. Using defaults.")
        return DEFAULT_CONFIG