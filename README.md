# DIW Control System

The **DIW_Control_System** codebase is the entire package used for control and communications for the printer.

Currently, the printer has a few key "modes":

* **Velocity Control:**
  A set velocity is set to the stepper motor in microns/second, goes until manual stop or a velocity change.

* **Step Control:**
  Manually jog a set number of steps at safe speed.

* **CSV Control:**
  A CSV with the columns `Time` and `Velocity` is imported. Maximum frequency ~500Hz for new velocities.

* **GCode Control:**
  A `.gcode` or `.nc` file with very custom instructions can be imported, which will coordinate both the Aerotech stage and the piston.
  * All X, Y, Z values are incremental.
  * All F Values are mm/s.
  * All E values are absolute microns/s to move the plunger.
  * Axes should be enabled, probably homed, and located at start of print.
  * Coordinated system might not catch Aerotech errors, treat like full open-loop motion.

### General Information

* If the teensy is active (see heartbeat LED), it will immediately stop piston motion **BEFORE** pressure goes critical.
* If the GUI process crashes while a uploaded profile or velocity is given, in the past the teensy has continued.
* The Aerotech coordination utilizes a relatively complex system of writing a file from the parsed gcode, and then telling the Aerotech controller to run it using the ascii interface. This is necessary because Aerotech is running 32-bit, but our python version is 64-bit, so the .dll files cannot talk for the .NET library implementation.
* Recommended environment is python3.14, currently the pc is using pixi as a python manager. Platformio is necessary for building and flashing the teensy.

### TODO

- [ ] Implement mode for relating bulk modulus and past data to reduce flowrate change transience
- [ ] Fix potential race conditions on buffer fill for CSV/Gcode velocity events
- [ ] Improve GUI consistency on colors and button placement
- [ ] Improve options for plotting
- [ ] Perform better stress-tests on system
- [ ] Implement teensy ping protocol to prevent continued motion if gui crashes

```text
DIW_Control_System/
├── Firmware/                  # Teensy 4.1 Motion Control Code
│   ├── src/
│   │   ├── Engine.cpp         # Core logic: Stepper ISRs, Buffer Management, Safety Watchdogs
│   │   ├── Engine.h           # Header interface for the Engine class
│   │   ├── HardwareConfig.h   # Pin definitions, Pressure limits, and Pulse width settings
│   │   └── main.cpp           # Entry point: USB Serial Parser and Command Dispatcher
│   └── platformio.ini         # PlatformIO build settings and dependencies
│
├── Host/                      # PC Control Software (Python/PySide6)
│   ├── aerotech.py            # Driver for Aerotech A3200 Gantry
│   ├── config.json            # User settings - Generated on run
│   ├── config_manager.py      # Utilities to load/save JSON config safely
│   ├── main_gui.py            # Main Application: UI Layout, Plotting, and Event Handling
│   ├── protocol.py            # Python wrapper for Shared constants. Critical for single definition of communication.
│   ├── serial_worker.py       # Threaded Serial Comms: Telemetry, Logging, and "Burst" Refill
│   └── trajectory_parser.py   # Logic to convert G-Code/CSV into dual motion streams
│
└── Shared/                    # Common Definitions
    └── protocol_constants.h   # "Source of Truth": OpCodes and Packet Structs used by C++ and Python
```