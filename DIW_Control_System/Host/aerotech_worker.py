import sys
import os
import time
import clr 
from PySide6.QtCore import QObject, Signal, QMutex, QThread

# DLL Path (Unchanged)
DLL_PATH = r"C:\Program Files (x86)\Aerotech\A3200\Bin\Aerotech.A3200.dll"

class AerotechWorker(QObject):
    connected = Signal()
    disconnected = Signal()
    connection_error = Signal(str)
    error_occurred = Signal(str)
    pos_updated = Signal(float, float, float)

    def __init__(self):
        super().__init__()
        self.running = False
        self.Controller = None
        self.Commands = None
        self._is_initialized = False

    def initialize_dll(self):
        if self._is_initialized: return True
        try:
            if not os.path.exists(DLL_PATH):
                self.connection_error.emit(f"DLL not found")
                return False
            sys.path.append(os.path.dirname(DLL_PATH))
            clr.AddReference("Aerotech.A3200")
            from Aerotech.A3200 import Controller, Commands
            self.Controller = Controller
            self.Commands = Commands
            self._is_initialized = True
            return True
        except Exception as e:
            self.connection_error.emit(f"DLL Load Error: {e}")
            return False

    def run(self):
        if not self.initialize_dll(): return
        try:
            self.Controller.Connect()
            self.connected.emit()
            self.running = True

            while self.running:
                try:
                    # Poll Positions
                    x = self.Commands.Status.AxisStatus.PositionFeedback("X")
                    y = self.Commands.Status.AxisStatus.PositionFeedback("Y")
                    z = self.Commands.Status.AxisStatus.PositionFeedback("Z")
                    self.pos_updated.emit(x, y, z)
                except:
                    pass
                time.sleep(0.1)

        except Exception as e:
            self.connection_error.emit(f"Connection Error: {e}")
        finally:
            if self._is_initialized:
                try: self.Controller.Disconnect()
                except: pass
            self.disconnected.emit()

    # --- NEW COMMANDS ---
    def enable_axis(self, axis):
        if not self.running: return
        try:
            self.Commands.Action.Enable(axis)
        except Exception as e:
            self.error_occurred.emit(f"Enable Error ({axis}): {e}")

    def disable_axis(self, axis):
        if not self.running: return
        try:
            self.Commands.Action.Disable(axis)
        except Exception as e:
            self.error_occurred.emit(f"Disable Error ({axis}): {e}")

    def move_relative(self, axis, distance, speed):
        if not self.running: return
        try:
            self.Commands.Motion.Inc(axis, distance, speed)
        except Exception as e:
            self.error_occurred.emit(f"Move Error: {e}")

    def stop_all(self):
        if self.running:
            try: self.Commands.Motion.Abort()
            except: pass

    def stop_worker(self):
        self.running = False