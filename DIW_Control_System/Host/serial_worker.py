import serial
import time
import collections
import datetime
import csv
import os
from PySide6.QtCore import QObject, Signal, QMutex
from protocol import *

class SerialWorker(QObject):
    connected = Signal(str)
    disconnected = Signal()
    telemetry_received = Signal(dict)
    error_occurred = Signal(str)
    stream_progress = Signal(int, int)
    log_status_changed = Signal(str)

    def __init__(self, port):
        super().__init__()
        self.port = port
        self.running = False
        self.ser = None
        
        # Buffer
        self.move_playlist = collections.deque()
        self.total_moves = 0
        self.streaming_active = False
        self.mutex = QMutex()
        
        # Logging
        self.logging_enabled = True
        self.log_file = None
        self.csv_writer = None
        
        # Hardware Info (Safe default if parsing fails)
        self.steps_per_um = CONSTANTS.get('STEPS_PER_UM', 1.0) 
        if self.steps_per_um == 0: self.steps_per_um = 1.0

    def run(self):
        try:
            self.ser = serial.Serial(self.port, 115200, timeout=0.01)
            self.ser.reset_input_buffer()
            self.running = True
            
            # Start Default Log
            default_name = f"session_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            self.change_log_file(default_name)
            
            self.connected.emit(self.port)

            while self.running:
                if self.ser.in_waiting >= TELEM_STRUCT.size:
                    data = self.ser.read(TELEM_STRUCT.size)
                    if len(data) == TELEM_STRUCT.size:
                        try:
                            # Unpack
                            t_us, p_raw, v_cur, v_tgt, pos_steps, err, fill = TELEM_STRUCT.unpack(data)
                            t_s = t_us / 1e6
                            
                            # Calculate Real World Position (SSOT)
                            pos_um = pos_steps / self.steps_per_um

                            telem_data = {
                                'time': t_s,
                                'pressure': p_raw,
                                'velocity_cur': v_cur,
                                'velocity_tgt': v_tgt,
                                'position_steps': pos_steps,
                                'position_um': pos_um, # <--- Added
                                'error': err,
                                'buffer': fill
                            }
                            self.telemetry_received.emit(telem_data)

                            # --- LOGGING ---
                            self.mutex.lock()
                            if self.logging_enabled and self.csv_writer:
                                try:
                                    self.csv_writer.writerow([
                                        f"{t_s:.4f}", 
                                        f"{p_raw:.2f}", 
                                        f"{v_cur:.2f}", 
                                        f"{v_tgt:.2f}", 
                                        f"{pos_um:.3f}",   # <--- Logged as Microns
                                        pos_steps,         # <--- Raw Steps kept for debug
                                        err
                                    ])
                                except Exception:
                                    pass 
                            self.mutex.unlock()
                            # ---------------

                            if self.streaming_active and fill < 50:
                                self.process_stream()

                        except struct.error:
                            pass 

                time.sleep(0.001) 

        except Exception as e:
            self.error_occurred.emit(str(e))
        finally:
            self.disconnect_serial()

    def change_log_file(self, filename):
        self.mutex.lock()
        try:
            if not os.path.exists("logs"):
                os.makedirs("logs")
            
            if self.log_file:
                self.log_file.close()
            
            full_path = os.path.join("logs", filename)
            is_new = not os.path.exists(full_path)
            
            self.log_file = open(full_path, 'a', newline='')
            self.csv_writer = csv.writer(self.log_file)
            
            if is_new:
                # UPDATED HEADER
                self.csv_writer.writerow([
                    "timestamp_s", 
                    "pressure_kPa", 
                    "velocity_current_um_s", 
                    "velocity_target_um_s", 
                    "position_um",       # <--- The critical column
                    "position_steps", 
                    "error_flags"
                ])
            
            self.log_status_changed.emit(f"Logging to: {filename}")
            
        except Exception as e:
            self.error_occurred.emit(f"Log Error: {e}")
        finally:
            self.mutex.unlock()

    # ... (Rest of file: process_stream, send_command, queue_stream, disconnect_serial are unchanged)
    def process_stream(self):
        self.mutex.lock()
        if self.move_playlist:
            vel, dur = self.move_playlist.popleft()
            self.send_command_internal(CMD_QUEUE_MOVE, vel, dur)
            remaining = len(self.move_playlist)
            self.stream_progress.emit(self.total_moves - remaining, self.total_moves)
        else:
            self.streaming_active = False
        self.mutex.unlock()

    def send_command(self, opcode, val_a=0.0, val_b=0.0):
        self.send_command_internal(opcode, val_a, val_b)

    def send_command_internal(self, opcode, val_a, val_b):
        if self.ser and self.ser.is_open:
            packet = create_packet(opcode, val_a, val_b)
            self.ser.write(packet)

    def queue_stream(self, moves):
        self.mutex.lock()
        self.move_playlist = collections.deque(moves)
        self.total_moves = len(moves)
        self.streaming_active = True
        self.mutex.unlock()

    def disconnect_serial(self):
        self.running = False
        self.mutex.lock()
        if self.ser:
            self.ser.close()
        if self.log_file:
            self.log_file.close()
            self.log_file = None
        self.mutex.unlock()
        self.disconnected.emit()