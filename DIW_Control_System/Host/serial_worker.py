import serial
import time
import collections
import datetime
import csv
import os
import struct
from PySide6.QtCore import QObject, Signal, QRecursiveMutex, QMutex
from protocol import *
from config_manager import load_config

class SerialWorker(QObject):
    connected = Signal(str)
    disconnected = Signal()
    telemetry_received = Signal(dict)
    error_occurred = Signal(str)
    stream_progress = Signal(int, int)
    log_status_changed = Signal(str)
    finished = Signal()

    def __init__(self, port):
        super().__init__()
        self.port = port
        self.running = False
        self.ser = None
        
        # Buffer
        self.move_playlist = collections.deque()
        self.total_moves = 0
        self.streaming_active = False
        self.waiting_for_buffer_sync = False
        
        # Thread-Safety, some form of interrupt lock is important on the serial port
        self.mutex = QRecursiveMutex()
        
        # Logging
        self.logging_enabled = True
        self.log_file = None
        self.csv_writer = None
        
        # Config
        self.config = load_config()
        self.log_folder = self.config.get("defaults", {}).get("log_folder", "logs")
        
        # Hardware Info
        self.steps_per_um = CONSTANTS.get('STEPS_PER_UM', 1.0) 
        if self.steps_per_um == 0: self.steps_per_um = 1.0

    def run(self):
        """Main Loop - Runs in Worker Thread"""
        try:
            self.ser = serial.Serial(self.port, 115200, timeout=0.01)
            self.ser.reset_input_buffer()
            self.running = True
            
            default_name = f"session_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            self.change_log_file(default_name)
            
            self.connected.emit(self.port)

            while self.running:
                # --- 1. Serial Access (Locked) ---
                self.mutex.lock()
                data_found = False
                telem_data = None
                fill = 512
                
                try:
                    if self.ser and self.ser.is_open and self.ser.in_waiting >= TELEM_STRUCT.size:
                        data = self.ser.read(TELEM_STRUCT.size)
                        if len(data) == TELEM_STRUCT.size:
                            try:
                                t_us, p_raw, v_cur, v_tgt, pos_steps, err, fill = TELEM_STRUCT.unpack(data)
                                t_s = t_us / 1e6
                                pos_um = pos_steps / self.steps_per_um

                                telem_data = {
                                    'time': t_s,
                                    'pressure': p_raw,
                                    'velocity_cur': v_cur,
                                    'velocity_tgt': v_tgt,
                                    'position_steps': pos_steps,
                                    'position_um': pos_um, 
                                    'error': err,
                                    'buffer': fill
                                }
                                data_found = True
                            except struct.error:
                                pass
                except Exception as e:
                    self.error_occurred.emit(str(e))
                finally:
                    self.mutex.unlock() 

                # --- 2. Logic ---
                if data_found and telem_data:
                    self.telemetry_received.emit(telem_data)

                    # Log Data (Requires Lock for File I/O)
                    self.mutex.lock()
                    if self.logging_enabled and self.csv_writer:
                        try:
                            self.csv_writer.writerow([
                                f"{telem_data['time']:.4f}", 
                                f"{telem_data['pressure']:.2f}", 
                                f"{telem_data['velocity_cur']:.2f}", 
                                f"{telem_data['velocity_tgt']:.2f}", 
                                f"{telem_data['position_um']:.3f}",   
                                telem_data['position_steps'],         
                                telem_data['error']
                            ])
                        except Exception:
                            pass
                    
                    # Sync Check
                    if self.waiting_for_buffer_sync:
                        if fill > 10 or fill >= self.total_moves:
                            self.waiting_for_buffer_sync = False
                    
                    # Refill
                    if self.streaming_active and not self.waiting_for_buffer_sync and fill < 400:
                        self.process_stream_internal() # Use internal unlocked version
                        
                    self.mutex.unlock()

                time.sleep(0.001) 

        except Exception as e:
            self.error_occurred.emit(f"Fatal Serial Error: {str(e)}")
        finally:
            # --- SAFE SHUTDOWN (In Worker Thread) ---
            self.mutex.lock()
            if self.ser and self.ser.is_open:
                self.ser.close()
            if self.log_file:
                self.log_file.close()
            self.mutex.unlock()
            self.disconnected.emit()
            self.finished.emit() # Signal to GUI we are done

    def stop(self):
        """Thread-safe stop signal. Called from Main Thread."""
        self.running = False

    def change_log_file(self, filename):
        self.mutex.lock()
        try:
            if not os.path.exists(self.log_folder):
                os.makedirs(self.log_folder)
            
            if self.log_file:
                self.log_file.close()
            
            full_path = os.path.join(self.log_folder, filename)
            self.log_file = open(full_path, 'a', newline='')
            self.csv_writer = csv.writer(self.log_file)
            self.csv_writer.writerow(["timestamp_s", "pressure_kPa", "velocity_cur", "velocity_tgt", "pos_um", "pos_steps", "error"])
            self.log_status_changed.emit(f"Logging to: {filename}")
        except Exception as e:
            self.error_occurred.emit(f"Log Init Error: {e}")
        finally:
            self.mutex.unlock()

    def process_stream_internal(self):
        """Internal Helper: Assumes Mutex is already locked."""
        if self.move_playlist:
            burst_limit = 10 
            count = 0
            while self.move_playlist and count < burst_limit:
                vel, dur = self.move_playlist.popleft()
                self.send_packet_locked(CMD_QUEUE_MOVE, vel, dur)
                count += 1
            remaining = len(self.move_playlist)
            self.stream_progress.emit(self.total_moves - remaining, self.total_moves)
        else:
            self.streaming_active = False

    def send_command(self, opcode, val_a=0.0, val_b=0.0):
        self.mutex.lock()
        try:
            self.send_packet_locked(opcode, val_a, val_b)
        finally:
            self.mutex.unlock()

    def send_packet_locked(self, opcode, val_a, val_b):
        """Internal Helper: Assumes Mutex is already locked."""
        if self.ser and self.ser.is_open:
            packet = create_packet(opcode, val_a, val_b)
            self.ser.write(packet)

    def queue_stream(self, moves):
        self.mutex.lock()
        try:
            self.move_playlist = collections.deque(moves)
            self.total_moves = len(moves)
            
            preload_count = 0
            preload_limit = 450
            while self.move_playlist and preload_count < preload_limit:
                vel, dur = self.move_playlist.popleft()
                self.send_packet_locked(CMD_QUEUE_MOVE, vel, dur)
                preload_count += 1
            
            self.streaming_active = True
            if preload_count > 0:
                self.waiting_for_buffer_sync = True
            
            self.stream_progress.emit(preload_count, self.total_moves)
        finally:
            self.mutex.unlock()