import sys
import os
import datetime
import numpy as np
import pandas as pd
import pyqtgraph as pg
from collections import deque
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QPushButton, QLabel, QLineEdit, QTabWidget, QGroupBox, 
    QProgressBar, QFileDialog, QMessageBox, QSpinBox
)
from PySide6.QtCore import Qt, QThread, Slot
from serial_worker import SerialWorker
from protocol import *
import math
from aerotech import AerotechDriver

class DIWController(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("DIW Control System - Integrated Mode")
        self.resize(1100, 950) 

        # Config
        self.port_name = "COM3" 
        self.plot_history_size = 500
        
        # Data Storage
        self.data_history = {
            'time': deque(maxlen=self.plot_history_size),
            'pressure': deque(maxlen=self.plot_history_size),
            'velocity_cur': deque(maxlen=self.plot_history_size),
            'velocity_tgt': deque(maxlen=self.plot_history_size),
            'position_steps': deque(maxlen=self.plot_history_size)
        }
        
        # Execution Queues
        self.pending_moves = [] # For Teensy [(vel, ms), ...]
        self.pending_pgm = None # For Aerotech (List of strings)
        self.loaded_filename = ""

        self.setup_ui()
        self.start_connection()

    def setup_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout(main_widget)

        # 1. Header
        header_layout = QHBoxLayout()
        self.lbl_status = QLabel("Status: Initializing...")
        self.lbl_status.setStyleSheet("font-weight: bold; font-size: 14px; color: gray;")
        header_layout.addWidget(self.lbl_status)
        
        self.lbl_position = QLabel("Pos: 0 steps")
        self.lbl_position.setStyleSheet("font-size: 18px; font-weight: bold; color: magenta;")
        header_layout.addStretch()
        header_layout.addWidget(self.lbl_position)
        
        layout.addLayout(header_layout)

        # 2. Tabs
        tabs = QTabWidget()
        layout.addWidget(tabs)

        # Tab 1: Manual
        self.tab_manual = QWidget()
        self.setup_manual_tab(self.tab_manual)
        tabs.addTab(self.tab_manual, "Manual Control")

        # Tab 2: Smart Streamer (Replaces CSV & GCode tabs)
        self.tab_stream = QWidget()
        self.setup_stream_tab(self.tab_stream)
        tabs.addTab(self.tab_stream, "Trajectory Streamer")
        
        # Tab 3: Session Config
        self.tab_settings = QWidget()
        self.setup_settings_tab(self.tab_settings)
        tabs.addTab(self.tab_settings, "Session Config")

        # 3. Plots
        self.setup_plots(layout)

    def setup_manual_tab(self, parent):
        layout = QVBoxLayout(parent)
        
        # A. Velocity Control
        grp_vel = QGroupBox("1. Velocity Control (Jog)")
        vel_layout = QHBoxLayout()
        self.txt_velocity = QLineEdit("500")
        self.txt_velocity.setPlaceholderText("Velocity (um/s)")
        btn_set = QPushButton("Run Constant Velocity")
        btn_set.clicked.connect(self.cmd_set_velocity)
        vel_layout.addWidget(QLabel("Speed (um/s):"))
        vel_layout.addWidget(self.txt_velocity)
        vel_layout.addWidget(btn_set)
        grp_vel.setLayout(vel_layout)
        layout.addWidget(grp_vel)

        # B. Position Control
        grp_pos = QGroupBox("2. Step Control (Move Relative)")
        pos_layout = QHBoxLayout()
        
        self.txt_steps = QLineEdit("1000")
        self.txt_steps.setPlaceholderText("Steps")
        
        btn_move_p = QPushButton("Move (+)")
        btn_move_p.clicked.connect(lambda: self.cmd_move_relative(1))
        
        btn_move_n = QPushButton("Move (-)")
        btn_move_n.clicked.connect(lambda: self.cmd_move_relative(-1))
        
        btn_zero = QPushButton("Zero Pos")
        btn_zero.setStyleSheet("background-color: #DDD;")
        btn_zero.clicked.connect(lambda: self.worker.send_command(CMD_ZERO_POSITION))

        pos_layout.addWidget(QLabel("Distance (Steps):"))
        pos_layout.addWidget(self.txt_steps)
        pos_layout.addWidget(btn_move_p)
        pos_layout.addWidget(btn_move_n)
        pos_layout.addWidget(btn_zero)
        grp_pos.setLayout(pos_layout)
        layout.addWidget(grp_pos)

        # C. Stop
        btn_stop = QPushButton("STOP ALL MOTION")
        btn_stop.setMinimumHeight(50)
        btn_stop.setStyleSheet("background-color: orange; font-weight: bold;")
        btn_stop.clicked.connect(self.cmd_stop)
        layout.addWidget(btn_stop)

        layout.addStretch()

    def setup_stream_tab(self, parent):
        layout = QVBoxLayout(parent)
        
        # Instructions
        layout.addWidget(QLabel("Supports: .csv (Teensy Only) AND .gcode/.nc (Aerotech + Teensy)"))

        # File Loading
        file_layout = QHBoxLayout()
        self.lbl_filename = QLabel("No file loaded")
        self.lbl_filename.setStyleSheet("font-weight: bold;")
        
        btn_load = QPushButton("Load File...")
        btn_load.setMinimumHeight(40)
        btn_load.clicked.connect(self.load_file)
        
        file_layout.addWidget(btn_load)
        file_layout.addWidget(self.lbl_filename)
        layout.addLayout(file_layout)

        # Controls
        ctrl_layout = QHBoxLayout()
        btn_start = QPushButton("Start Stream")
        btn_start.setMinimumHeight(50)
        btn_start.setStyleSheet("background-color: green; color: white; font-size: 14px;")
        btn_start.clicked.connect(self.start_streaming)
        
        btn_abort = QPushButton("Abort / Reset")
        btn_abort.setMinimumHeight(50)
        btn_abort.setStyleSheet("background-color: #e74c3c; color: white; font-size: 14px;")
        btn_abort.clicked.connect(self.abort_stream)
        
        ctrl_layout.addWidget(btn_start)
        ctrl_layout.addWidget(btn_abort)
        layout.addLayout(ctrl_layout)

        # Progress
        self.prog_stream = QProgressBar()
        layout.addWidget(QLabel("Playlist Progress:"))
        layout.addWidget(self.prog_stream)
        
        self.prog_buffer = QProgressBar()
        self.prog_buffer.setRange(0, 64)
        self.prog_buffer.setFormat("Hardware Buffer: %v / 64")
        layout.addWidget(self.prog_buffer)
        layout.addStretch()

    def setup_settings_tab(self, parent):
        layout = QVBoxLayout(parent)
        
        # 1. Data Logging
        grp_log = QGroupBox("Data Logging")
        log_layout = QVBoxLayout()
        
        self.lbl_log_status = QLabel("Logging to: Default")
        self.lbl_log_status.setStyleSheet("color: blue;")
        log_layout.addWidget(self.lbl_log_status)

        file_input_layout = QHBoxLayout()
        self.txt_log_name = QLineEdit()
        default_ts = f"exp_{datetime.datetime.now().strftime('%H%M')}.csv"
        self.txt_log_name.setText(default_ts)

        btn_apply_log = QPushButton("Apply Log File")
        btn_apply_log.clicked.connect(self.cmd_update_log_name)

        file_input_layout.addWidget(QLabel("Filename:"))
        file_input_layout.addWidget(self.txt_log_name)
        file_input_layout.addWidget(btn_apply_log)
        
        log_layout.addLayout(file_input_layout)
        grp_log.setLayout(log_layout)
        layout.addWidget(grp_log)

        # 2. Plot Config
        grp_plot = QGroupBox("Plot Configuration")
        plot_layout = QHBoxLayout()
        
        self.spin_points = QSpinBox()
        self.spin_points.setRange(100, 10000)
        self.spin_points.setValue(self.plot_history_size)
        self.spin_points.setSingleStep(100)
        self.spin_points.valueChanged.connect(self.update_plot_settings)
        
        plot_layout.addWidget(QLabel("History Points:"))
        plot_layout.addWidget(self.spin_points)
        plot_layout.addStretch()
        grp_plot.setLayout(plot_layout)
        layout.addWidget(grp_plot)
        
        layout.addStretch()

    def setup_plots(self, parent_layout):
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.plot_widget.setBackground('k')
        
        # Plot 0: Position
        self.p0 = self.plot_widget.addPlot(title="Position (Steps)")
        self.p0.showGrid(x=True, y=True)
        self.curve_pos = self.p0.plot(pen=pg.mkPen('m', width=2))
        self.plot_widget.nextRow()

        # Plot 1: Velocity
        self.p1 = self.plot_widget.addPlot(title="Velocity (um/s)")
        self.p1.showGrid(x=True, y=True)
        self.p1.addLegend()
        self.curve_vel_tgt = self.p1.plot(pen=pg.mkPen('g', width=2, style=Qt.DashLine), name='Target')
        self.curve_vel_cur = self.p1.plot(pen=pg.mkPen('y', width=2), name='Current')
        self.p1.setXLink(self.p0)
        self.plot_widget.nextRow()

        # Plot 2: Pressure
        self.p2 = self.plot_widget.addPlot(title="Pressure (kPa)")
        self.p2.showGrid(x=True, y=True)
        self.curve_pressure = self.p2.plot(pen=pg.mkPen('c', width=2))
        self.p2.setXLink(self.p0)

        parent_layout.addWidget(self.plot_widget)

    # --- Logic ---
    def start_connection(self):
        self.thread = QThread()
        self.worker = SerialWorker(self.port_name)
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.run)
        
        self.worker.telemetry_received.connect(self.update_plots)
        self.worker.connected.connect(lambda p: self.lbl_status.setText(f"Connected: {p}"))
        self.worker.stream_progress.connect(self.prog_stream.setValue)
        self.worker.log_status_changed.connect(self.lbl_log_status.setText)
        
        self.thread.start()

    def load_file(self):
        """Smart loader that handles both CSV and GCode"""
        fname, _ = QFileDialog.getOpenFileName(self, "Open File", "", "Files (*.csv *.gcode *.nc *.txt)")
        if not fname: return

        self.loaded_filename = fname
        ext = os.path.splitext(fname)[1].lower()
        self.lbl_filename.setText(f"Loading {os.path.basename(fname)}...")
        QApplication.processEvents()

        try:
            # Mode 1: CSV (Teensy Only)
            if ext == '.csv':
                self.pending_pgm = None # Disable Aerotech Logic
                df = pd.read_csv(fname)
                times = df['Time'].values
                vels = df['Velocity'].values
                moves = []
                for i in range(len(times)-1):
                    dt = (times[i+1] - times[i]) * 1000.0
                    if dt > 0: moves.append((float(vels[i]), float(dt)))
                
                self.pending_moves = moves
                self.lbl_filename.setText(f"{os.path.basename(fname)} [CSV: {len(moves)} pts]")
            
            # Mode 2: G-Code (Dual Stream)
            elif ext in ['.gcode', '.nc', '.txt']:
                pgm_lines, teensy_moves = self.parse_gcode_to_dual_streams(fname)
                self.pending_pgm = pgm_lines
                self.pending_moves = teensy_moves
                self.lbl_filename.setText(f"{os.path.basename(fname)} [G-Code: {len(teensy_moves)} moves]")

            self.prog_stream.setMaximum(len(self.pending_moves))
            self.prog_stream.setValue(0)
            
        except Exception as e:
            self.lbl_filename.setText("Error loading file")
            QMessageBox.critical(self, "Load Error", str(e))

    def parse_gcode_to_dual_streams(self, gcode_filepath):
        aerotech_lines = [
            "ENABLE X",
            "ENABLE Y",
            "ENABLE Z",
            "INCREMENTAL",
            "METRIC", 
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
                

                
                # Convert to ms for Protocol
                duration_ms = duration_sec * 1000.0
                
                teensy_moves.append((de, duration_ms))

        aerotech_lines.append("END PROGRAM")
        return aerotech_lines, teensy_moves

    def start_streaming(self):
        if not self.pending_moves:
            return

        print("Starting execution sequence...")

        # 1. Handle Aerotech (If G-Code mode)
        if self.pending_pgm:
            pgm_filename = "diw_job.pgm"
            driver = AerotechDriver() 
            save_path = os.path.join(driver.user_files_path, pgm_filename)
            
            # Directory Safety Check
            folder_path = os.path.dirname(save_path)
            if not os.path.exists(folder_path):
                try:
                    os.makedirs(folder_path, exist_ok=True)
                except Exception as e:
                    QMessageBox.critical(self, "System Error", f"Cannot create Aerotech folder: {e}")
                    return

            # Write PGM
            try:
                with open(save_path, "w") as f:
                    f.write("\n".join(self.pending_pgm))
            except Exception as e:
                QMessageBox.critical(self, "Write Error", f"Failed to save PGM file: {e}")
                return

            # Trigger Stage Motion
            # We queue the Teensy FIRST so it's ready the instant the stage moves
            print("Uploading to Stage...")
            success, msg = driver.run_pgm(pgm_filename)
            if not success:
                QMessageBox.critical(self, "Stage Error", f"Aerotech refused start: {msg}")
                return
            print("Stage Started.")

        # 2. Start Teensy (Common to both modes)
        # This will start feeding the buffer immediately.
        # If manual CSV, it just runs. If G-Code, it runs in sync with the file we just triggered.
        self.worker.queue_stream(self.pending_moves)
        print("Teensy Stream Started.")

    def abort_stream(self):
        # 1. Stop Teensy
        self.worker.queue_stream([])
        self.worker.send_command(CMD_CLEAR_QUEUE)
        
        # 2. Stop Aerotech (Blind Abort)
        if self.pending_pgm:
            try:
                driver = AerotechDriver()
                driver.reset_task(1)
            except:
                pass

    def cmd_set_velocity(self):
        try:
            val = float(self.txt_velocity.text())
            self.worker.send_command(CMD_SET_VELOCITY, val_a=val)
        except ValueError:
            pass

    def cmd_move_relative(self, direction):
        try:
            steps = int(self.txt_steps.text()) * direction
            speed = float(self.txt_velocity.text())
            if speed <= 0: speed = 500.0
            self.worker.send_command(CMD_MOVE_RELATIVE, val_a=steps, val_b=speed)
        except ValueError:
            pass

    def cmd_stop(self):
        self.worker.send_command(CMD_SET_VELOCITY, val_a=0.0)
        self.abort_stream()

    def cmd_update_log_name(self):
        fname = self.txt_log_name.text().strip()
        if not fname: return
        if not fname.endswith(".csv"): fname += ".csv"
        self.worker.change_log_file(fname)

    def update_plot_settings(self):
        new_size = self.spin_points.value()
        self.plot_history_size = new_size
        for k in self.data_history:
            self.data_history[k] = deque(self.data_history[k], maxlen=new_size)

    @Slot(dict)
    def update_plots(self, data):
        self.lbl_position.setText(f"Pos: {data['position_steps']} steps")
        self.prog_buffer.setValue(data['buffer'])

        self.data_history['time'].append(data['time'])
        self.data_history['pressure'].append(data['pressure'])
        self.data_history['velocity_cur'].append(data['velocity_cur'])
        self.data_history['velocity_tgt'].append(data['velocity_tgt'])
        self.data_history['position_steps'].append(data['position_steps']) 
        
        t = np.array(self.data_history['time'])
        if len(t) > 1:
            try:
                self.curve_pos.setData(t, np.array(self.data_history['position_steps']))
                self.curve_vel_tgt.setData(t, np.array(self.data_history['velocity_tgt']))
                self.curve_vel_cur.setData(t, np.array(self.data_history['velocity_cur']))
                self.curve_pressure.setData(t, np.array(self.data_history['pressure']))
            except Exception:
                pass 

    def closeEvent(self, event):
        self.worker.disconnect_serial()
        self.thread.quit()
        self.thread.wait()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = DIWController()
    window.show()
    sys.exit(app.exec())