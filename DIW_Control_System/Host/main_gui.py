import sys
import os
import datetime
import numpy as np
import pyqtgraph as pg
from collections import deque
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QPushButton, QLabel, QLineEdit, QTabWidget, QGroupBox, 
    QProgressBar, QFileDialog, QMessageBox, QSpinBox, QDoubleSpinBox,
    QGridLayout
)
from PySide6.QtCore import Qt, QThread, Slot
from serial_worker import SerialWorker
from protocol import *
from aerotech import AerotechDriver
from trajectory_parser import parse_gcode_to_dual_streams, parse_csv_trajectory
from config_manager import load_config

class DIWController(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("DIW Control System - Integrated Mode")
        self.resize(1100, 950) 

        # Load Config
        self.config = load_config()
        self.port_name = self.config.get("teensy_port", "COM3")
        
        # Config Defaults
        defaults = self.config.get("defaults", {})
        self.default_vel = defaults.get("velocity_um_s", 500.0)
        self.default_steps = defaults.get("move_steps", 1000)
        
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
        main_layout = QVBoxLayout(main_widget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)

        # --- 1. GLOBAL DASHBOARD (Persistent Header) ---
        header_frame = QGroupBox("System Overview")
        header_layout = QHBoxLayout(header_frame)
        
        # Status & Connection
        status_vbox = QVBoxLayout()
        self.lbl_status = QLabel("Status: Initializing...")
        self.lbl_status.setStyleSheet("font-weight: bold; font-size: 13px; color: #555;")
        self.lbl_log_status = QLabel("Logging: Inactive")
        self.lbl_log_status.setStyleSheet("color: #2980b9; font-size: 11px;")
        status_vbox.addWidget(self.lbl_status)
        status_vbox.addWidget(self.lbl_log_status)
        header_layout.addLayout(status_vbox)
        
        header_layout.addStretch()

        # Telemetry Displays
        self.lbl_position = QLabel("0 steps")
        self.lbl_position.setStyleSheet("font-size: 22px; font-weight: bold; color: #8e44ad;")
        pos_box = QVBoxLayout()
        pos_box.addWidget(QLabel("Current Position:"), alignment=Qt.AlignCenter)
        pos_box.addWidget(self.lbl_position, alignment=Qt.AlignCenter)
        header_layout.addLayout(pos_box)

        header_layout.addSpacing(40)

        self.lbl_pressure_val = QLabel("0.0 kPa")
        self.lbl_pressure_val.setStyleSheet("font-size: 22px; font-weight: bold; color: #16a085;")
        press_box = QVBoxLayout()
        press_box.addWidget(QLabel("Extruder Pressure:"), alignment=Qt.AlignCenter)
        press_box.addWidget(self.lbl_pressure_val, alignment=Qt.AlignCenter)
        header_layout.addLayout(press_box)

        header_layout.addStretch()

        # Global Stop & Reset
        btn_layout = QHBoxLayout()
        
        self.btn_clear_errors = QPushButton("RESET SYSTEM")
        self.btn_clear_errors.setFixedSize(100, 60)
        self.btn_clear_errors.setStyleSheet("background-color: #34495e; color: white; font-weight: bold; font-size: 11px; border-radius: 5px;")
        self.btn_clear_errors.clicked.connect(self.cmd_clear_errors)
        
        self.btn_global_stop = QPushButton("STOP")
        self.btn_global_stop.setFixedSize(100, 60)
        self.btn_global_stop.setStyleSheet("background-color: #e74c3c; color: white; font-weight: bold; font-size: 16px; border-radius: 5px;")
        self.btn_global_stop.clicked.connect(self.cmd_stop)
        
        btn_layout.addWidget(self.btn_clear_errors)
        btn_layout.addWidget(self.btn_global_stop)
        header_layout.addLayout(btn_layout)
        
        main_layout.addWidget(header_frame)

        # --- 2. CONTROL AREA (Tabs) ---
        self.tabs = QTabWidget()
        main_layout.addWidget(self.tabs)

        # Tab 1: Manual
        self.tab_manual = QWidget()
        self.setup_manual_tab(self.tab_manual)
        self.tabs.addTab(self.tab_manual, "Manual Control")

        # Tab 2: Smart Streamer
        self.tab_stream = QWidget()
        self.setup_stream_tab(self.tab_stream)
        self.tabs.addTab(self.tab_stream, "Trajectory Streamer")
        
        # Tab 3: Configuration
        self.tab_settings = QWidget()
        self.setup_settings_tab(self.tab_settings)
        self.tabs.addTab(self.tab_settings, "Settings & Logging")

        # --- 3. PLOTS AREA ---
        self.setup_plots(main_layout)

    def setup_manual_tab(self, parent):
        layout = QVBoxLayout(parent)
        
        # Layout for Group Boxes
        h_layout = QHBoxLayout()

        # A. Velocity Control
        grp_vel = QGroupBox("Velocity Control (Jog)")
        vel_layout = QVBoxLayout()
        
        self.spin_velocity = QDoubleSpinBox()
        self.spin_velocity.setRange(-5000.0, 5000.0)
        self.spin_velocity.setValue(self.default_vel)
        self.spin_velocity.setSuffix(" um/s")
        self.spin_velocity.setSingleStep(50.0)
        self.spin_velocity.setMinimumHeight(30)
        
        btn_set = QPushButton("Run Constant Velocity")
        btn_set.setMinimumHeight(40)
        btn_set.clicked.connect(self.cmd_set_velocity)
        
        vel_layout.addWidget(QLabel("Target Speed:"))
        vel_layout.addWidget(self.spin_velocity)
        vel_layout.addWidget(btn_set)
        vel_layout.addStretch()
        grp_vel.setLayout(vel_layout)
        h_layout.addWidget(grp_vel)

        # B. Position Control
        grp_pos = QGroupBox("Step Control (Relative Move)")
        pos_layout = QVBoxLayout()
        
        self.spin_steps = QSpinBox()
        self.spin_steps.setRange(1, 1000000)
        self.spin_steps.setValue(self.default_steps)
        self.spin_steps.setSuffix(" steps")
        self.spin_steps.setSingleStep(1000)
        self.spin_steps.setMinimumHeight(30)
        
        move_btns = QHBoxLayout()
        btn_move_p = QPushButton("Move (+)")
        btn_move_p.setMinimumHeight(40)
        btn_move_p.clicked.connect(lambda: self.cmd_move_relative(1))
        
        btn_move_n = QPushButton("Move (-)")
        btn_move_n.setMinimumHeight(40)
        btn_move_n.clicked.connect(lambda: self.cmd_move_relative(-1))
        move_btns.addWidget(btn_move_n)
        move_btns.addWidget(btn_move_p)

        btn_zero = QPushButton("Set Origin (Zero)")
        btn_zero.setMinimumHeight(30)
        btn_zero.clicked.connect(lambda: self.worker.send_command(CMD_ZERO_POSITION))

        pos_layout.addWidget(QLabel("Relative Distance:"))
        pos_layout.addWidget(self.spin_steps)
        pos_layout.addLayout(move_btns)
        pos_layout.addWidget(btn_zero)
        pos_layout.addStretch()
        grp_pos.setLayout(pos_layout)
        h_layout.addWidget(grp_pos)

        layout.addLayout(h_layout)

    def setup_stream_tab(self, parent):
        layout = QVBoxLayout(parent)
        
        # File Loading Section
        grp_file = QGroupBox("Job Selection")
        file_layout = QHBoxLayout()
        self.lbl_filename = QLabel("No job file loaded")
        self.lbl_filename.setStyleSheet("font-style: italic; color: #7f8c8d;")
        
        btn_load = QPushButton("Select File...")
        btn_load.setMinimumHeight(30)
        btn_load.clicked.connect(self.load_file)
        
        file_layout.addWidget(btn_load)
        file_layout.addWidget(self.lbl_filename)
        file_layout.addStretch()
        grp_file.setLayout(file_layout)
        layout.addWidget(grp_file)

        # Progress Section
        grp_prog = QGroupBox("Execution Progress")
        prog_layout = QVBoxLayout()
        
        self.prog_stream = QProgressBar()
        prog_layout.addWidget(QLabel("Overall Playlist Progress:"))
        prog_layout.addWidget(self.prog_stream)
        
        self.prog_buffer = QProgressBar()
        self.prog_buffer.setRange(0, 512)
        self.prog_buffer.setFormat("Hardware Command Buffer: %v / 512")
        prog_layout.addWidget(self.prog_buffer)
        grp_prog.setLayout(prog_layout)
        layout.addWidget(grp_prog)

        # Control Buttons
        ctrl_layout = QHBoxLayout()
        btn_start = QPushButton("START TRAJECTORY")
        btn_start.setMinimumHeight(50)
        btn_start.setStyleSheet("background-color: #27ae60; color: white; font-weight: bold; font-size: 14px;")
        btn_start.clicked.connect(self.start_streaming)
        
        btn_abort = QPushButton("ABORT")
        btn_abort.setMinimumHeight(50)
        btn_abort.setStyleSheet("background-color: #e67e22; color: white; font-weight: bold; font-size: 14px;")
        btn_abort.clicked.connect(self.abort_stream)
        
        ctrl_layout.addWidget(btn_start)
        ctrl_layout.addWidget(btn_abort)
        layout.addLayout(ctrl_layout)

    def setup_settings_tab(self, parent):
        layout = QVBoxLayout(parent)
        
        # Logging Config
        grp_log = QGroupBox("Data Logging")
        log_layout = QGridLayout()
        
        self.txt_log_name = QLineEdit()
        default_ts = f"exp_{datetime.datetime.now().strftime('%H%M')}.csv"
        self.txt_log_name.setText(default_ts)

        btn_apply_log = QPushButton("Update Log Filename")
        btn_apply_log.clicked.connect(self.cmd_update_log_name)

        log_layout.addWidget(QLabel("Output File:"), 0, 0)
        log_layout.addWidget(self.txt_log_name, 0, 1)
        log_layout.addWidget(btn_apply_log, 0, 2)
        grp_log.setLayout(log_layout)
        layout.addWidget(grp_log)

        # Plot Config
        grp_plot = QGroupBox("Visualization Settings")
        plot_layout = QHBoxLayout()
        
        self.spin_points = QSpinBox()
        self.spin_points.setRange(100, 10000)
        self.spin_points.setValue(self.plot_history_size)
        self.spin_points.setSingleStep(100)
        self.spin_points.valueChanged.connect(self.update_plot_settings)
        
        plot_layout.addWidget(QLabel("Rolling History Window (points):"))
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
        self.worker.error_occurred.connect(lambda e: QMessageBox.warning(self, "Error", e)) 
        
        self.thread.start()

    def load_file(self):
        fname, _ = QFileDialog.getOpenFileName(self, "Open File", "", "Files (*.csv *.gcode *.nc *.txt)")
        if not fname: return

        self.loaded_filename = fname
        ext = os.path.splitext(fname)[1].lower()
        self.lbl_filename.setText(f"Loading {os.path.basename(fname)}...")
        QApplication.processEvents()

        try:
            if ext == '.csv':
                self.pending_pgm = None 
                self.pending_moves = parse_csv_trajectory(fname)
                self.lbl_filename.setText(f"{os.path.basename(fname)} [CSV: {len(self.pending_moves)} pts]")
            
            elif ext in ['.gcode', '.nc', '.txt']:
                pgm_lines, teensy_moves = parse_gcode_to_dual_streams(fname)
                self.pending_pgm = pgm_lines
                self.pending_moves = teensy_moves
                self.lbl_filename.setText(f"{os.path.basename(fname)} [G-Code: {len(teensy_moves)} moves]")

            self.prog_stream.setMaximum(len(self.pending_moves))
            self.prog_stream.setValue(0)
            
        except Exception as e:
            self.lbl_filename.setText("Error loading file")
            QMessageBox.critical(self, "Load Error", str(e))

    def start_streaming(self):
        if not self.pending_moves:
            return

        print("Starting execution sequence...")
        
        # 1. Clear Old Buffer (Fixes the "System Locked" issue)
        self.worker.send_command(CMD_CLEAR_QUEUE)
        
        # 2. Arm Teensy (Now includes Pre-load)
        # This will fill the first 32 buffer slots immediately
        self.worker.queue_stream(self.pending_moves)
        
        # 3. Prepare Aerotech (If G-Code mode)
        driver = None
        pgm_filename = "diw_job.pgm"
        
        if self.pending_pgm:
            driver = AerotechDriver() 
            save_path = os.path.join(driver.user_files_path, pgm_filename)
            folder_path = os.path.dirname(save_path)
            if not os.path.exists(folder_path):
                os.makedirs(folder_path, exist_ok=True)

            try:
                with open(save_path, "w") as f:
                    f.write("\n".join(self.pending_pgm))
            except Exception as e:
                QMessageBox.critical(self, "Write Error", f"Failed to save PGM file: {e}")
                return

        # 4. FIRE (Synchronized Start)
        print("Sending FIRE command to Teensy...")
        # Because we pre-loaded, the buffer is guaranteed not empty.
        self.worker.send_command(CMD_START_QUEUE) 

        if driver and self.pending_pgm:
            print("Triggering Stage Motion...")
            success, msg = driver.run_pgm(pgm_filename)
            if not success:
                QMessageBox.critical(self, "Stage Error", f"Aerotech refused start: {msg}")
                self.worker.send_command(CMD_EMERGENCY_STOP) 
                return
            print("Stage Started.")
        
        print("Stream Active.")

    def abort_stream(self):
        # 1. Stop Teensy
        self.worker.queue_stream([])
        self.worker.send_command(CMD_CLEAR_QUEUE)
        self.worker.send_command(CMD_EMERGENCY_STOP)
        
        # 2. Stop Aerotech
        if self.pending_pgm:
            try:
                driver = AerotechDriver()
                driver.reset_task(1)
            except:
                pass

    def cmd_clear_errors(self):
        self.worker.send_command(CMD_CLEAR_ERRORS)
        self.lbl_status.setText("Status: Errors Cleared")

    def cmd_set_velocity(self):
        val = self.spin_velocity.value()
        self.worker.send_command(CMD_SET_VELOCITY, val_a=val)

    def cmd_move_relative(self, direction):
        steps = self.spin_steps.value() * direction
        speed = self.spin_velocity.value()
        if speed <= 0: speed = 500.0
        self.worker.send_command(CMD_MOVE_RELATIVE, val_a=steps, val_b=speed)

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
        self.lbl_position.setText(f"{data['position_steps']} steps")
        self.lbl_pressure_val.setText(f"{data['pressure']:.1f} kPa")
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
            # STOP the worker safely
            if self.worker:
                self.worker.stop()
                # Wait for thread to finish (Max 1 sec to prevent hanging)
                if self.thread.isRunning():
                    self.thread.quit()
                    self.thread.wait(1000) 
            
            event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = DIWController()
    window.show()
    sys.exit(app.exec())