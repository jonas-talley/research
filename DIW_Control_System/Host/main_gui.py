import sys
import os
import datetime
import numpy as np
import pyqtgraph as pg
from collections import deque
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QPushButton, QLabel, QLineEdit, QTabWidget, QGroupBox, 
    QProgressBar, QFileDialog, QMessageBox, QSpinBox, QDoubleSpinBox
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

        # Tab 2: Smart Streamer
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
        
        self.spin_velocity = QDoubleSpinBox()
        self.spin_velocity.setRange(-5000.0, 5000.0)
        self.spin_velocity.setValue(self.default_vel)
        self.spin_velocity.setSuffix(" um/s")
        self.spin_velocity.setSingleStep(50.0)
        
        btn_set = QPushButton("Run Constant Velocity")
        btn_set.clicked.connect(self.cmd_set_velocity)
        
        vel_layout.addWidget(QLabel("Speed:"))
        vel_layout.addWidget(self.spin_velocity)
        vel_layout.addWidget(btn_set)
        grp_vel.setLayout(vel_layout)
        layout.addWidget(grp_vel)

        # B. Position Control
        grp_pos = QGroupBox("2. Step Control (Move Relative)")
        pos_layout = QHBoxLayout()
        
        self.spin_steps = QSpinBox()
        self.spin_steps.setRange(1, 1000000)
        self.spin_steps.setValue(self.default_steps)
        self.spin_steps.setSuffix(" steps")
        self.spin_steps.setSingleStep(100)
        
        btn_move_p = QPushButton("Move (+)")
        btn_move_p.clicked.connect(lambda: self.cmd_move_relative(1))
        
        btn_move_n = QPushButton("Move (-)")
        btn_move_n.clicked.connect(lambda: self.cmd_move_relative(-1))
        
        btn_zero = QPushButton("Zero Pos")
        btn_zero.setStyleSheet("background-color: #DDD;")
        btn_zero.clicked.connect(lambda: self.worker.send_command(CMD_ZERO_POSITION))

        pos_layout.addWidget(QLabel("Distance:"))
        pos_layout.addWidget(self.spin_steps)
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
        self.prog_buffer.setRange(0, 512)
        self.prog_buffer.setFormat("Hardware Buffer: %v / 512")
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
        
        # 2. Pre-load the teensy
        # This will fill the first like 400 buffer slots immediately
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
            if not success: #I have never seen this case activate, unsure if it would trigger.
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