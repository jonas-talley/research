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
from aerotech_worker import AerotechWorker

class DIWController(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("DIW Control System - Absolute Mode")
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
        self.pending_moves = [] 

        self.aero_thread = None
        self.aero_worker = None
        
        self.setup_ui()
        self.start_connection()

        self.start_aerotech_connection()

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

        # Tab 2: Streamer
        self.tab_stream = QWidget()
        self.setup_stream_tab(self.tab_stream)
        tabs.addTab(self.tab_stream, "Trajectory Streamer")
        
        # Tab 3: Session Config
        self.tab_settings = QWidget()
        self.setup_settings_tab(self.tab_settings)
        tabs.addTab(self.tab_settings, "Session Config")

        # 3. Plots
        self.setup_plots(layout)

        # Tab 4: Motion Platform (Aerotech)
        self.tab_motion = QWidget()
        self.setup_motion_tab(self.tab_motion)
        tabs.addTab(self.tab_motion, "Motion Platform")

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

        # B. Position Control (Move Relative)
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
        
        # File Loading
        file_layout = QHBoxLayout()
        self.lbl_filename = QLabel("No file loaded")
        btn_load = QPushButton("Load CSV...")
        btn_load.clicked.connect(self.load_csv)
        file_layout.addWidget(btn_load)
        file_layout.addWidget(self.lbl_filename)
        layout.addLayout(file_layout)

        # Controls
        ctrl_layout = QHBoxLayout()
        btn_start = QPushButton("Start Stream")
        btn_start.setStyleSheet("background-color: green; color: white;")
        btn_start.clicked.connect(self.start_streaming)
        btn_abort = QPushButton("Abort")
        btn_abort.clicked.connect(self.abort_stream)
        ctrl_layout.addWidget(btn_start)
        ctrl_layout.addWidget(btn_abort)
        layout.addLayout(ctrl_layout)

        # Progress
        self.prog_stream = QProgressBar()
        layout.addWidget(QLabel("CSV Progress:"))
        layout.addWidget(self.prog_stream)
        
        self.prog_buffer = QProgressBar()
        self.prog_buffer.setRange(0, 64)
        self.prog_buffer.setFormat("Buffer: %v / 64")
        layout.addWidget(self.prog_buffer)
        layout.addStretch()

    def setup_settings_tab(self, parent):
        layout = QVBoxLayout(parent)
        
        # 1. Data Logging
        grp_log = QGroupBox("Data Logging")
        log_layout = QVBoxLayout()
        
        # Current Log Label
        self.lbl_log_status = QLabel("Logging to: Default")
        self.lbl_log_status.setStyleSheet("color: blue;")
        log_layout.addWidget(self.lbl_log_status)

        # File Change Inputs
        file_input_layout = QHBoxLayout()
        self.txt_log_name = QLineEdit()
        self.txt_log_name.setPlaceholderText("Enter new filename (e.g. experiment_1.csv)")
        
        # Pre-fill with a timestamp
        default_ts = f"exp_{datetime.datetime.now().strftime('%H%M')}.csv"
        self.txt_log_name.setText(default_ts)

        btn_apply_log = QPushButton("Apply / Switch Log File")
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
        
        plot_layout.addWidget(QLabel("History Points (Time Window):"))
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
        self.p0.setLabel('left', 'Steps')
        self.curve_pos = self.p0.plot(pen=pg.mkPen('m', width=2))
        
        self.plot_widget.nextRow()

        # Plot 1: Velocity
        self.p1 = self.plot_widget.addPlot(title="Velocity (um/s)")
        self.p1.showGrid(x=True, y=True)
        self.p1.setLabel('left', 'um/s')
        self.p1.addLegend()
        self.curve_vel_tgt = self.p1.plot(pen=pg.mkPen('g', width=2, style=Qt.DashLine), name='Target')
        self.curve_vel_cur = self.p1.plot(pen=pg.mkPen('y', width=2), name='Current')
        self.p1.setXLink(self.p0)

        self.plot_widget.nextRow()

        # Plot 2: Pressure
        self.p2 = self.plot_widget.addPlot(title="Pressure (kPa)")
        self.p2.showGrid(x=True, y=True)
        self.p2.setLabel('left', 'kPa')
        self.p2.setLabel('bottom', 'Time (s)')
        self.curve_pressure = self.p2.plot(pen=pg.mkPen('c', width=2))
        self.p2.setXLink(self.p0)

        parent_layout.addWidget(self.plot_widget)

    def setup_motion_tab(self, parent):
        layout = QVBoxLayout(parent)
        
        # Status Header
        self.lbl_aero_status = QLabel("Aerotech: Disconnected")
        self.lbl_aero_status.setStyleSheet("font-weight: bold; color: red;")
        layout.addWidget(self.lbl_aero_status)

        # Position Readout
        self.lbl_aero_pos = QLabel("X: 0.000 | Y: 0.000 | Z: 0.000")
        self.lbl_aero_pos.setStyleSheet("font-size: 20px; font-weight: bold; color: blue; border: 1px solid gray; padding: 10px;")
        layout.addWidget(self.lbl_aero_pos)

        # --- ENABLE CONTROLS (NEW) ---
        grp_power = QGroupBox("Motor Power")
        power_layout = QHBoxLayout()
        
        btn_enable = QPushButton("ENABLE ALL AXES")
        btn_enable.setStyleSheet("background-color: #90EE90; font-weight: bold; height: 35px;")
        btn_enable.clicked.connect(self.cmd_enable_motors)
        
        btn_disable = QPushButton("DISABLE ALL")
        btn_disable.setStyleSheet("background-color: #FFB6C1; height: 35px;")
        btn_disable.clicked.connect(self.cmd_disable_motors)
        
        power_layout.addWidget(btn_enable)
        power_layout.addWidget(btn_disable)
        grp_power.setLayout(power_layout)
        layout.addWidget(grp_power)
        # -----------------------------

        # Jog Controls
        grp_jog = QGroupBox("Jog Controls")
        grid = QVBoxLayout()
        
        # Speed & Step Inputs
        param_layout = QHBoxLayout()
        self.txt_aero_speed = QLineEdit("10")
        self.txt_aero_speed.setPlaceholderText("Speed (mm/s)")
        self.txt_aero_step = QLineEdit("1.0")
        self.txt_aero_step.setPlaceholderText("Step (mm)")
        
        param_layout.addWidget(QLabel("Speed (mm/s):"))
        param_layout.addWidget(self.txt_aero_speed)
        param_layout.addWidget(QLabel("Step (mm):"))
        param_layout.addWidget(self.txt_aero_step)
        grid.addLayout(param_layout)

        # Axis Buttons
        def make_axis_row(axis):
            row = QHBoxLayout()
            row.addWidget(QLabel(f"{axis} Axis:", alignment=Qt.AlignRight))
            
            btn_neg = QPushButton(f"- {axis}")
            btn_neg.setMinimumHeight(40)
            btn_neg.clicked.connect(lambda: self.cmd_jog_aerotech(axis, -1))
            
            btn_pos = QPushButton(f"+ {axis}")
            btn_pos.setMinimumHeight(40)
            btn_pos.clicked.connect(lambda: self.cmd_jog_aerotech(axis, 1))
            
            row.addWidget(btn_neg)
            row.addWidget(btn_pos)
            return row

        grid.addLayout(make_axis_row("X"))
        grid.addLayout(make_axis_row("Y"))
        grid.addLayout(make_axis_row("Z"))
        
        # Stop Button
        btn_stop = QPushButton("STOP MOTION")
        btn_stop.setStyleSheet("background-color: red; color: white; font-weight: bold; height: 50px;")
        btn_stop.clicked.connect(self.cmd_stop_aerotech)
        grid.addWidget(btn_stop)

        grp_jog.setLayout(grid)
        layout.addWidget(grp_jog)
        layout.addStretch()

    # --- Logic ---
    def start_connection(self):
        self.thread = QThread()
        self.worker = SerialWorker(self.port_name)
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.run)
        
        # Signals
        self.worker.telemetry_received.connect(self.update_plots)
        self.worker.connected.connect(lambda p: self.lbl_status.setText(f"Connected: {p}"))
        self.worker.stream_progress.connect(self.prog_stream.setValue)
        self.worker.log_status_changed.connect(self.lbl_log_status.setText) # Feedback connection
        
        self.thread.start()

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
            QMessageBox.warning(self, "Error", "Invalid Step or Speed value")

    def cmd_stop(self):
        self.worker.send_command(CMD_SET_VELOCITY, val_a=0.0)
        self.abort_stream()

    def cmd_update_log_name(self):
        fname = self.txt_log_name.text().strip()
        if not fname:
            QMessageBox.warning(self, "Error", "Filename cannot be empty")
            return
        if not fname.endswith(".csv"):
            fname += ".csv"
        
        # Send to worker
        # Note: calling this directly is safe because the worker method uses mutex locks
        self.worker.change_log_file(fname)

    def update_plot_settings(self):
        new_size = self.spin_points.value()
        self.plot_history_size = new_size
        for k in self.data_history:
            self.data_history[k] = deque(self.data_history[k], maxlen=new_size)

    def load_csv(self):
        fname, _ = QFileDialog.getOpenFileName(self, "Open CSV", "", "CSV Files (*.csv)")
        if fname:
            try:
                df = pd.read_csv(fname)
                times = df['Time'].values
                vels = df['Velocity'].values
                moves = []
                for i in range(len(times)-1):
                    dt = (times[i+1] - times[i]) * 1000.0
                    if dt > 0: moves.append((float(vels[i]), float(dt)))
                self.pending_moves = moves
                self.lbl_filename.setText(f"{os.path.basename(fname)} ({len(moves)} pts)")
                self.prog_stream.setMaximum(len(moves))
            except Exception as e:
                QMessageBox.critical(self, "Error", str(e))

    def start_streaming(self):
        if self.pending_moves:
            self.worker.queue_stream(self.pending_moves)

    def abort_stream(self):
        self.worker.queue_stream([])
        self.worker.send_command(CMD_CLEAR_QUEUE)

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

    def start_aerotech_connection(self):
        self.aero_thread = QThread()
        self.aero_worker = AerotechWorker()
        self.aero_worker.moveToThread(self.aero_thread)
        
        self.aero_thread.started.connect(self.aero_worker.run)
        
        # Signals
        self.aero_worker.connected.connect(lambda: self.lbl_aero_status.setText("Aerotech: Connected"))
        self.aero_worker.connected.connect(lambda: self.lbl_aero_status.setStyleSheet("color: green; font-weight: bold;"))
        self.aero_worker.connection_error.connect(lambda e: self.lbl_aero_status.setText(f"Aerotech Error: {e}"))
        self.aero_worker.pos_updated.connect(self.update_aero_pos)
        
        self.aero_thread.start()

    @Slot(float, float, float)
    def update_aero_pos(self, x, y, z):
        self.lbl_aero_pos.setText(f"X: {x:.3f} | Y: {y:.3f} | Z: {z:.3f}")

    def cmd_jog_aerotech(self, axis, direction):
        try:
            step = float(self.txt_aero_step.text())
            speed = float(self.txt_aero_speed.text())
            
            # Send to worker
            # Note: Since this calls a .NET function, strictly speaking we should emit a signal 
            # connected to the worker slot, but for this simple test, direct call is usually fine.
            # Ideally: self.jog_signal.emit(axis, step*direction, speed)
            self.aero_worker.move_relative(axis, step * direction, speed)
        except ValueError:
            pass

    def cmd_stop_aerotech(self):
        self.aero_worker.stop_all()

    # Update closeEvent to clean up Aerotech too
    def closeEvent(self, event):
        # Stop Serial
        self.worker.disconnect_serial()
        self.thread.quit()
        self.thread.wait()
        
        # Stop Aerotech
        if self.aero_worker:
            self.aero_worker.stop_worker()
        if self.aero_thread:
            self.aero_thread.quit()
            self.aero_thread.wait()
            
        event.accept()

    def cmd_enable_motors(self):
        if self.aero_worker:
            self.aero_worker.enable_axis("X")
            self.aero_worker.enable_axis("Y")
            self.aero_worker.enable_axis("Z")

    def cmd_disable_motors(self):
        if self.aero_worker:
            self.aero_worker.disable_axis("X")
            self.aero_worker.disable_axis("Y")
            self.aero_worker.disable_axis("Z")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = DIWController()
    window.show()
    sys.exit(app.exec())