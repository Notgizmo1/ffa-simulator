import logging
import asyncio
import csv
import time
import os
from PySide6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                                QPushButton, QLabel, QComboBox, QTextEdit, QSplitter,
                                QTabWidget, QFileDialog, QMessageBox)
from PySide6.QtCore import Qt, QTimer
from ffa_simulator.orchestrator.process_manager import ProcessManager
from ffa_simulator.telemetry.mavlink_bridge import TelemetryBridge
from ffa_simulator.gui.telemetry_panel import TelemetryPanel
from ffa_simulator.gui.mission_control_panel import MissionControlPanel

logger = logging.getLogger(__name__)

class LogHandler(logging.Handler):
    """Custom handler to route Python logs to GUI QTextEdit"""
    def __init__(self, text_widget):
        super().__init__()
        self.text_widget = text_widget
        
    def emit(self, record):
        msg = self.format(record)
        try:
            self.text_widget.append(msg)
        except:
            pass

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.process_manager = ProcessManager()
        self.telemetry_bridge = TelemetryBridge()
        self.telemetry_log = []  # List of dicts for CSV export
        self.recording_start_time = None
        self.init_ui()
        self.setup_logging()
        self.setup_telemetry_timer()
        
    def init_ui(self):
        self.setWindowTitle("FFA Simulator - ArduPilot VTOL Training")
        self.setGeometry(100, 100, 1600, 900)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        main_layout = QHBoxLayout(central_widget)
        
        left_panel = self.create_control_panel()
        right_panel = self.create_right_panel()
        
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left_panel)
        splitter.addWidget(right_panel)
        splitter.setSizes([400, 1200])
        
        main_layout.addWidget(splitter)
        
        logger.info("Main window UI initialized")
    
    def setup_logging(self):
        """Connect Python logging to GUI log display"""
        gui_handler = LogHandler(self.log_display)
        gui_handler.setLevel(logging.INFO)
        formatter = logging.Formatter('%(asctime)s | %(levelname)-5s | %(message)s', 
                                     datefmt='%H:%M:%S')
        gui_handler.setFormatter(formatter)
        root_logger = logging.getLogger()
        root_logger.addHandler(gui_handler)
        logger.info("GUI logging initialized")
    
    def setup_telemetry_timer(self):
        """Setup timer to update telemetry display"""
        self.telemetry_timer = QTimer()
        self.telemetry_timer.timeout.connect(self.update_telemetry_display)
        self.telemetry_timer.setInterval(100)
    
    def create_control_panel(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        title = QLabel("FFA Simulator Control")
        title.setStyleSheet("font-size: 16pt; font-weight: bold;")
        layout.addWidget(title)
        
        self.status_label = QLabel("Status: Stopped")
        self.status_label.setStyleSheet("padding: 10px; background-color: #ff6b6b; color: white; border-radius: 5px;")
        layout.addWidget(self.status_label)
        
        self.start_button = QPushButton("▶ Start Simulation")
        self.start_button.clicked.connect(self.start_simulation)
        self.start_button.setMinimumHeight(40)
        layout.addWidget(self.start_button)
        
        self.stop_button = QPushButton("■ Stop Simulation")
        self.stop_button.clicked.connect(self.stop_simulation)
        self.stop_button.setEnabled(False)
        self.stop_button.setMinimumHeight(40)
        layout.addWidget(self.stop_button)
        
        layout.addWidget(QLabel("\nVehicle:"))
        self.vehicle_combo = QComboBox()
        self.vehicle_combo.addItems(["QuadPlane", "Aether VTOL", "Vanguard VTOL"])
        layout.addWidget(self.vehicle_combo)
        
        layout.addWidget(QLabel("\nScenario:"))
        self.scenario_combo = QComboBox()
        self.scenario_combo.addItems(["Basic Hover", "Simple Waypoints", "VTOL Transition"])
        layout.addWidget(self.scenario_combo)
        
        layout.addWidget(QLabel("\nSystem Log:"))
        self.log_display = QTextEdit()
        self.log_display.setReadOnly(True)
        self.log_display.setMaximumHeight(300)
        layout.addWidget(self.log_display)
        
        # Export flight data button
        self.export_button = QPushButton("Export Flight Data (CSV)")
        self.export_button.clicked.connect(self.export_telemetry)
        self.export_button.setEnabled(False)
        self.export_button.setStyleSheet("background-color: #228be6; color: white; font-weight: bold; padding: 8px;")
        layout.addWidget(self.export_button)
        
        layout.addStretch()
        
        return panel
    
    def create_right_panel(self):
        """Create tabbed interface for Mission Control and Telemetry"""
        tab_widget = QTabWidget()
        
        # Tab 1: Mission Control
        self.mission_control_panel = MissionControlPanel()
        tab_widget.addTab(self.mission_control_panel, "Mission Control")
        
        # Tab 2: Telemetry
        self.telemetry_panel = TelemetryPanel()
        tab_widget.addTab(self.telemetry_panel, "Telemetry Plots")
        
        # Connect mission control signals
        self.mission_control_panel.mission_upload_requested.connect(self.upload_mission)
        self.mission_control_panel.command_requested.connect(self.send_command)
        
        return tab_widget
    
    def update_telemetry_display(self):
        """Update telemetry panel with current data"""
        if not self.telemetry_bridge.connected:
            return
        
        telemetry = self.telemetry_bridge.get_telemetry()
        
        # Update telemetry plots
        self.telemetry_panel.update_display(telemetry)
        
        # Update mission control with current position
        lat = telemetry['latitude']
        lon = telemetry['longitude']
        alt = telemetry['altitude']
        groundspeed = telemetry['groundspeed']
        
        if lat != 0 and lon != 0:
            # Set home position on first valid GPS
            if self.mission_control_panel.home_position is None:
                self.mission_control_panel.set_home_position(lat, lon)
                logger.info(f"Home position set: {lat:.6f}, {lon:.6f}")
            
            # Update current position on map
            self.mission_control_panel.update_current_position(lat, lon)
        
        # Update altitude and position displays (Phase 3.1)
        self.mission_control_panel.update_altitude_display(alt)
        self.mission_control_panel.update_position_display(lat, lon)
        
        # ── Phase 3.2: Mission Progress Tracking ──
        mission_state = self.telemetry_bridge.get_mission_state()
        current_seq = mission_state['current_seq']
        total_wps = mission_state['total_wps']
        waypoints = mission_state['waypoints']
        mission_active = mission_state['mission_active']
        mission_complete = mission_state.get('mission_complete', False)
        
        # Update mission progress widget (distance, ETA, bearing, progress bar)
        self.mission_control_panel.update_mission_progress(
            lat, lon, groundspeed,
            current_seq, total_wps,
            waypoints, mission_active, mission_complete
        )
        
        # Update active waypoint marker on map
        # ArduPilot seq: 0=home, 1=takeoff, 2+=user WPs → user_wp_index = current_seq - 2
        if waypoints and current_seq >= 2:
            user_wp_index = current_seq - 2
            self.mission_control_panel.update_active_waypoint_marker(waypoints, user_wp_index)
        
        # ── Mission Complete Detection ──
        if mission_complete and telemetry['armed']:
            if not hasattr(self, '_mission_complete_handled') or not self._mission_complete_handled:
                self._mission_complete_handled = True
                logger.info("MISSION COMPLETE — All waypoints reached. Commanding RTL.")
                self.send_command("RTL", {})
        elif not mission_complete:
            self._mission_complete_handled = False
        
        # ── Telemetry Recording (for CSV export) ──
        if self.recording_start_time is None:
            self.recording_start_time = time.time()
        
        # ── Battery Failsafe ──
        battery_pct = telemetry.get('battery_remaining', 100)
        if battery_pct <= 0 and telemetry['armed']:
            # 0% battery — emergency motor kill
            if not hasattr(self, '_force_disarm_sent') or not self._force_disarm_sent:
                self._force_disarm_sent = True
                logger.warning("BATTERY DEAD: 0% — FORCE DISARM (motor kill)")
                self.send_command("FORCE_DISARM", {})
        elif battery_pct <= 10 and battery_pct > 0 and telemetry['armed']:
            self._force_disarm_sent = False
            if not self.telemetry_bridge.battery_failsafe_triggered:
                self.telemetry_bridge.battery_failsafe_triggered = True
                logger.warning(f"BATTERY CRITICAL: {battery_pct}% — Commanding emergency LAND")
            
            # Persistently enforce LAND mode — re-command every 5 seconds if overridden
            current_mode = telemetry.get('mode', '')
            if current_mode not in ('LAND', 'QLAND', 'RTL'):
                if not hasattr(self, '_last_failsafe_land') or (time.time() - self._last_failsafe_land) > 5:
                    self._last_failsafe_land = time.time()
                    logger.warning(f"BATTERY FAILSAFE: Overriding {current_mode} → LAND ({battery_pct}%)")
                    self.send_command("LAND", {})
        elif battery_pct > 20:
            self.telemetry_bridge.battery_failsafe_triggered = False
            self._force_disarm_sent = False
        
        elapsed = time.time() - self.recording_start_time
        self.telemetry_log.append({
            'time_s': round(elapsed, 2),
            'mode': telemetry['mode'],
            'armed': telemetry['armed'],
            'altitude_m': round(telemetry['altitude'], 2),
            'groundspeed_ms': round(groundspeed, 2),
            'latitude': round(lat, 8),
            'longitude': round(lon, 8),
            'roll_deg': round(telemetry.get('roll', 0), 2),
            'pitch_deg': round(telemetry.get('pitch', 0), 2),
            'yaw_deg': round(telemetry.get('yaw', 0), 2),
            'battery_v': round(telemetry.get('battery_voltage', 0), 2),
            'battery_pct': telemetry.get('battery_remaining', 0),
            'gps_fix': telemetry.get('gps_fix', 0),
            'gps_sats': telemetry.get('gps_satellites', 0),
            'wp_seq': current_seq,
        })
    
    def upload_mission(self, waypoints):
        """Upload mission to autopilot"""
        logger.info(f"Uploading {len(waypoints)} waypoints to autopilot...")
        
        async def upload_async():
            try:
                success = await self.telemetry_bridge.upload_mission(waypoints)
                if success:
                    logger.info("Mission uploaded successfully!")
                else:
                    logger.error("Mission upload failed")
            except Exception as e:
                logger.error(f"Mission upload error: {e}")
        
        asyncio.create_task(upload_async())
    
    def send_command(self, command, params):
        """Send flight command to autopilot"""
        # Block dangerous commands during battery failsafe
        if self.telemetry_bridge.battery_failsafe_triggered:
            blocked_commands = ("AUTO", "ARM", "ARM_FORCE", "TAKEOFF", "CHANGE_ALTITUDE", "GUIDED")
            if command in blocked_commands:
                logger.warning(f"BLOCKED: {command} rejected — battery failsafe active. Land or disarm first.")
                return
        
        logger.info(f"Sending command: {command} with params: {params}")
        
        async def command_async():
            try:
                success = await self.telemetry_bridge.send_command(command, params)
                if success:
                    logger.info(f"Command {command} sent successfully")
                else:
                    logger.warning(f"Command {command} failed")
            except Exception as e:
                logger.error(f"Command error: {e}")
        
        asyncio.create_task(command_async())
    
    def start_simulation(self):
        logger.info("Ready for simulation")
        
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(False)
        self.vehicle_combo.setEnabled(False)
        self.scenario_combo.setEnabled(False)
        
        self.status_label.setText("Status: Starting...")
        self.status_label.setStyleSheet("padding: 10px; background-color: #ffd43b; color: black; border-radius: 5px;")
        
        vehicle = self.vehicle_combo.currentText().lower().replace(" ", "_")
        scenario = self.scenario_combo.currentText().lower().replace(" ", "_")
        
        async def start_async():
            try:
                await self.process_manager.start_simulation(vehicle, scenario)
                
                logger.info("Waiting 3 seconds for SITL to stabilize...")
                await asyncio.sleep(3)
                
                wsl_ip = "172.24.119.69"
                # MAVProxy broadcasts UDP to Windows on port 14550
                connection_string = "udpin:0.0.0.0:14550"
                
                logger.info(f"Connecting to MAVLink: {connection_string}")
                connected = await self.telemetry_bridge.connect(connection_string)
                
                if connected:
                    logger.info("Telemetry connected")
                    self.telemetry_timer.start()
                    self.telemetry_log.clear()
                    self.recording_start_time = None
                    self.export_button.setEnabled(True)
                else:
                    logger.warning("Telemetry connection failed")
                
                self.start_button.setEnabled(False)
                self.stop_button.setEnabled(True)
                self.status_label.setText("Status: Running")
                self.status_label.setStyleSheet("padding: 10px; background-color: #51cf66; color: white; border-radius: 5px;")
                
            except Exception as e:
                logger.error(f"Failed to start: {e}")
                self.start_button.setEnabled(True)
                self.stop_button.setEnabled(False)
                self.vehicle_combo.setEnabled(True)
                self.scenario_combo.setEnabled(True)
                self.status_label.setText("Status: Error")
                self.status_label.setStyleSheet("padding: 10px; background-color: #ff6b6b; color: white; border-radius: 5px;")
        
        asyncio.create_task(start_async())
    
    def export_telemetry(self):
        """Export recorded telemetry data to CSV file"""
        if not self.telemetry_log:
            QMessageBox.warning(self, "No Data", "No telemetry data recorded yet.")
            return
        
        # Default filename with timestamp
        default_name = f"flight_data_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        filename, _ = QFileDialog.getSaveFileName(
            self, "Export Flight Data", default_name, "CSV Files (*.csv);;All Files (*)"
        )
        
        if filename:
            try:
                fieldnames = list(self.telemetry_log[0].keys())
                with open(filename, 'w', newline='') as f:
                    writer = csv.DictWriter(f, fieldnames=fieldnames)
                    writer.writeheader()
                    writer.writerows(self.telemetry_log)
                
                rows = len(self.telemetry_log)
                duration = self.telemetry_log[-1]['time_s'] if self.telemetry_log else 0
                logger.info(f"Telemetry exported: {rows} samples, {duration:.1f}s to {filename}")
                QMessageBox.information(
                    self, "Export Complete", 
                    f"Exported {rows} data points ({duration:.0f}s of flight data)\n\nSaved to: {filename}"
                )
            except Exception as e:
                logger.error(f"Export failed: {e}")
                QMessageBox.critical(self, "Export Error", f"Failed to export: {e}")
    
    def stop_simulation(self):
        logger.info("Stopping simulation...")
        
        self.stop_button.setEnabled(False)
        self.telemetry_timer.stop()
        self.telemetry_panel.reset()
        
        async def stop_async():
            try:
                await self.telemetry_bridge.disconnect()
                await self.process_manager.stop_simulation()
                
                self.start_button.setEnabled(True)
                self.stop_button.setEnabled(False)
                self.vehicle_combo.setEnabled(True)
                self.scenario_combo.setEnabled(True)
                self.status_label.setText("Status: Stopped")
                self.status_label.setStyleSheet("padding: 10px; background-color: #ff6b6b; color: white; border-radius: 5px;")
                
            except Exception as e:
                logger.error(f"Error stopping: {e}")
                self.stop_button.setEnabled(True)
        
        asyncio.create_task(stop_async())
