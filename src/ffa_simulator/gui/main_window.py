import logging
import asyncio
from PySide6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                                QPushButton, QLabel, QComboBox, QTextEdit, QSplitter,
                                QTabWidget)
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
        
        if lat != 0 and lon != 0:
            # Set home position on first valid GPS
            if self.mission_control_panel.home_position is None:
                self.mission_control_panel.set_home_position(lat, lon)
                logger.info(f"Home position set: {lat:.6f}, {lon:.6f}")
            
            # Update current position
            self.mission_control_panel.update_current_position(lat, lon)
        
        # Update mission progress panel (Phase 2.2)
        self._update_mission_progress(telemetry)
    
    def _update_mission_progress(self, telemetry):
        """
        Update mission progress panel with telemetry data.
        Phase 2.2 feature.
        """
        # ULTRA DEBUG - Print to console with print() to bypass logging
        speed_from_telemetry = telemetry['groundspeed']
        print(f"!!! MAIN_WINDOW: groundspeed from telemetry dict = {speed_from_telemetry} m/s !!!")
        
        # Prepare data for mission progress panel
        progress_data = {
            'current_position': (
                telemetry['latitude'],
                telemetry['longitude'],
                telemetry['altitude']
            ),
            'current_seq': telemetry.get('current_seq'),
            'ground_speed': telemetry['groundspeed'],
            'mode': telemetry['mode']
        }
        
        print(f"!!! MAIN_WINDOW: progress_data['ground_speed'] = {progress_data['ground_speed']} m/s !!!")
        
        # Update the mission progress display
        self.mission_control_panel.update_mission_progress(progress_data)
    
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
