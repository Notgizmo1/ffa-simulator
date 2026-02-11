"""
Main window for FFA Simulator.
"""

from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QComboBox, QTextEdit, QGroupBox, QTabWidget
)
from PySide6.QtCore import QTimer, Slot
from PySide6.QtGui import QTextCursor

from ..orchestrator.process_manager import ProcessManager
from ..telemetry.mavlink_bridge import MAVLinkBridge  # FIXED: Was TelemetryBridge
from .telemetry_panel import TelemetryPanel
from .mission_control_panel import MissionControlPanel


class MainWindow(QMainWindow):
    """Main application window."""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("FFA Simulator - ArduPilot VTOL Training")
        self.setGeometry(100, 100, 1600, 900)
        
        # Core components
        self.process_manager = ProcessManager()
        self.mavlink_bridge = MAVLinkBridge()  # FIXED: Was TelemetryBridge()
        
        # Track initialization state
        self.home_position_set = False
        self.current_groundspeed = 0.0
        
        # Setup UI
        self._init_ui()
        
        # Setup signal connections
        self._setup_connections()
        
        # Setup timers
        self._setup_timers()
    
    def _init_ui(self):
        """Initialize UI components."""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        layout = QHBoxLayout(central_widget)
        
        # Left panel: Control + Log
        left_panel = QVBoxLayout()
        left_panel.addWidget(self._create_control_panel())
        left_panel.addWidget(self._create_log_panel())
        
        # Right panel: Tabbed interface
        right_panel = QVBoxLayout()
        self.tab_widget = QTabWidget()
        
        # Tab 1: Mission Control
        self.mission_control = MissionControlPanel()
        self.tab_widget.addTab(self.mission_control, "Mission Control")
        
        # Tab 2: Telemetry Plots
        self.telemetry_panel = TelemetryPanel()
        self.tab_widget.addTab(self.telemetry_panel, "Telemetry Plots")
        
        right_panel.addWidget(self.tab_widget)
        
        layout.addLayout(left_panel, 1)
        layout.addLayout(right_panel, 3)
    
    def _create_control_panel(self) -> QGroupBox:
        """Create simulation control panel."""
        group = QGroupBox("FFA Simulator Control")
        layout = QVBoxLayout(group)
        
        # Status indicator
        self.status_label = QLabel("Status: Idle")
        self.status_label.setStyleSheet(
            "background-color: #gray; color: white; padding: 10px; "
            "font-weight: bold; border-radius: 5px;"
        )
        layout.addWidget(self.status_label)
        
        # Start button
        self.start_btn = QPushButton("▶ Start Simulation")
        self.start_btn.clicked.connect(self._start_simulation)
        layout.addWidget(self.start_btn)
        
        # Connect button (manual MAVLink connection)
        self.connect_btn = QPushButton("🔌 Connect MAVLink")
        self.connect_btn.clicked.connect(self._connect_mavlink)
        self.connect_btn.setEnabled(False)
        layout.addWidget(self.connect_btn)
        
        # Stop button
        self.stop_btn = QPushButton("■ Stop Simulation")
        self.stop_btn.clicked.connect(self._stop_simulation)
        self.stop_btn.setEnabled(False)
        layout.addWidget(self.stop_btn)
        
        # Vehicle selection
        layout.addWidget(QLabel("Vehicle:"))
        self.vehicle_combo = QComboBox()
        self.vehicle_combo.addItems(["QuadPlane"])
        layout.addWidget(self.vehicle_combo)
        
        # Scenario selection
        layout.addWidget(QLabel("Scenario:"))
        self.scenario_combo = QComboBox()
        self.scenario_combo.addItems(["Basic Hover"])
        layout.addWidget(self.scenario_combo)
        
        layout.addStretch()
        
        return group
    
    def _create_log_panel(self) -> QGroupBox:
        """Create system log panel."""
        group = QGroupBox("System Log")
        layout = QVBoxLayout(group)
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(200)
        layout.addWidget(self.log_text)
        
        return group
    
    def _setup_connections(self):
        """Setup signal-slot connections."""
        # Process manager signals
        self.process_manager.process_started.connect(self._on_process_started)
        self.process_manager.process_stopped.connect(self._on_process_stopped)
        self.process_manager.log_message.connect(self._append_log)
        
        # MAVLink signals
        self.mavlink_bridge.connection_status.connect(self._on_connection_status)
        
        # Telemetry Panel signals (TODO: Verify these methods exist in telemetry_panel.py)
        # self.mavlink_bridge.heartbeat_received.connect(self.telemetry_panel.update_heartbeat)
        self.mavlink_bridge.gps_data_received.connect(self._on_gps_data)
        # self.mavlink_bridge.attitude_received.connect(self.telemetry_panel.update_attitude)
        self.mavlink_bridge.position_received.connect(self._on_position_data)
        self.mavlink_bridge.mission_progress_received.connect(self._on_mission_progress)
        
        # Mission control signals
        self.mission_control.mission_upload_requested.connect(self.upload_mission)
        self.mission_control.command_requested.connect(self.send_command)
    
    def _setup_timers(self):
        """Setup update timers."""
        # Telemetry update timer (10 Hz)
        self.telemetry_timer = QTimer()
        self.telemetry_timer.timeout.connect(self._update_telemetry)
        self.telemetry_timer.setInterval(100)  # 100ms = 10 Hz
    
    @Slot()
    def _start_simulation(self):
        """Start SITL simulation."""
        self._append_log("Starting simulation...")
        self.status_label.setText("Status: Starting...")
        self.status_label.setStyleSheet(
            "background-color: #ff9800; color: white; padding: 10px; "
            "font-weight: bold; border-radius: 5px;"
        )
        
        # Start SITL process
        vehicle = self.vehicle_combo.currentText()
        if self.process_manager.start_sitl(vehicle):
            self.start_btn.setEnabled(False)
            self.connect_btn.setEnabled(True)  # Enable manual connection
            self.stop_btn.setEnabled(True)
            
            # Log initialization info (no auto-connection)
            self._append_log("SITL started - waiting for initialization...")
            self._append_log("Click '🔌 Connect MAVLink' when SITL is ready (wait ~20 seconds)")
    
    @Slot()
    def _stop_simulation(self):
        """Stop SITL simulation."""
        self._append_log("Stopping simulation...")
        
        # Stop MAVLink
        import asyncio
        loop = asyncio.get_event_loop()
        loop.create_task(self.mavlink_bridge.stop())
        
        self.telemetry_timer.stop()
        
        # Stop SITL
        self.process_manager.stop_sitl()
        
        # Reset UI
        self.start_btn.setEnabled(True)
        self.connect_btn.setEnabled(False)
        self.stop_btn.setEnabled(False)
        self.status_label.setText("Status: Stopped")
        self.status_label.setStyleSheet(
            "background-color: #gray; color: white; padding: 10px; "
            "font-weight: bold; border-radius: 5px;"
        )
        
        # Reset telemetry displays
        # TODO: Verify this method exists in telemetry_panel.py
        # self.telemetry_panel.reset()
        self.home_position_set = False
    
    def _connect_mavlink(self):
        """Connect to MAVLink."""
        # Get connection string from ProcessManager
        connection_string = self.process_manager.get_mavlink_connection_string()
        self._append_log(f"Connecting to MAVLink at {connection_string}...")
        
        # Update MAVLinkBridge connection string
        self.mavlink_bridge.connection_string = connection_string
        
        import asyncio
        loop = asyncio.get_event_loop()
        
        async def connect():
            success = await self.mavlink_bridge.connect()
            if success:
                await self.mavlink_bridge.start_telemetry()
                self.telemetry_timer.start()
        
        loop.create_task(connect())
    
    @Slot(bool, str)
    def _on_connection_status(self, connected: bool, message: str):
        """Handle MAVLink connection status."""
        if connected:
            self._append_log(f"MAVLink: {message}")
            self.status_label.setText("Status: Running")
            self.status_label.setStyleSheet(
                "background-color: #51cf66; color: white; padding: 10px; "
                "font-weight: bold; border-radius: 5px;"
            )
        else:
            self._append_log(f"MAVLink ERROR: {message}")
    
    @Slot(str)
    def _on_process_started(self, process_name: str):
        """Handle process started."""
        self._append_log(f"Process started: {process_name}")
    
    @Slot(str)
    def _on_process_stopped(self, process_name: str):
        """Handle process stopped."""
        self._append_log(f"Process stopped: {process_name}")
    
    @Slot(dict)
    def _on_gps_data(self, data: dict):
        """Handle GPS data."""
        # TODO: Verify this method exists in telemetry_panel.py
        # self.telemetry_panel.update_gps(data)
        
        # Set home position on first valid 3D fix
        if not self.home_position_set and data["fix_type"] >= 3:
            home_pos = (data["lat"], data["lon"])
            self.mission_control.set_home_position(home_pos)
            self.home_position_set = True
            self._append_log(f"Home position set: ({data['lat']:.6f}, {data['lon']:.6f})")
    
    @Slot(dict)
    def _on_position_data(self, data: dict):
        """Handle position data."""
        # TODO: Verify this method exists in telemetry_panel.py
        # self.telemetry_panel.update_position(data)
        
        # Update current position on map
        current_pos = (data["lat"], data["lon"])
        self.mission_control.update_current_position(current_pos)
        
        # Track groundspeed for ETA calculation
        self.current_groundspeed = data["groundspeed"]
        self.mission_control.update_groundspeed(self.current_groundspeed)
    
    @Slot(dict)
    def _on_mission_progress(self, progress_data: dict):
        """Handle mission progress updates."""
        self.mission_control.update_mission_progress(progress_data)
        
        # Log waypoint arrivals
        current_wp = progress_data.get("current_waypoint", 0)
        if current_wp > 0:
            distance = progress_data.get("distance_to_waypoint", 0)
            if distance < 15:  # Within acceptance radius
                self._append_log(f"Approaching waypoint {current_wp}")
    
    @Slot()
    def _update_telemetry(self):
        """Update telemetry displays (called at 10 Hz)."""
        # Telemetry updates are now handled by signals
        # This timer ensures GUI updates smoothly
        pass
    
    @Slot(list)
    def upload_mission(self, waypoints: list):
        """Upload mission to autopilot."""
        self._append_log(f"Uploading mission with {len(waypoints)} waypoints...")
        
        import asyncio
        loop = asyncio.get_event_loop()
        
        async def upload():
            success = await self.mavlink_bridge.upload_mission(waypoints)
            if success:
                self._append_log(f"Mission uploaded successfully! ({len(waypoints) + 1} items)")
            else:
                self._append_log("ERROR: Mission upload failed")
        
        loop.create_task(upload())
    
    @Slot(str, dict)
    def send_command(self, command: str, params: dict):
        """Send command to autopilot."""
        self._append_log(f"Sending {command} command...")
        
        import asyncio
        loop = asyncio.get_event_loop()
        
        async def send():
            if command == "TAKEOFF":
                altitude = params.get("altitude", 50)
                self._append_log(f"Requesting takeoff to {altitude}m...")
                self._append_log("Commanding takeoff to 50m...")
                self._append_log("Setting mode to QLOITER...")
            elif command == "AUTO":
                self._append_log("Sending command: AUTO")
                self._append_log("Sending command: AUTO with params: {}")
                self._append_log("Setting mode to AUTO...")
            
            success = await self.mavlink_bridge.send_command(command, params)
            if success:
                self._append_log(f"Command {command} sent successfully")
            else:
                self._append_log(f"ERROR: Command {command} failed")
        
        loop.create_task(send())
    
    @Slot(str)
    def _append_log(self, message: str):
        """Append message to log."""
        from datetime import datetime
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_entry = f"{timestamp} | INFO | {message}"
        
        self.log_text.append(log_entry)
        
        # Auto-scroll to bottom
        cursor = self.log_text.textCursor()
        cursor.movePosition(QTextCursor.End)
        self.log_text.setTextCursor(cursor)
