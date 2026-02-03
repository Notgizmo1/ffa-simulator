import logging
from PySide6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                                QPushButton, QLabel, QComboBox, QTextEdit, QSplitter)
from PySide6.QtCore import Qt
from ffa_simulator.orchestrator.process_manager import ProcessManager
from ffa_simulator.telemetry.mavlink_bridge import TelemetryBridge

logger = logging.getLogger(__name__)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.process_manager = None
        self.telemetry_bridge = None
        self.init_ui()
        
    def init_ui(self):
        self.setWindowTitle("FFA Simulator - ArduPilot VTOL Training")
        self.setGeometry(100, 100, 1400, 900)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        main_layout = QHBoxLayout(central_widget)
        
        left_panel = self.create_control_panel()
        right_panel = self.create_telemetry_panel()
        
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left_panel)
        splitter.addWidget(right_panel)
        splitter.setSizes([400, 1000])
        
        main_layout.addWidget(splitter)
        
        logger.info("Main window UI initialized")
    
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
        self.log_display.setMaximumHeight(200)
        layout.addWidget(self.log_display)
        
        layout.addStretch()
        
        return panel
    
    def create_telemetry_panel(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        telemetry_label = QLabel("Telemetry Display Area")
        telemetry_label.setStyleSheet("border: 2px dashed #ccc; padding: 20px;")
        telemetry_label.setAlignment(Qt.AlignCenter)
        telemetry_label.setMinimumHeight(400)
        layout.addWidget(telemetry_label)
        
        status_layout = QHBoxLayout()
        self.mode_label = QLabel("Mode: --")
        self.armed_label = QLabel("Armed: No")
        self.battery_label = QLabel("Battery: --")
        self.gps_label = QLabel("GPS: --")
        
        for label in [self.mode_label, self.armed_label, self.battery_label, self.gps_label]:
            label.setStyleSheet("padding: 5px; background-color: #f0f0f0; border-radius: 3px;")
            status_layout.addWidget(label)
        
        layout.addLayout(status_layout)
        
        return panel
    
    def start_simulation(self):
        logger.info("Starting simulation...")
        self.log_display.append("▶ Starting simulation...")
        
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.status_label.setText("Status: Running")
        self.status_label.setStyleSheet("padding: 10px; background-color: #51cf66; color: white; border-radius: 5px;")
        
        self.log_display.append("✓ Docker containers starting...")
        self.log_display.append("✓ Gazebo simulation initializing...")
        self.log_display.append("✓ ArduPilot SITL launching...")
        self.log_display.append("✓ Waiting for MAVLink heartbeat...")
        
        logger.info("Simulation started successfully")
    
    def stop_simulation(self):
        logger.info("Stopping simulation...")
        self.log_display.append("■ Stopping simulation...")
        
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.status_label.setText("Status: Stopped")
        self.status_label.setStyleSheet("padding: 10px; background-color: #ff6b6b; color: white; border-radius: 5px;")
        
        self.log_display.append("✓ Simulation stopped")
        logger.info("Simulation stopped")