import logging
import numpy as np
from math import radians, cos, sin, asin, sqrt
from collections import deque
from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                                QPushButton, QListWidget, QSpinBox, QDoubleSpinBox,
                                QGroupBox, QGridLayout, QFileDialog, QMessageBox,
                                QProgressBar)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont
import pyqtgraph as pg

logger = logging.getLogger(__name__)

class Waypoint:
    """Represents a single waypoint"""
    def __init__(self, lat, lon, alt, wp_type="WAYPOINT"):
        self.lat = lat
        self.lon = lon
        self.alt = alt  # Relative altitude in meters
        self.wp_type = wp_type  # WAYPOINT, LOITER_TIME, LAND, etc.
        self.loiter_time = 0  # seconds
        self.acceptance_radius = 5  # meters
    
    def __str__(self):
        return f"{self.wp_type}: ({self.lat:.6f}, {self.lon:.6f}) @ {self.alt}m"

class MissionControlPanel(QWidget):
    """Mission planning and flight control interface"""
    
    # Signals
    mission_upload_requested = Signal(list)  # List of waypoints
    command_requested = Signal(str, dict)  # command_name, params
    
    def __init__(self):
        super().__init__()
        
        self.waypoints = []
        self.current_waypoint_index = -1
        self.home_position = None  # (lat, lon)
        self.mavlink_bridge = None  # Will be set externally
        
        self.init_ui()
        
    def init_ui(self):
        """Setup mission control UI"""
        layout = QVBoxLayout(self)
        
        # Top: Flight controls
        controls = self.create_flight_controls()
        layout.addWidget(controls)
        
        # Middle: Map and waypoint list (side by side)
        middle_layout = QHBoxLayout()
        
        # Map (left, larger)
        map_widget = self.create_map_widget()
        middle_layout.addWidget(map_widget, stretch=3)
        
        # Waypoint list (right, smaller)
        waypoint_widget = self.create_waypoint_widget()
        middle_layout.addWidget(waypoint_widget, stretch=1)
        
        layout.addLayout(middle_layout)
        
        # Bottom: Mission Progress panel (NEW - Phase 2.2)
        progress_panel = self._create_mission_progress_panel()
        layout.addWidget(progress_panel)
        
        logger.info("Mission control panel initialized")
    
    def create_flight_controls(self):
        """Create flight control buttons"""
        group = QGroupBox("Flight Controls")
        layout = QHBoxLayout(group)
        
        self.arm_button = QPushButton("ARM")
        self.arm_button.setStyleSheet("background-color: #ff6b6b; color: white; font-weight: bold; padding: 10px;")
        self.arm_button.clicked.connect(lambda: self.send_command("ARM"))
        layout.addWidget(self.arm_button)
        
        self.disarm_button = QPushButton("DISARM")
        self.disarm_button.setStyleSheet("background-color: #51cf66; color: white; font-weight: bold; padding: 10px;")
        self.disarm_button.clicked.connect(lambda: self.send_command("DISARM"))
        layout.addWidget(self.disarm_button)
        
        self.takeoff_button = QPushButton("TAKEOFF")
        self.takeoff_button.clicked.connect(self.request_takeoff)
        layout.addWidget(self.takeoff_button)
        
        self.rtl_button = QPushButton("RTL (Return Home)")
        self.rtl_button.clicked.connect(lambda: self.send_command("RTL"))
        layout.addWidget(self.rtl_button)
        
        self.land_button = QPushButton("LAND NOW")
        self.land_button.clicked.connect(lambda: self.send_command("LAND"))
        layout.addWidget(self.land_button)
        
        self.auto_button = QPushButton("START MISSION (AUTO)")
        self.auto_button.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold; padding: 10px;")
        self.auto_button.clicked.connect(lambda: self.send_command("AUTO"))
        layout.addWidget(self.auto_button)
        
        return group
    
    def create_map_widget(self):
        """Create interactive map for waypoint planning"""
        group = QGroupBox("Mission Map (Click to Add Waypoint)")
        layout = QVBoxLayout(group)
        
        # PyQtGraph plot widget
        self.map_plot = pg.PlotWidget()
        self.map_plot.setLabel('left', 'Latitude', units='°')
        self.map_plot.setLabel('bottom', 'Longitude', units='°')
        self.map_plot.showGrid(x=True, y=True, alpha=0.3)
        self.map_plot.setAspectLocked(True)
        
        # Home position marker
        self.home_marker = pg.ScatterPlotItem(size=15, brush=pg.mkBrush('g'), symbol='o')
        self.map_plot.addItem(self.home_marker)
        
        # Waypoint markers
        self.waypoint_markers = pg.ScatterPlotItem(size=10, brush=pg.mkBrush('r'), symbol='s')
        self.map_plot.addItem(self.waypoint_markers)
        
        # Mission path line
        self.path_line = self.map_plot.plot(pen=pg.mkPen(color='b', width=2, style=Qt.DashLine))
        
        # Current position marker
        self.current_pos_marker = pg.ScatterPlotItem(size=12, brush=pg.mkBrush('y'), symbol='t')
        self.map_plot.addItem(self.current_pos_marker)
        
        # Click handler
        self.map_plot.scene().sigMouseClicked.connect(self.on_map_click)
        
        layout.addWidget(self.map_plot)
        
        return group
    
    def create_waypoint_widget(self):
        """Create waypoint list and controls"""
        group = QGroupBox("Mission Waypoints")
        layout = QVBoxLayout(group)
        
        # Waypoint list
        self.waypoint_list = QListWidget()
        layout.addWidget(self.waypoint_list)
        
        # Waypoint parameters
        params_layout = QGridLayout()
        
        params_layout.addWidget(QLabel("Altitude (m):"), 0, 0)
        self.alt_spin = QSpinBox()
        self.alt_spin.setRange(0, 500)
        self.alt_spin.setValue(50)
        params_layout.addWidget(self.alt_spin, 0, 1)
        
        params_layout.addWidget(QLabel("Loiter (s):"), 1, 0)
        self.loiter_spin = QSpinBox()
        self.loiter_spin.setRange(0, 300)
        self.loiter_spin.setValue(0)
        params_layout.addWidget(self.loiter_spin, 1, 1)
        
        layout.addLayout(params_layout)
        
        # Buttons
        btn_layout = QHBoxLayout()
        
        delete_btn = QPushButton("Delete Selected")
        delete_btn.clicked.connect(self.delete_waypoint)
        btn_layout.addWidget(delete_btn)
        
        clear_btn = QPushButton("Clear All")
        clear_btn.clicked.connect(self.clear_mission)
        btn_layout.addWidget(clear_btn)
        
        layout.addLayout(btn_layout)
        
        # Mission file buttons
        file_layout = QHBoxLayout()
        
        save_btn = QPushButton("Save Mission")
        save_btn.clicked.connect(self.save_mission)
        file_layout.addWidget(save_btn)
        
        load_btn = QPushButton("Load Mission")
        load_btn.clicked.connect(self.load_mission)
        file_layout.addWidget(load_btn)
        
        layout.addLayout(file_layout)
        
        # Upload button
        upload_btn = QPushButton("UPLOAD TO AUTOPILOT")
        upload_btn.setStyleSheet("background-color: #FF9800; color: white; font-weight: bold; padding: 10px;")
        upload_btn.clicked.connect(self.upload_mission)
        layout.addWidget(upload_btn)
        
        return group
    
    def _create_mission_progress_panel(self):
        """
        Create the mission progress monitoring panel.
        Shows current waypoint, distance, ETA, and overall progress.
        Phase 2.2 feature.
        """
        self.progress_group = QGroupBox("Mission Progress")
        layout = QVBoxLayout()
        
        # Status line - which waypoint we're flying to
        self.progress_status = QLabel("No mission active")
        font = QFont("Arial", 11)
        font.setBold(True)
        self.progress_status.setFont(font)
        self.progress_status.setStyleSheet("color: #333333;")
        layout.addWidget(self.progress_status)
        
        # Add some spacing
        layout.addSpacing(10)
        
        # Metrics row - distance and ETA side by side
        metrics_layout = QHBoxLayout()
        
        self.distance_label = QLabel("Distance: --")
        self.distance_label.setFont(QFont("Arial", 12))
        self.distance_label.setStyleSheet("color: #0066CC;")
        metrics_layout.addWidget(self.distance_label)
        
        metrics_layout.addStretch()
        
        self.eta_label = QLabel("ETA: --")
        self.eta_label.setFont(QFont("Arial", 12))
        self.eta_label.setStyleSheet("color: #009900;")
        metrics_layout.addWidget(self.eta_label)
        
        layout.addLayout(metrics_layout)
        
        # Add spacing
        layout.addSpacing(10)
        
        # Progress bar with label
        progress_label = QLabel("Mission Progress")
        layout.addWidget(progress_label)
        
        self.mission_progress_bar = QProgressBar()
        self.mission_progress_bar.setMinimum(0)
        self.mission_progress_bar.setMaximum(100)
        self.mission_progress_bar.setValue(0)
        self.mission_progress_bar.setTextVisible(True)
        self.mission_progress_bar.setFormat("%p%")
        self.mission_progress_bar.setFixedHeight(25)
        
        # Style the progress bar
        self.mission_progress_bar.setStyleSheet("""
            QProgressBar {
                border: 1px solid #BDBDBD;
                border-radius: 3px;
                background-color: #E0E0E0;
                text-align: center;
            }
            QProgressBar::chunk {
                background-color: #4CAF50;
            }
        """)
        
        layout.addWidget(self.mission_progress_bar)
        
        # Add spacing
        layout.addSpacing(10)
        
        # Next waypoint info
        self.next_wp_label = QLabel("Upload a mission to begin")
        font_italic = QFont("Arial", 10)
        font_italic.setItalic(True)
        self.next_wp_label.setFont(font_italic)
        self.next_wp_label.setStyleSheet("color: #666666;")
        layout.addWidget(self.next_wp_label)
        
        self.progress_group.setLayout(layout)
        return self.progress_group
    
    def update_mission_progress(self, telemetry_data: dict):
        """
        Update mission progress display from telemetry.
        Phase 2.2 feature.
        
        Args:
            telemetry_data: Dictionary with keys:
                - current_position: (lat, lon, alt) tuple
                - current_seq: Current waypoint sequence
                - ground_speed: Speed in m/s
                - mode: Current flight mode
        """
        # ULTRA DEBUG
        received_speed = telemetry_data.get('ground_speed')
        print(f"!!! MISSION_CONTROL_PANEL: Received ground_speed = {received_speed} m/s !!!")
        
        # Check if mission exists
        if not self.waypoints or len(self.waypoints) == 0:
            self._clear_mission_progress()
            return
        
        # Extract data
        current_pos = telemetry_data.get('current_position')
        current_seq = telemetry_data.get('current_seq')
        ground_speed = telemetry_data.get('ground_speed')
        mode = telemetry_data.get('mode', 'UNKNOWN')
        
        # Debug logging for ground speed
        logger.info(f"Mission progress update - Speed: {ground_speed} m/s, Seq: {current_seq}, Mode: {mode}")
        
        # Handle None ground_speed explicitly
        if ground_speed is None:
            ground_speed = 0
            logger.warning("Ground speed is None, defaulting to 0")
        
        # Validate data
        if current_pos is None:
            return
        
        # Default current_seq to 0 if not provided (hasn't started yet)
        if current_seq is None:
            if mode == 'AUTO':
                current_seq = 0
            else:
                self._clear_mission_progress()
                return
        
        current_lat, current_lon, current_alt = current_pos
        total_waypoints = len(self.waypoints)
        
        # Update status based on mission state
        if current_seq < total_waypoints:
            self.progress_status.setText(f"Flying to Waypoint {current_seq + 1} of {total_waypoints}")
            
            # Get target waypoint
            target_wp = self.waypoints[current_seq]
            
            # Calculate distance using haversine formula
            distance_m = self._haversine(current_lat, current_lon, target_wp.lat, target_wp.lon)
            
            # Format distance display
            if distance_m > 1000:
                distance_str = f"{distance_m / 1000:.1f} km"
            else:
                distance_str = f"{int(distance_m)} m"
            self.distance_label.setText(f"Distance: {distance_str}")
            
            # Calculate and format ETA
            eta_seconds = self._calculate_eta(distance_m, ground_speed)
            eta_str = self._format_eta(eta_seconds)
            self.eta_label.setText(f"ETA: {eta_str}")
            
            # Update progress bar (waypoints completed / total)
            progress_pct = (current_seq / total_waypoints) * 100
            self.mission_progress_bar.setValue(int(progress_pct))
            
            # Update next waypoint info
            self.next_wp_label.setText(
                f"Next: WP{current_seq + 1} @ ({target_wp.lat:.4f}, {target_wp.lon:.4f}, {target_wp.alt:.0f}m)"
            )
        else:
            # Mission complete
            self.progress_status.setText(f"Mission Complete ({total_waypoints} waypoints)")
            self.distance_label.setText("Distance: 0 m")
            self.eta_label.setText("ETA: Complete")
            self.mission_progress_bar.setValue(100)
            self.next_wp_label.setText("All waypoints reached")
    
    def _clear_mission_progress(self):
        """Reset progress display when no mission active. Phase 2.2."""
        self.progress_status.setText("No mission active")
        self.distance_label.setText("Distance: --")
        self.eta_label.setText("ETA: --")
        self.mission_progress_bar.setValue(0)
        self.next_wp_label.setText("Upload a mission to begin")
    
    def _haversine(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """
        Calculate great circle distance between two points on Earth.
        Phase 2.2 calculation.
        
        Args:
            lat1, lon1: Current position (decimal degrees)
            lat2, lon2: Target position (decimal degrees)
            
        Returns:
            Distance in meters
        """
        R = 6371000  # Earth radius in meters
        
        # Convert to radians
        lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
        
        # Haversine formula
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
        c = 2 * asin(sqrt(a))
        
        return R * c
    
    def _calculate_eta(self, distance_m: float, ground_speed_ms: float) -> float:
        """
        Calculate estimated time to reach target.
        Phase 2.2 calculation.
        
        Args:
            distance_m: Distance to target in meters
            ground_speed_ms: Current ground speed in m/s
            
        Returns:
            ETA in seconds, or -1 if speed too low
        """
        MIN_SPEED = 1.0  # m/s - below this, ETA unreliable
        
        if ground_speed_ms < MIN_SPEED:
            return -1  # Signal that ETA is not available
        
        eta_seconds = distance_m / ground_speed_ms
        return eta_seconds
    
    def _format_eta(self, eta_seconds: float) -> str:
        """
        Format ETA for human-readable display.
        Phase 2.2 helper.
        
        Args:
            eta_seconds: Estimated time in seconds (or -1 if unavailable)
            
        Returns:
            Formatted string like "45s", "2.5min", or "1.2hr"
        """
        if eta_seconds < 0:
            return "--"
        
        if eta_seconds < 60:
            # Less than 60 seconds - show as seconds
            return f"{int(eta_seconds)}s"
        elif eta_seconds < 3600:
            # Less than 60 minutes - show as minutes
            minutes = eta_seconds / 60
            return f"{minutes:.1f}min"
        else:
            # 60+ minutes - show as hours
            hours = eta_seconds / 3600
            return f"{hours:.1f}hr"
    
    def set_mavlink_bridge(self, bridge):
        """
        Connect to MAVLink bridge for telemetry updates.
        Phase 2.2 integration.
        
        Args:
            bridge: MAVLinkBridge instance
        """
        self.mavlink_bridge = bridge
        
        # Connect telemetry signal to update method
        if hasattr(bridge, 'telemetry_updated'):
            bridge.telemetry_updated.connect(self._on_telemetry_updated)
            logger.info("Mission progress connected to telemetry stream")
    
    def _on_telemetry_updated(self):
        """
        Process telemetry and update mission progress.
        Phase 2.2 signal handler.
        """
        if not self.mavlink_bridge:
            return
        
        # Gather required data from mavlink_bridge
        telemetry_data = {
            'current_position': getattr(self.mavlink_bridge, 'current_position', None),
            'current_seq': getattr(self.mavlink_bridge, 'current_seq', None),
            'ground_speed': getattr(self.mavlink_bridge, 'ground_speed', 0),
            'mode': getattr(self.mavlink_bridge, 'mode', 'UNKNOWN')
        }
        
        self.update_mission_progress(telemetry_data)
    
    def on_map_click(self, event):
        """Handle map click to add waypoint"""
        pos = event.scenePos()
        if self.map_plot.sceneBoundingRect().contains(pos):
            mouse_point = self.map_plot.getViewBox().mapSceneToView(pos)
            lon = mouse_point.x()
            lat = mouse_point.y()
            
            # Add waypoint
            alt = self.alt_spin.value()
            wp = Waypoint(lat, lon, alt)
            wp.loiter_time = self.loiter_spin.value()
            
            self.waypoints.append(wp)
            self.update_waypoint_display()
            
            logger.info(f"Added waypoint: {wp}")
    
    def update_waypoint_display(self):
        """Update map and list with current waypoints"""
        # Update list
        self.waypoint_list.clear()
        for i, wp in enumerate(self.waypoints):
            self.waypoint_list.addItem(f"{i+1}. {wp}")
        
        # Update map
        if len(self.waypoints) > 0:
            lons = [wp.lon for wp in self.waypoints]
            lats = [wp.lat for wp in self.waypoints]
            
            self.waypoint_markers.setData(lons, lats)
            
            # Draw path
            if self.home_position:
                path_lons = [self.home_position[1]] + lons
                path_lats = [self.home_position[0]] + lats
            else:
                path_lons = lons
                path_lats = lats
            
            self.path_line.setData(path_lons, path_lats)
        else:
            self.waypoint_markers.setData([], [])
            self.path_line.setData([], [])
    
    def delete_waypoint(self):
        """Delete selected waypoint"""
        current_row = self.waypoint_list.currentRow()
        if current_row >= 0 and current_row < len(self.waypoints):
            del self.waypoints[current_row]
            self.update_waypoint_display()
            logger.info(f"Deleted waypoint {current_row + 1}")
    
    def clear_mission(self):
        """Clear all waypoints"""
        reply = QMessageBox.question(self, 'Clear Mission', 
                                     'Delete all waypoints?',
                                     QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.waypoints.clear()
            self.update_waypoint_display()
            logger.info("Mission cleared")
    
    def set_home_position(self, lat, lon):
        """Set home/current position"""
        self.home_position = (lat, lon)
        self.home_marker.setData([lon], [lat])
        
        # Auto-center map on home
        if lat != 0 and lon != 0:
            self.map_plot.setXRange(lon - 0.01, lon + 0.01)
            self.map_plot.setYRange(lat - 0.01, lat + 0.01)
    
    def update_current_position(self, lat, lon):
        """Update aircraft current position marker"""
        if lat != 0 and lon != 0:
            self.current_pos_marker.setData([lon], [lat])
    
    def upload_mission(self):
        """Request mission upload to autopilot"""
        if len(self.waypoints) == 0:
            QMessageBox.warning(self, "No Waypoints", "Add waypoints before uploading mission.")
            return
        
        logger.info(f"Uploading mission with {len(self.waypoints)} waypoints...")
        self.mission_upload_requested.emit(self.waypoints)
    
    def send_command(self, command):
        """Send flight command"""
        logger.info(f"Sending command: {command}")
        self.command_requested.emit(command, {})
    
    def request_takeoff(self):
        """Request takeoff with altitude"""
        alt = self.alt_spin.value()
        logger.info(f"Requesting takeoff to {alt}m")
        self.command_requested.emit("TAKEOFF", {"altitude": alt})
    
    def save_mission(self):
        """Save mission to file"""
        if len(self.waypoints) == 0:
            QMessageBox.warning(self, "No Mission", "No waypoints to save.")
            return
        
        filename, _ = QFileDialog.getSaveFileName(self, "Save Mission", "", "Waypoint Files (*.waypoints)")
        if filename:
            try:
                with open(filename, 'w') as f:
                    f.write("QGC WPL 110\n")  # ArduPilot format header
                    for i, wp in enumerate(self.waypoints):
                        f.write(f"{i+1}\t0\t3\t16\t0\t0\t0\t0\t{wp.lat}\t{wp.lon}\t{wp.alt}\t1\n")
                logger.info(f"Mission saved to {filename}")
                QMessageBox.information(self, "Success", f"Mission saved to {filename}")
            except Exception as e:
                logger.error(f"Failed to save mission: {e}")
                QMessageBox.critical(self, "Error", f"Failed to save: {e}")
    
    def load_mission(self):
        """Load mission from file"""
        filename, _ = QFileDialog.getOpenFileName(self, "Load Mission", "", "Waypoint Files (*.waypoints)")
        if filename:
            try:
                self.waypoints.clear()
                with open(filename, 'r') as f:
                    lines = f.readlines()
                    for line in lines[1:]:  # Skip header
                        parts = line.strip().split('\t')
                        if len(parts) >= 11:
                            lat = float(parts[8])
                            lon = float(parts[9])
                            alt = float(parts[10])
                            self.waypoints.append(Waypoint(lat, lon, alt))
                
                self.update_waypoint_display()
                logger.info(f"Loaded {len(self.waypoints)} waypoints from {filename}")
                QMessageBox.information(self, "Success", f"Loaded {len(self.waypoints)} waypoints")
            except Exception as e:
                logger.error(f"Failed to load mission: {e}")
                QMessageBox.critical(self, "Error", f"Failed to load: {e}")
