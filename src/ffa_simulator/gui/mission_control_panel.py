import logging
import time
import numpy as np
from collections import deque
from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                                QPushButton, QListWidget, QSpinBox, QDoubleSpinBox,
                                QGroupBox, QGridLayout, QFileDialog, QMessageBox,
                                QProgressBar)
from PySide6.QtCore import Qt, Signal
import pyqtgraph as pg

from ffa_simulator.telemetry.mission_calculator import MissionCalculator

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
        self.flight_start_time = None  # Timestamp when mission started
        
        self.init_ui()
        
    def init_ui(self):
        """Setup mission control UI"""
        layout = QVBoxLayout(self)
        
        # Top: Flight controls
        controls = self.create_flight_controls()
        layout.addWidget(controls)
        
        # Altitude Controls (Phase 3.1)
        altitude_controls = self.create_altitude_controls()
        layout.addWidget(altitude_controls)
        
        # Mission Progress (Phase 3.2)
        mission_progress = self.create_mission_progress_widget()
        layout.addWidget(mission_progress)
        
        # Middle: Map and waypoint list (side by side)
        middle_layout = QHBoxLayout()
        
        # Map (left, larger)
        map_widget = self.create_map_widget()
        middle_layout.addWidget(map_widget, stretch=3)
        
        # Waypoint list (right, smaller)
        waypoint_widget = self.create_waypoint_widget()
        middle_layout.addWidget(waypoint_widget, stretch=1)
        
        layout.addLayout(middle_layout)
        
        logger.info("Mission control panel initialized")
    
    def create_flight_controls(self):
        """Create flight control buttons"""
        group = QGroupBox("Flight Controls")
        layout = QHBoxLayout(group)
        
        self.arm_button = QPushButton("ARM")
        self.arm_button.setStyleSheet("background-color: #ff6b6b; color: white; font-weight: bold; padding: 10px;")
        self.arm_button.clicked.connect(lambda: self.send_command("ARM"))
        layout.addWidget(self.arm_button)
        
        # Force ARM button for SITL (bypasses PreArm checks)
        self.force_arm_button = QPushButton("FORCE ARM (SITL)")
        self.force_arm_button.setStyleSheet("background-color: #dc3545; color: white; font-weight: bold; padding: 10px; border: 2px solid #a71d2a;")
        self.force_arm_button.clicked.connect(lambda: self.send_command("ARM_FORCE"))
        layout.addWidget(self.force_arm_button)
        
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
    
    def create_altitude_controls(self):
        """Create altitude hold and loiter control panel (Phase 3.1)"""
        group = QGroupBox("Altitude & Loiter Controls")
        main_layout = QVBoxLayout(group)
        
        # Top row: Current status displays
        status_layout = QHBoxLayout()
        
        # Current Altitude
        self.current_alt_label = QLabel("Current Altitude: 0.0m")
        self.current_alt_label.setStyleSheet("font-weight: bold; font-size: 12pt; padding: 5px;")
        status_layout.addWidget(self.current_alt_label)
        
        # Current Position
        self.current_pos_label = QLabel("Current Position: (0.000000, 0.000000)")
        self.current_pos_label.setStyleSheet("font-weight: bold; font-size: 12pt; padding: 5px;")
        status_layout.addWidget(self.current_pos_label)
        
        main_layout.addLayout(status_layout)
        
        # Middle row: Altitude adjustment buttons
        adj_layout = QHBoxLayout()
        
        # Target altitude spinbox
        adj_layout.addWidget(QLabel("Target Altitude (m):"))
        self.target_alt_spin = QSpinBox()
        self.target_alt_spin.setRange(0, 500)
        self.target_alt_spin.setValue(150)
        self.target_alt_spin.setStyleSheet("font-size: 11pt; padding: 5px;")
        adj_layout.addWidget(self.target_alt_spin)
        
        # Quick adjustment buttons
        adj_layout.addWidget(QLabel("  Quick Adjust:"))
        
        plus_10_btn = QPushButton("+10m")
        plus_10_btn.clicked.connect(lambda: self.adjust_altitude(10))
        plus_10_btn.setStyleSheet("background-color: #228B22; color: white; font-weight: bold;")
        adj_layout.addWidget(plus_10_btn)
        
        plus_5_btn = QPushButton("+5m")
        plus_5_btn.clicked.connect(lambda: self.adjust_altitude(5))
        plus_5_btn.setStyleSheet("background-color: #32CD32; color: white; font-weight: bold;")
        adj_layout.addWidget(plus_5_btn)
        
        plus_1_btn = QPushButton("+1m")
        plus_1_btn.clicked.connect(lambda: self.adjust_altitude(1))
        plus_1_btn.setStyleSheet("background-color: #90EE90; color: black; font-weight: bold;")
        adj_layout.addWidget(plus_1_btn)
        
        minus_1_btn = QPushButton("-1m")
        minus_1_btn.clicked.connect(lambda: self.adjust_altitude(-1))
        minus_1_btn.setStyleSheet("background-color: #FFA07A; color: black; font-weight: bold;")
        adj_layout.addWidget(minus_1_btn)
        
        minus_5_btn = QPushButton("-5m")
        minus_5_btn.clicked.connect(lambda: self.adjust_altitude(-5))
        minus_5_btn.setStyleSheet("background-color: #FF6347; color: white; font-weight: bold;")
        adj_layout.addWidget(minus_5_btn)
        
        minus_10_btn = QPushButton("-10m")
        minus_10_btn.clicked.connect(lambda: self.adjust_altitude(-10))
        minus_10_btn.setStyleSheet("background-color: #DC143C; color: white; font-weight: bold;")
        adj_layout.addWidget(minus_10_btn)
        
        main_layout.addLayout(adj_layout)
        
        # Bottom row: Mode and loiter controls
        mode_layout = QHBoxLayout()
        
        # Mode buttons
        qloiter_btn = QPushButton("QLOITER")
        qloiter_btn.clicked.connect(lambda: self.send_mode_command("QLOITER"))
        qloiter_btn.setStyleSheet("background-color: #4A90E2; color: white; font-weight: bold; padding: 8px;")
        mode_layout.addWidget(qloiter_btn)
        
        guided_btn = QPushButton("GUIDED")
        guided_btn.clicked.connect(lambda: self.send_mode_command("GUIDED"))
        guided_btn.setStyleSheet("background-color: #7B68EE; color: white; font-weight: bold; padding: 8px;")
        mode_layout.addWidget(guided_btn)
        
        # Hold altitude button
        hold_alt_btn = QPushButton("HOLD CURRENT ALTITUDE")
        hold_alt_btn.clicked.connect(self.hold_current_altitude)
        hold_alt_btn.setStyleSheet("background-color: #FF8C00; color: white; font-weight: bold; padding: 8px;")
        mode_layout.addWidget(hold_alt_btn)
        
        # Loiter here button
        loiter_here_btn = QPushButton("LOITER HERE")
        loiter_here_btn.clicked.connect(self.loiter_at_current_position)
        loiter_here_btn.setStyleSheet("background-color: #9370DB; color: white; font-weight: bold; padding: 8px;")
        mode_layout.addWidget(loiter_here_btn)
        
        # Loiter radius controls
        mode_layout.addWidget(QLabel("  Loiter Radius (m):"))
        self.loiter_radius_spin = QSpinBox()
        self.loiter_radius_spin.setRange(10, 500)
        self.loiter_radius_spin.setValue(50)
        self.loiter_radius_spin.setStyleSheet("font-size: 11pt; padding: 5px;")
        mode_layout.addWidget(self.loiter_radius_spin)
        
        set_radius_btn = QPushButton("SET RADIUS")
        set_radius_btn.clicked.connect(self.set_loiter_radius)
        set_radius_btn.setStyleSheet("background-color: #20B2AA; color: white; font-weight: bold; padding: 8px;")
        mode_layout.addWidget(set_radius_btn)
        
        main_layout.addLayout(mode_layout)
        
        # Store current altitude for tracking
        self.current_altitude = 0.0
        
        return group
    
    def create_mission_progress_widget(self):
        """Create mission progress tracking panel (Phase 3.2)"""
        group = QGroupBox("Mission Progress")
        main_layout = QHBoxLayout(group)
        
        # Left column: Waypoint info
        left_layout = QGridLayout()
        
        # Current waypoint label
        left_layout.addWidget(QLabel("Current WP:"), 0, 0)
        self.progress_current_wp = QLabel("--- / ---")
        self.progress_current_wp.setStyleSheet("font-weight: bold; font-size: 12pt; color: #2196F3;")
        left_layout.addWidget(self.progress_current_wp, 0, 1)
        
        # Distance to next waypoint
        left_layout.addWidget(QLabel("Dist to WP:"), 1, 0)
        self.progress_dist_to_wp = QLabel("---")
        self.progress_dist_to_wp.setStyleSheet("font-weight: bold; font-size: 11pt;")
        left_layout.addWidget(self.progress_dist_to_wp, 1, 1)
        
        # Bearing to next waypoint
        left_layout.addWidget(QLabel("Bearing:"), 2, 0)
        self.progress_bearing = QLabel("---")
        self.progress_bearing.setStyleSheet("font-weight: bold; font-size: 11pt;")
        left_layout.addWidget(self.progress_bearing, 2, 1)
        
        main_layout.addLayout(left_layout)
        
        # Center column: ETA info
        center_layout = QGridLayout()
        
        # ETA to next waypoint
        center_layout.addWidget(QLabel("ETA to WP:"), 0, 0)
        self.progress_eta_wp = QLabel("---")
        self.progress_eta_wp.setStyleSheet("font-weight: bold; font-size: 11pt;")
        center_layout.addWidget(self.progress_eta_wp, 0, 1)
        
        # Total remaining distance
        center_layout.addWidget(QLabel("Remaining:"), 1, 0)
        self.progress_remaining_dist = QLabel("---")
        self.progress_remaining_dist.setStyleSheet("font-weight: bold; font-size: 11pt;")
        center_layout.addWidget(self.progress_remaining_dist, 1, 1)
        
        # Total ETA
        center_layout.addWidget(QLabel("Total ETA:"), 2, 0)
        self.progress_total_eta = QLabel("---")
        self.progress_total_eta.setStyleSheet("font-weight: bold; font-size: 11pt; color: #FF8C00;")
        center_layout.addWidget(self.progress_total_eta, 2, 1)
        
        main_layout.addLayout(center_layout)
        
        # Right column: Progress bar and status
        right_layout = QVBoxLayout()
        
        # Mission status label
        self.progress_status = QLabel("NO MISSION")
        self.progress_status.setStyleSheet(
            "font-weight: bold; font-size: 11pt; padding: 4px; "
            "background-color: #555; color: white; border-radius: 3px;"
        )
        self.progress_status.setAlignment(Qt.AlignCenter)
        right_layout.addWidget(self.progress_status)
        
        # Progress bar
        self.mission_progress_bar = QProgressBar()
        self.mission_progress_bar.setRange(0, 100)
        self.mission_progress_bar.setValue(0)
        self.mission_progress_bar.setTextVisible(True)
        self.mission_progress_bar.setFormat("%v%")
        self.mission_progress_bar.setStyleSheet("""
            QProgressBar {
                border: 2px solid #555;
                border-radius: 5px;
                text-align: center;
                font-weight: bold;
                font-size: 11pt;
                min-height: 24px;
            }
            QProgressBar::chunk {
                background-color: #2196F3;
                border-radius: 3px;
            }
        """)
        right_layout.addWidget(self.mission_progress_bar)
        
        # Groundspeed display
        self.progress_groundspeed = QLabel("GS: --- m/s")
        self.progress_groundspeed.setStyleSheet("font-weight: bold; font-size: 10pt; color: #888;")
        self.progress_groundspeed.setAlignment(Qt.AlignCenter)
        right_layout.addWidget(self.progress_groundspeed)
        
        # Flight timer
        self.flight_timer_label = QLabel("Flight Time: 00:00:00")
        self.flight_timer_label.setStyleSheet("font-weight: bold; font-size: 10pt; color: #51cf66;")
        self.flight_timer_label.setAlignment(Qt.AlignCenter)
        right_layout.addWidget(self.flight_timer_label)
        
        main_layout.addLayout(right_layout)
        
        return group
    
    def update_mission_progress(self, lat, lon, groundspeed, current_seq, total_wps, waypoints, mission_active, mission_complete=False):
        """Update mission progress display with current telemetry."""
        # Update groundspeed display always
        self.progress_groundspeed.setText(f"GS: {groundspeed:.1f} m/s")
        
        # No mission loaded
        if total_wps == 0 or not waypoints:
            self.progress_status.setText("NO MISSION")
            self.progress_status.setStyleSheet(
                "font-weight: bold; font-size: 11pt; padding: 4px; "
                "background-color: #555; color: white; border-radius: 3px;"
            )
            self.progress_current_wp.setText("--- / ---")
            self.progress_dist_to_wp.setText("---")
            self.progress_bearing.setText("---")
            self.progress_eta_wp.setText("---")
            self.progress_remaining_dist.setText("---")
            self.progress_total_eta.setText("---")
            self.mission_progress_bar.setValue(0)
            return
        
        # Mission complete — either from MISSION_ITEM_REACHED or seq past last WP
        # current_seq from ArduPilot: seq 0 = home, seq 1 = NAV_TAKEOFF, seq 2+ = user WPs
        user_wp_index = current_seq - 2
        
        if mission_complete or user_wp_index >= total_wps:
            self.progress_status.setText("MISSION COMPLETE — RTL")
            self.progress_status.setStyleSheet(
                "font-weight: bold; font-size: 11pt; padding: 4px; "
                "background-color: #51cf66; color: white; border-radius: 3px;"
            )
            self.progress_current_wp.setText(f"{total_wps} / {total_wps}")
            self.progress_dist_to_wp.setText("0m")
            self.progress_bearing.setText("---")
            self.progress_eta_wp.setText("Done")
            self.progress_remaining_dist.setText("0m")
            self.progress_total_eta.setText("Done")
            self.mission_progress_bar.setValue(100)
            return
        
        # Clamp to valid range
        if user_wp_index < 0:
            user_wp_index = 0
        
        # Update status
        if mission_active:
            self.progress_status.setText("AUTO - IN PROGRESS")
            self.progress_status.setStyleSheet(
                "font-weight: bold; font-size: 11pt; padding: 4px; "
                "background-color: #2196F3; color: white; border-radius: 3px;"
            )
            # Start flight timer on first active update
            if self.flight_start_time is None:
                self.flight_start_time = time.time()
                logger.info("Flight timer started")
        else:
            self.progress_status.setText("MISSION LOADED")
            self.progress_status.setStyleSheet(
                "font-weight: bold; font-size: 11pt; padding: 4px; "
                "background-color: #FF8C00; color: white; border-radius: 3px;"
            )
        
        # Update flight timer
        if self.flight_start_time is not None:
            elapsed = time.time() - self.flight_start_time
            hours = int(elapsed // 3600)
            minutes = int((elapsed % 3600) // 60)
            seconds = int(elapsed % 60)
            self.flight_timer_label.setText(f"Flight Time: {hours:02d}:{minutes:02d}:{seconds:02d}")
            self.flight_timer_label.setStyleSheet("font-weight: bold; font-size: 10pt; color: #51cf66;")
        
        # Current waypoint display (1-indexed for user)
        self.progress_current_wp.setText(f"{user_wp_index + 1} / {total_wps}")
        
        # Distance to next waypoint
        if lat != 0 and lon != 0:
            target_wp = waypoints[user_wp_index]
            dist_to_wp = MissionCalculator.haversine_distance(
                lat, lon, target_wp.lat, target_wp.lon
            )
            self.progress_dist_to_wp.setText(MissionCalculator.format_distance(dist_to_wp))
            
            # Bearing to next waypoint
            bearing = MissionCalculator.calculate_bearing(
                lat, lon, target_wp.lat, target_wp.lon
            )
            cardinal = MissionCalculator.bearing_to_cardinal(bearing)
            self.progress_bearing.setText(f"{bearing:.0f}° ({cardinal})")
            
            # ETA to next waypoint
            eta_wp = MissionCalculator.calculate_eta(dist_to_wp, groundspeed)
            self.progress_eta_wp.setText(MissionCalculator.format_time(eta_wp))
            
            # Total remaining distance
            remaining = MissionCalculator.calculate_remaining_distance(
                lat, lon, user_wp_index, waypoints
            )
            self.progress_remaining_dist.setText(MissionCalculator.format_distance(remaining))
            
            # Total remaining ETA
            total_eta = MissionCalculator.calculate_eta(remaining, groundspeed)
            self.progress_total_eta.setText(MissionCalculator.format_time(total_eta))
            
            # Progress bar: percentage of waypoints completed
            # Factor in distance to current WP for smoother progress
            total_dist = MissionCalculator.calculate_total_mission_distance(waypoints)
            if total_dist > 0:
                completed_dist = total_dist - remaining
                progress_pct = max(0, min(100, int((completed_dist / total_dist) * 100)))
            else:
                # Fallback to waypoint-count-based progress
                progress_pct = int((user_wp_index / total_wps) * 100)
            
            self.mission_progress_bar.setValue(progress_pct)
        else:
            self.progress_dist_to_wp.setText("No GPS")
            self.progress_bearing.setText("---")
            self.progress_eta_wp.setText("---")
            self.progress_remaining_dist.setText("---")
            self.progress_total_eta.setText("---")
    
    def adjust_altitude(self, delta):
        """Adjust target altitude by delta and send command"""
        current = self.target_alt_spin.value()
        new_alt = current + delta
        
        # Safety checks
        if new_alt < 0:
            QMessageBox.warning(self, "Invalid Altitude", "Altitude cannot be negative.")
            return
        
        if new_alt > 400:
            reply = QMessageBox.question(self, 'High Altitude Warning', 
                                         f'Target altitude {new_alt}m exceeds 400m. Continue?',
                                         QMessageBox.Yes | QMessageBox.No)
            if reply != QMessageBox.Yes:
                return
        
        if new_alt < 100:
            QMessageBox.information(self, 'Low Altitude Notice', 
                                   f'Target altitude {new_alt}m is below 100m. Ensure sufficient clearance.')
        
        # Check for large changes
        if abs(delta) > 50 or abs(new_alt - self.current_altitude) > 50:
            reply = QMessageBox.question(self, 'Large Altitude Change', 
                                         f'Change altitude by {abs(new_alt - self.current_altitude):.1f}m (current: {self.current_altitude:.1f}m → target: {new_alt}m)?',
                                         QMessageBox.Yes | QMessageBox.No)
            if reply != QMessageBox.Yes:
                return
        
        self.target_alt_spin.setValue(new_alt)
        self.send_altitude_command(new_alt)
    
    def send_altitude_command(self, altitude):
        """Send altitude change command to autopilot"""
        logger.info(f"Setting target altitude: {altitude}m (current: {self.current_altitude}m)")
        self.command_requested.emit("CHANGE_ALTITUDE", {"altitude": altitude})
    
    def hold_current_altitude(self):
        """Hold at current altitude"""
        if self.current_altitude > 0:
            logger.info(f"Holding current altitude: {self.current_altitude}m")
            self.target_alt_spin.setValue(int(self.current_altitude))
            self.send_altitude_command(self.current_altitude)
        else:
            QMessageBox.warning(self, "No Altitude Data", "Current altitude is 0m. Aircraft may not be airborne.")
    
    def loiter_at_current_position(self):
        """Command aircraft to loiter at current position"""
        if self.current_altitude < 50:
            QMessageBox.warning(self, "Altitude Too Low", 
                              "Cannot loiter below 50m altitude. Climb first.")
            return
        
        logger.info(f"Commanding loiter at current position (alt: {self.current_altitude}m)")
        self.command_requested.emit("LOITER_HERE", {})
    
    def set_loiter_radius(self):
        """Set loiter circle radius"""
        radius = self.loiter_radius_spin.value()
        logger.info(f"Setting loiter radius: {radius}m")
        self.command_requested.emit("SET_LOITER_RADIUS", {"radius": radius})
    
    def send_mode_command(self, mode):
        """Send mode change command"""
        logger.info(f"Requesting mode change to: {mode}")
        self.command_requested.emit("SET_MODE", {"mode": mode})
    
    def update_altitude_display(self, altitude):
        """Update current altitude display"""
        self.current_altitude = altitude
        display_alt = max(0.0, altitude)  # Clamp to 0 on ground
        self.current_alt_label.setText(f"Current Altitude: {display_alt:.1f}m")
    
    def update_position_display(self, lat, lon):
        """Update current position display"""
        self.current_pos_label.setText(f"Current Position: ({lat:.6f}, {lon:.6f})")
    
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
        
        # Active waypoint marker (Phase 3.2 - highlights target WP)
        self.active_wp_marker = pg.ScatterPlotItem(
            size=18, brush=pg.mkBrush(0, 255, 0, 120), 
            pen=pg.mkPen(color='lime', width=2), symbol='o'
        )
        self.map_plot.addItem(self.active_wp_marker)
        
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
    
    def update_active_waypoint_marker(self, waypoints, user_wp_index):
        """Highlight the current target waypoint on the map (Phase 3.2)."""
        if waypoints and 0 <= user_wp_index < len(waypoints):
            target = waypoints[user_wp_index]
            self.active_wp_marker.setData([target.lon], [target.lat])
        else:
            self.active_wp_marker.setData([], [])
    
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
            self.active_wp_marker.setData([], [])
            self.flight_start_time = None
            self.flight_timer_label.setText("Flight Time: 00:00:00")
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
        """Save mission to file (QGC WPL 110 format, preserves loiter time)"""
        if len(self.waypoints) == 0:
            QMessageBox.warning(self, "No Mission", "No waypoints to save.")
            return
        
        filename, _ = QFileDialog.getSaveFileName(self, "Save Mission", "", "Waypoint Files (*.waypoints);;All Files (*)")
        if filename:
            try:
                with open(filename, 'w') as f:
                    # QGC WPL 110 format:
                    # seq current frame command p1 p2 p3 p4 lat lon alt autocontinue
                    f.write("QGC WPL 110\n")
                    for i, wp in enumerate(self.waypoints):
                        # command 16 = NAV_WAYPOINT, p1 = hold time (loiter), p2 = acceptance radius
                        f.write(f"{i+1}\t0\t3\t16\t{wp.loiter_time}\t{wp.acceptance_radius}\t0\t0\t{wp.lat:.8f}\t{wp.lon:.8f}\t{wp.alt:.1f}\t1\n")
                logger.info(f"Mission saved to {filename} ({len(self.waypoints)} waypoints)")
                QMessageBox.information(self, "Success", f"Mission saved: {len(self.waypoints)} waypoints")
            except Exception as e:
                logger.error(f"Failed to save mission: {e}")
                QMessageBox.critical(self, "Error", f"Failed to save: {e}")
    
    def load_mission(self):
        """Load mission from file (QGC WPL 110 format, restores loiter time)"""
        filename, _ = QFileDialog.getOpenFileName(self, "Load Mission", "", "Waypoint Files (*.waypoints);;All Files (*)")
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
                            wp = Waypoint(lat, lon, alt)
                            # Restore loiter time (p1) and acceptance radius (p2)
                            if len(parts) >= 6:
                                wp.loiter_time = int(float(parts[4]))
                                wp.acceptance_radius = float(parts[5])
                            self.waypoints.append(wp)
                
                self.update_waypoint_display()
                logger.info(f"Loaded {len(self.waypoints)} waypoints from {filename}")
                QMessageBox.information(self, "Success", f"Loaded {len(self.waypoints)} waypoints")
            except Exception as e:
                logger.error(f"Failed to load mission: {e}")
                QMessageBox.critical(self, "Error", f"Failed to load: {e}")
