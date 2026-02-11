"""
Mission Control Panel - Interactive mission planning interface with progress tracking.
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QListWidget, QSpinBox, QGroupBox, QFileDialog, QProgressBar
)
from PySide6.QtCore import Signal, Slot
from PySide6.QtGui import QColor
import pyqtgraph as pg
from typing import List, Dict, Any, Optional
import json


class Waypoint:
    """Represents a mission waypoint."""
    
    def __init__(self, lat: float, lon: float, alt: float = 50,
                 wp_type: str = "WAYPOINT", loiter_time: float = 0,
                 acceptance_radius: float = 10):
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.wp_type = wp_type
        self.loiter_time = loiter_time
        self.acceptance_radius = acceptance_radius
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert waypoint to dictionary."""
        return {
            "lat": self.lat,
            "lon": self.lon,
            "alt": self.alt,
            "wp_type": self.wp_type,
            "loiter_time": self.loiter_time,
            "acceptance_radius": self.acceptance_radius
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'Waypoint':
        """Create waypoint from dictionary."""
        return cls(
            lat=data["lat"],
            lon=data["lon"],
            alt=data.get("alt", 50),
            wp_type=data.get("wp_type", "WAYPOINT"),
            loiter_time=data.get("loiter_time", 0),
            acceptance_radius=data.get("acceptance_radius", 10)
        )


class MissionControlPanel(QWidget):
    """Mission planning and control interface with progress tracking."""
    
    # Signals
    mission_upload_requested = Signal(list)
    command_requested = Signal(str, dict)
    
    def __init__(self):
        super().__init__()
        
        # Mission data
        self.waypoints: List[Waypoint] = []
        self.home_position: Optional[tuple] = None
        self.current_position: Optional[tuple] = None
        self.current_waypoint_index: int = 0
        
        # Mission progress data
        self.distance_to_waypoint: float = 0.0
        self.bearing_to_waypoint: float = 0.0
        self.groundspeed: float = 0.0
        self.eta_seconds: float = 0.0
        
        self._init_ui()
    
    def _init_ui(self):
        """Initialize UI components."""
        layout = QHBoxLayout(self)
        
        # Left column: Flight controls + mission stats
        left_column = QVBoxLayout()
        left_column.addWidget(self._create_flight_controls())
        left_column.addWidget(self._create_mission_stats())  # PROGRESS PANEL HERE!
        left_column.addStretch()
        
        # Center: Mission map
        center_layout = QVBoxLayout()
        center_layout.addWidget(self._create_mission_map())
        
        # Right column: Waypoint list
        right_layout = QVBoxLayout()
        right_layout.addWidget(self._create_waypoint_list())
        right_layout.addWidget(self._create_mission_io())
        
        layout.addLayout(left_column, 1)
        layout.addLayout(center_layout, 3)
        layout.addLayout(right_layout, 1)
    
    def _create_flight_controls(self) -> QGroupBox:
        """Create flight control buttons."""
        group = QGroupBox("Flight Controls")
        layout = QVBoxLayout(group)
        
        # First row: ARM / DISARM
        row1 = QHBoxLayout()
        self.arm_btn = QPushButton("ARM")
        self.arm_btn.setStyleSheet("background-color: #ff6b6b; color: white; font-weight: bold; padding: 10px;")
        self.arm_btn.clicked.connect(lambda: self.command_requested.emit("ARM", {}))
        
        self.disarm_btn = QPushButton("DISARM")
        self.disarm_btn.setStyleSheet("background-color: #51cf66; color: white; font-weight: bold; padding: 10px;")
        self.disarm_btn.clicked.connect(lambda: self.command_requested.emit("DISARM", {}))
        
        row1.addWidget(self.arm_btn)
        row1.addWidget(self.disarm_btn)
        
        # Second row: TAKEOFF / RTL / LAND
        row2 = QHBoxLayout()
        self.takeoff_btn = QPushButton("TAKEOFF")
        self.takeoff_btn.clicked.connect(self._handle_takeoff)
        
        self.rtl_btn = QPushButton("RTL (Return Home)")
        self.rtl_btn.clicked.connect(lambda: self.command_requested.emit("RTL", {}))
        
        self.land_btn = QPushButton("LAND NOW")
        self.land_btn.clicked.connect(lambda: self.command_requested.emit("LAND", {}))
        
        row2.addWidget(self.takeoff_btn)
        row2.addWidget(self.rtl_btn)
        row2.addWidget(self.land_btn)
        
        # Third row: START MISSION
        self.auto_btn = QPushButton("START MISSION (AUTO)")
        self.auto_btn.setStyleSheet("background-color: #4dabf7; color: white; font-weight: bold; padding: 12px;")
        self.auto_btn.clicked.connect(lambda: self.command_requested.emit("AUTO", {}))
        
        layout.addLayout(row1)
        layout.addLayout(row2)
        layout.addWidget(self.auto_btn)
        
        return group
    
    def _create_mission_stats(self) -> QGroupBox:
        """Create mission statistics panel - THIS IS THE NEW PROGRESS PANEL!"""
        group = QGroupBox("Mission Progress")
        layout = QVBoxLayout(group)
        
        # Progress bar
        self.progress_bar = QProgressBar()
        self.progress_bar.setMinimum(0)
        self.progress_bar.setMaximum(100)
        self.progress_bar.setValue(0)
        self.progress_bar.setTextVisible(True)
        layout.addWidget(self.progress_bar)
        
        # Current waypoint
        self.current_wp_label = QLabel("Current Waypoint: --")
        self.current_wp_label.setStyleSheet("font-weight: bold; color: #4dabf7;")
        layout.addWidget(self.current_wp_label)
        
        # Distance to waypoint
        self.distance_label = QLabel("Distance: -- m")
        layout.addWidget(self.distance_label)
        
        # Bearing to waypoint
        self.bearing_label = QLabel("Bearing: -- °")
        layout.addWidget(self.bearing_label)
        
        # ETA
        self.eta_label = QLabel("ETA: --")
        layout.addWidget(self.eta_label)
        
        # Ground speed
        self.speed_label = QLabel("Ground Speed: -- m/s")
        layout.addWidget(self.speed_label)
        
        return group
    
    def _create_mission_map(self) -> QGroupBox:
        """Create mission planning map."""
        group = QGroupBox("Mission Map (Click to Add Waypoint)")
        layout = QVBoxLayout(group)
        
        # PyQtGraph plot widget
        self.map_widget = pg.PlotWidget()
        self.map_widget.setLabel('left', 'Latitude (m°)')
        self.map_widget.setLabel('bottom', 'Longitude (m°)')
        self.map_widget.setAspectLocked(True)
        self.map_widget.showGrid(x=True, y=True, alpha=0.3)
        
        # Enable click to add waypoints
        self.map_widget.scene().sigMouseClicked.connect(self._on_map_click)
        
        # Plot items
        self.home_marker = pg.ScatterPlotItem(size=15, pen=pg.mkPen(None), brush=pg.mkBrush(0, 255, 0, 200))
        self.waypoint_markers = pg.ScatterPlotItem(size=12, pen=pg.mkPen(None), brush=pg.mkBrush(255, 0, 0, 200), symbol='s')
        self.current_position_marker = pg.ScatterPlotItem(size=12, pen=pg.mkPen(None), brush=pg.mkBrush(255, 255, 0, 200), symbol='t')
        self.current_waypoint_marker = pg.ScatterPlotItem(size=18, pen=pg.mkPen('b', width=3), brush=pg.mkBrush(None), symbol='o')
        self.path_line = pg.PlotDataItem(pen=pg.mkPen('b', width=2, style=pg.QtCore.Qt.DashLine))
        
        self.map_widget.addItem(self.home_marker)
        self.map_widget.addItem(self.path_line)
        self.map_widget.addItem(self.waypoint_markers)
        self.map_widget.addItem(self.current_waypoint_marker)
        self.map_widget.addItem(self.current_position_marker)
        
        layout.addWidget(self.map_widget)
        return group
    
    def _create_waypoint_list(self) -> QGroupBox:
        """Create waypoint list widget."""
        group = QGroupBox("Mission Waypoints")
        layout = QVBoxLayout(group)
        
        # Waypoint list
        self.waypoint_list = QListWidget()
        layout.addWidget(self.waypoint_list)
        
        # Parameters
        param_layout = QHBoxLayout()
        
        param_layout.addWidget(QLabel("Altitude (m):"))
        self.alt_spinbox = QSpinBox()
        self.alt_spinbox.setRange(0, 500)
        self.alt_spinbox.setValue(50)
        param_layout.addWidget(self.alt_spinbox)
        
        param_layout.addWidget(QLabel("Loiter (s):"))
        self.loiter_spinbox = QSpinBox()
        self.loiter_spinbox.setRange(0, 300)
        self.loiter_spinbox.setValue(0)
        param_layout.addWidget(self.loiter_spinbox)
        
        layout.addLayout(param_layout)
        
        # Control buttons
        btn_layout = QHBoxLayout()
        
        delete_btn = QPushButton("Delete Selected")
        delete_btn.clicked.connect(self._delete_selected_waypoint)
        btn_layout.addWidget(delete_btn)
        
        clear_btn = QPushButton("Clear All")
        clear_btn.clicked.connect(self._clear_waypoints)
        btn_layout.addWidget(clear_btn)
        
        layout.addLayout(btn_layout)
        
        return group
    
    def _create_mission_io(self) -> QGroupBox:
        """Create mission save/load controls."""
        group = QGroupBox("Mission File")
        layout = QVBoxLayout(group)
        
        # Save/Load buttons
        btn_layout = QHBoxLayout()
        
        save_btn = QPushButton("Save Mission")
        save_btn.clicked.connect(self._save_mission)
        btn_layout.addWidget(save_btn)
        
        load_btn = QPushButton("Load Mission")
        load_btn.clicked.connect(self._load_mission)
        btn_layout.addWidget(load_btn)
        
        layout.addLayout(btn_layout)
        
        # Upload button
        upload_btn = QPushButton("UPLOAD TO AUTOPILOT")
        upload_btn.setStyleSheet("background-color: #ff9800; color: white; font-weight: bold; padding: 10px;")
        upload_btn.clicked.connect(self._upload_mission)
        layout.addWidget(upload_btn)
        
        return group
    
    def _on_map_click(self, event):
        """Handle map click to add waypoint."""
        # Get click position in plot coordinates
        mouse_point = self.map_widget.plotItem.vb.mapSceneToView(event.scenePos())
        lon = mouse_point.x()
        lat = mouse_point.y()
        
        # Add waypoint
        alt = self.alt_spinbox.value()
        loiter = self.loiter_spinbox.value()
        wp = Waypoint(lat, lon, alt, loiter_time=loiter)
        self.waypoints.append(wp)
        
        # Update UI
        self._update_waypoint_list()
        self._update_map()
        
        print(f"[Mission] Added waypoint: WAYPOINT: ({lat:.6f}, {lon:.6f}) @ {alt}m")
    
    def _update_waypoint_list(self):
        """Update waypoint list widget."""
        self.waypoint_list.clear()
        for i, wp in enumerate(self.waypoints):
            current_marker = " <- CURRENT" if i == self.current_waypoint_index - 1 else ""
            self.waypoint_list.addItem(
                f"{i+1}. WAYPOINT: ({wp.lat:.6f}, {wp.lon:.6f}) @ {wp.alt}m{current_marker}"
            )
    
    def _update_map(self):
        """Update map visualization."""
        if not self.waypoints:
            self.waypoint_markers.setData([], [])
            self.path_line.setData([], [])
            self.current_waypoint_marker.setData([], [])
            return
        
        # Plot waypoints
        lats = [wp.lat for wp in self.waypoints]
        lons = [wp.lon for wp in self.waypoints]
        self.waypoint_markers.setData(lons, lats)
        
        # Plot path (home -> waypoints -> home)
        if self.home_position:
            path_lons = [self.home_position[1]] + lons + [self.home_position[1]]
            path_lats = [self.home_position[0]] + lats + [self.home_position[0]]
        else:
            path_lons = lons
            path_lats = lats
        
        self.path_line.setData(path_lons, path_lats)
        
        # Highlight current waypoint
        if 0 < self.current_waypoint_index <= len(self.waypoints):
            wp = self.waypoints[self.current_waypoint_index - 1]
            self.current_waypoint_marker.setData([wp.lon], [wp.lat])
        else:
            self.current_waypoint_marker.setData([], [])
    
    def _delete_selected_waypoint(self):
        """Delete selected waypoint."""
        current_row = self.waypoint_list.currentRow()
        if current_row >= 0:
            del self.waypoints[current_row]
            self._update_waypoint_list()
            self._update_map()
    
    def _clear_waypoints(self):
        """Clear all waypoints."""
        self.waypoints.clear()
        self.current_waypoint_index = 0
        self._update_waypoint_list()
        self._update_map()
        self._update_mission_stats(None)
    
    def _upload_mission(self):
        """Upload mission to autopilot."""
        if not self.waypoints:
            print("[Mission] ERROR: No waypoints to upload")
            return
        
        waypoint_dicts = [wp.to_dict() for wp in self.waypoints]
        self.mission_upload_requested.emit(waypoint_dicts)
        print(f"[Mission] Uploading mission with {len(waypoint_dicts)} waypoints...")
    
    def _handle_takeoff(self):
        """Handle takeoff button click."""
        altitude = self.alt_spinbox.value()
        self.command_requested.emit("TAKEOFF", {"altitude": altitude})
    
    def _save_mission(self):
        """Save mission to file."""
        if not self.waypoints:
            print("[Mission] ERROR: No waypoints to save")
            return
        
        filename, _ = QFileDialog.getSaveFileName(
            self, "Save Mission", "", "Waypoint Files (*.waypoints);;JSON Files (*.json)"
        )
        
        if filename:
            mission_data = {
                "waypoints": [wp.to_dict() for wp in self.waypoints],
                "home": self.home_position
            }
            
            with open(filename, 'w') as f:
                json.dump(mission_data, f, indent=2)
            
            print(f"[Mission] Saved to {filename}")
    
    def _load_mission(self):
        """Load mission from file."""
        filename, _ = QFileDialog.getOpenFileName(
            self, "Load Mission", "", "Waypoint Files (*.waypoints);;JSON Files (*.json)"
        )
        
        if filename:
            with open(filename, 'r') as f:
                mission_data = json.load(f)
            
            self.waypoints = [Waypoint.from_dict(wp) for wp in mission_data["waypoints"]]
            if mission_data.get("home"):
                self.home_position = tuple(mission_data["home"])
            
            self._update_waypoint_list()
            self._update_map()
            
            print(f"[Mission] Loaded {len(self.waypoints)} waypoints from {filename}")
    
    @Slot(tuple)
    def set_home_position(self, position: tuple):
        """Set home position (lat, lon)."""
        self.home_position = position
        self.home_marker.setData([position[1]], [position[0]])
        self._update_map()
        print(f"[Mission] Home position set: ({position[0]:.6f}, {position[1]:.6f})")
    
    @Slot(tuple)
    def update_current_position(self, position: tuple):
        """Update aircraft current position (lat, lon)."""
        self.current_position = position
        self.current_position_marker.setData([position[1]], [position[0]])
    
    @Slot(dict)
    def update_mission_progress(self, progress_data: dict):
        """Update mission progress display - THIS IS WHERE THE MAGIC HAPPENS!"""
        self.current_waypoint_index = progress_data.get("current_waypoint", 0)
        self.distance_to_waypoint = progress_data.get("distance_to_waypoint", 0.0)
        self.bearing_to_waypoint = progress_data.get("bearing_to_waypoint", 0.0)
        
        # Update progress bar
        progress_pct = progress_data.get("progress_percent", 0.0)
        self.progress_bar.setValue(int(progress_pct))
        
        # Update labels
        total_wp = progress_data.get("total_waypoints", len(self.waypoints))
        self.current_wp_label.setText(f"Current Waypoint: {self.current_waypoint_index} of {total_wp}")
        self.distance_label.setText(f"Distance: {self.distance_to_waypoint:.1f} m")
        self.bearing_label.setText(f"Bearing: {self.bearing_to_waypoint:.0f}°")
        
        # Calculate ETA
        if self.groundspeed > 1.0 and self.distance_to_waypoint > 0:
            self.eta_seconds = self.distance_to_waypoint / self.groundspeed
            eta_min = int(self.eta_seconds // 60)
            eta_sec = int(self.eta_seconds % 60)
            self.eta_label.setText(f"ETA: {eta_min}:{eta_sec:02d}")
        else:
            self.eta_label.setText("ETA: --")
        
        # Update map highlighting
        self._update_waypoint_list()
        self._update_map()
    
    @Slot(float)
    def update_groundspeed(self, speed: float):
        """Update ground speed for ETA calculation."""
        self.groundspeed = speed
        self.speed_label.setText(f"Ground Speed: {speed:.1f} m/s")
        
        # Recalculate ETA if we have distance data
        if self.distance_to_waypoint > 0:
            self.update_mission_progress({
                "current_waypoint": self.current_waypoint_index,
                "distance_to_waypoint": self.distance_to_waypoint,
                "bearing_to_waypoint": self.bearing_to_waypoint,
                "total_waypoints": len(self.waypoints),
                "progress_percent": (self.current_waypoint_index / max(len(self.waypoints), 1)) * 100
            })
    
    def _update_mission_stats(self, data):
        """Update mission statistics (fallback for when no progress data)."""
        if data is None:
            self.progress_bar.setValue(0)
            self.current_wp_label.setText("Current Waypoint: --")
            self.distance_label.setText("Distance: -- m")
            self.bearing_label.setText("Bearing: -- °")
            self.eta_label.setText("ETA: --")
            self.speed_label.setText("Ground Speed: -- m/s")
