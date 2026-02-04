import logging
import numpy as np
from collections import deque
from PySide6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QGridLayout
from PySide6.QtCore import Qt
import pyqtgraph as pg

logger = logging.getLogger(__name__)

class TelemetryPanel(QWidget):
    """Real-time telemetry visualization with PyQtGraph"""
    
    def __init__(self):
        super().__init__()
        
        # Data buffers (60 seconds at 10 Hz = 600 points)
        self.max_points = 600
        self.time_data = deque(maxlen=self.max_points)
        self.altitude_data = deque(maxlen=self.max_points)
        self.speed_data = deque(maxlen=self.max_points)
        self.roll_data = deque(maxlen=self.max_points)
        self.pitch_data = deque(maxlen=self.max_points)
        self.yaw_data = deque(maxlen=self.max_points)
        
        # GPS track (lat/lon points)
        self.gps_lat = deque(maxlen=1000)
        self.gps_lon = deque(maxlen=1000)
        
        self.start_time = None
        
        self.init_ui()
        
    def init_ui(self):
        """Setup PyQtGraph plots"""
        layout = QVBoxLayout(self)
        
        # Status bar
        status_layout = QHBoxLayout()
        self.mode_label = QLabel("Mode: --")
        self.armed_label = QLabel("Armed: No")
        self.battery_label = QLabel("Battery: --")
        self.gps_label = QLabel("GPS: --")
        
        for label in [self.mode_label, self.armed_label, self.battery_label, self.gps_label]:
            label.setStyleSheet("padding: 5px; background-color: #f0f0f0; border-radius: 3px; font-weight: bold;")
            status_layout.addWidget(label)
        
        layout.addLayout(status_layout)
        
        # Configure PyQtGraph
        pg.setConfigOptions(antialias=True)
        
        # Grid layout for plots (2x2)
        plot_grid = QGridLayout()
        
        # Altitude plot (top-left)
        self.altitude_plot = pg.PlotWidget(title="Altitude (m)")
        self.altitude_plot.setLabel('left', 'Altitude', units='m')
        self.altitude_plot.setLabel('bottom', 'Time', units='s')
        self.altitude_plot.showGrid(x=True, y=True, alpha=0.3)
        self.altitude_curve = self.altitude_plot.plot(pen=pg.mkPen(color='#2196F3', width=2))
        plot_grid.addWidget(self.altitude_plot, 0, 0)
        
        # Ground speed plot (top-right)
        self.speed_plot = pg.PlotWidget(title="Ground Speed (m/s)")
        self.speed_plot.setLabel('left', 'Speed', units='m/s')
        self.speed_plot.setLabel('bottom', 'Time', units='s')
        self.speed_plot.showGrid(x=True, y=True, alpha=0.3)
        self.speed_curve = self.speed_plot.plot(pen=pg.mkPen(color='#4CAF50', width=2))
        plot_grid.addWidget(self.speed_plot, 0, 1)
        
        # Attitude plot (bottom-left)
        self.attitude_plot = pg.PlotWidget(title="Attitude (degrees)")
        self.attitude_plot.setLabel('left', 'Angle', units='°')
        self.attitude_plot.setLabel('bottom', 'Time', units='s')
        self.attitude_plot.showGrid(x=True, y=True, alpha=0.3)
        self.attitude_plot.addLegend()
        self.roll_curve = self.attitude_plot.plot(pen=pg.mkPen(color='#F44336', width=2), name='Roll')
        self.pitch_curve = self.attitude_plot.plot(pen=pg.mkPen(color='#FF9800', width=2), name='Pitch')
        self.yaw_curve = self.attitude_plot.plot(pen=pg.mkPen(color='#9C27B0', width=2), name='Yaw')
        plot_grid.addWidget(self.attitude_plot, 1, 0)
        
        # GPS track plot (bottom-right)
        self.gps_plot = pg.PlotWidget(title="GPS Track")
        self.gps_plot.setLabel('left', 'Latitude', units='°')
        self.gps_plot.setLabel('bottom', 'Longitude', units='°')
        self.gps_plot.showGrid(x=True, y=True, alpha=0.3)
        self.gps_plot.setAspectLocked(True)
        self.gps_curve = self.gps_plot.plot(pen=None, symbol='o', symbolSize=3, symbolBrush='#2196F3')
        plot_grid.addWidget(self.gps_plot, 1, 1)
        
        layout.addLayout(plot_grid)
        
        logger.info("Telemetry panel initialized")
    
    def update_display(self, telemetry):
        """Update all plots with new telemetry data"""
        import time
        
        # Initialize start time on first update
        if self.start_time is None:
            self.start_time = time.time()
        
        current_time = time.time() - self.start_time
        
        # Update status labels
        self.mode_label.setText(f"Mode: {telemetry['mode']}")
        
        if telemetry['armed']:
            self.armed_label.setText("Armed: YES")
            self.armed_label.setStyleSheet("padding: 5px; background-color: #ff6b6b; color: white; border-radius: 3px; font-weight: bold;")
        else:
            self.armed_label.setText("Armed: No")
            self.armed_label.setStyleSheet("padding: 5px; background-color: #f0f0f0; border-radius: 3px; font-weight: bold;")
        
        voltage = telemetry['battery_voltage']
        remaining = telemetry['battery_remaining']
        if voltage > 0:
            self.battery_label.setText(f"Battery: {voltage:.1f}V ({remaining}%)")
        else:
            self.battery_label.setText("Battery: --")
        
        fix_type = telemetry['gps_fix']
        sats = telemetry['gps_satellites']
        if fix_type >= 3:
            self.gps_label.setText(f"GPS: 3D Fix ({sats} sats)")
            self.gps_label.setStyleSheet("padding: 5px; background-color: #51cf66; color: white; border-radius: 3px; font-weight: bold;")
        elif fix_type > 0:
            self.gps_label.setText(f"GPS: No Fix ({sats} sats)")
            self.gps_label.setStyleSheet("padding: 5px; background-color: #ffd43b; color: black; border-radius: 3px; font-weight: bold;")
        else:
            self.gps_label.setText("GPS: --")
            self.gps_label.setStyleSheet("padding: 5px; background-color: #f0f0f0; border-radius: 3px; font-weight: bold;")
        
        # Append new data to buffers
        self.time_data.append(current_time)
        self.altitude_data.append(telemetry['altitude'])
        self.speed_data.append(telemetry['groundspeed'])
        self.roll_data.append(telemetry['roll'])
        self.pitch_data.append(telemetry['pitch'])
        self.yaw_data.append(telemetry['yaw'])
        
        # GPS track (only if valid fix)
        if telemetry['latitude'] != 0 and telemetry['longitude'] != 0:
            self.gps_lat.append(telemetry['latitude'])
            self.gps_lon.append(telemetry['longitude'])
        
        # Update plots (convert deques to numpy arrays)
        time_array = np.array(self.time_data)
        
        if len(time_array) > 1:
            self.altitude_curve.setData(time_array, np.array(self.altitude_data))
            self.speed_curve.setData(time_array, np.array(self.speed_data))
            self.roll_curve.setData(time_array, np.array(self.roll_data))
            self.pitch_curve.setData(time_array, np.array(self.pitch_data))
            self.yaw_curve.setData(time_array, np.array(self.yaw_data))
        
        if len(self.gps_lon) > 0:
            self.gps_curve.setData(np.array(self.gps_lon), np.array(self.gps_lat))
    
    def reset(self):
        """Clear all data buffers"""
        self.time_data.clear()
        self.altitude_data.clear()
        self.speed_data.clear()
        self.roll_data.clear()
        self.pitch_data.clear()
        self.yaw_data.clear()
        self.gps_lat.clear()
        self.gps_lon.clear()
        self.start_time = None
        
        # Clear plots
        self.altitude_curve.setData([], [])
        self.speed_curve.setData([], [])
        self.roll_curve.setData([], [])
        self.pitch_curve.setData([], [])
        self.yaw_curve.setData([], [])
        self.gps_curve.setData([], [])
        
        logger.info("Telemetry panel reset")