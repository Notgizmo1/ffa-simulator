import logging
import asyncio
from pymavlink import mavutil

logger = logging.getLogger(__name__)

class TelemetryBridge:
    """Handles MAVLink connection and telemetry data"""
    
    def __init__(self):
        self.connection = None
        self.connected = False
        self.running = False
        self.telemetry_task = None
        
        # Latest telemetry data
        self.mode = "UNKNOWN"
        self.armed = False
        self.battery_voltage = 0.0
        self.battery_remaining = 0
        self.gps_fix = 0
        self.gps_satellites = 0
        
        logger.info("TelemetryBridge initialized")
    
    async def connect(self, connection_string="tcp:127.0.0.1:5760"):
        """Connect to ArduPilot SITL via MAVLink"""
        logger.info(f"Connecting to MAVLink: {connection_string}")
        
        try:
            # Create connection (run in thread pool to avoid blocking)
            loop = asyncio.get_event_loop()
            self.connection = await loop.run_in_executor(
                None,
                lambda: mavutil.mavlink_connection(connection_string, timeout=5)
            )
            
            logger.info("Waiting for heartbeat...")
            
            # Wait for heartbeat with timeout
            start_time = loop.time()
            timeout = 15.0
            
            while loop.time() - start_time < timeout:
                msg = self.connection.recv_match(type='HEARTBEAT', blocking=False)
                if msg:
                    self.connected = True
                    logger.info(f"✓ Connected! System {self.connection.target_system}, Component {self.connection.target_component}")
                    
                    # Start telemetry monitoring
                    self.running = True
                    self.telemetry_task = asyncio.create_task(self._monitor_telemetry())
                    
                    return True
                
                await asyncio.sleep(0.1)
            
            logger.error("Timeout waiting for heartbeat")
            return False
            
        except Exception as e:
            logger.error(f"Connection failed: {e}")
            return False
    
    async def _monitor_telemetry(self):
        """Monitor MAVLink messages and update telemetry data"""
        logger.info("Telemetry monitoring started")
        
        try:
            while self.running and self.connected:
                # Non-blocking receive
                msg = self.connection.recv_match(blocking=False)
                
                if msg:
                    self._process_message(msg)
                else:
                    await asyncio.sleep(0.01)  # 100 Hz polling
                    
        except asyncio.CancelledError:
            logger.info("Telemetry monitoring cancelled")
        except Exception as e:
            logger.error(f"Telemetry error: {e}")
    
    def _process_message(self, msg):
        """Process individual MAVLink messages"""
        msg_type = msg.get_type()
        
        if msg_type == "HEARTBEAT":
            # Update mode
            mode_map = {
                0: "MANUAL", 1: "CIRCLE", 2: "STABILIZE", 3: "TRAINING",
                4: "ACRO", 5: "FBWA", 6: "FBWB", 7: "CRUISE",
                8: "AUTOTUNE", 10: "AUTO", 11: "RTL", 12: "LOITER",
                15: "GUIDED", 16: "INITIALIZING", 17: "QSTABILIZE",
                18: "QHOVER", 19: "QLOITER", 20: "QLAND", 21: "QRTL"
            }
            self.mode = mode_map.get(msg.custom_mode, f"MODE_{msg.custom_mode}")
            
            # Update armed status
            self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        
        elif msg_type == "SYS_STATUS":
            # Battery voltage (mV to V)
            self.battery_voltage = msg.voltage_battery / 1000.0
            # Battery remaining (%)
            self.battery_remaining = msg.battery_remaining
        
        elif msg_type == "GPS_RAW_INT":
            # GPS fix type (0=no fix, 3=3D fix)
            self.gps_fix = msg.fix_type
            # Number of satellites
            self.gps_satellites = msg.satellites_visible
    
    def get_telemetry(self):
        """Return current telemetry data as dict"""
        return {
            "mode": self.mode,
            "armed": self.armed,
            "battery_voltage": self.battery_voltage,
            "battery_remaining": self.battery_remaining,
            "gps_fix": self.gps_fix,
            "gps_satellites": self.gps_satellites
        }
    
    async def disconnect(self):
        """Disconnect from MAVLink"""
        logger.info("Disconnecting MAVLink...")
        
        self.running = False
        
        if self.telemetry_task:
            self.telemetry_task.cancel()
            try:
                await self.telemetry_task
            except asyncio.CancelledError:
                pass
        
        if self.connection:
            self.connection.close()
            self.connection = None
        
        self.connected = False
        logger.info("✓ MAVLink disconnected")