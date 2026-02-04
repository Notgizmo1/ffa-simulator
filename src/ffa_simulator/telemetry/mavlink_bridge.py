import logging
import asyncio
import numpy as np
from pymavlink import mavutil

logger = logging.getLogger(__name__)

class TelemetryBridge:
    """MAVLink telemetry bridge for ArduPilot SITL"""
    
    def __init__(self):
        self.connection = None
        self.connected = False
        self.receive_task = None
        
        # Flight mode and status
        self.current_mode = "UNKNOWN"
        self.armed = False
        
        # Battery
        self.battery_voltage = 0.0
        self.battery_remaining = 0
        
        # GPS
        self.gps_fix_type = 0
        self.gps_satellites = 0
        
        # Position and attitude
        self.altitude = 0.0
        self.groundspeed = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.latitude = 0.0
        self.longitude = 0.0
        
        # Message counter for debug
        self._msg_count = 0
        
        logger.info("TelemetryBridge initialized")
    
    async def connect(self, connection_string):
        """Connect to MAVLink endpoint"""
        try:
            logger.info(f"Connecting to MAVLink: {connection_string}")
            
            loop = asyncio.get_event_loop()
            self.connection = await loop.run_in_executor(
                None,
                mavutil.mavlink_connection,
                connection_string,
                10
            )
            
            logger.info("Waiting for heartbeat...")
            heartbeat = await loop.run_in_executor(
                None,
                self.connection.wait_heartbeat
            )
            
            if heartbeat:
                logger.info(f"Connected! System {self.connection.target_system}, Component {self.connection.target_component}")
                self.connected = True
                
                # Start receive task
                self.receive_task = asyncio.create_task(self._receive_messages())
                
                logger.info("Telemetry monitoring started")
                return True
            else:
                logger.error("No heartbeat received")
                return False
                
        except Exception as e:
            logger.error(f"Connection failed: {e}")
            self.connected = False
            return False
    
    async def _receive_messages(self):
        """Async message receive loop"""
        logger.info("Starting message receive loop...")
        
        try:
            while self.connected:
                # Receive message (blocking call in executor)
                msg = self.connection.recv_match(blocking=False)
                
                if msg:
                    msg_type = msg.get_type()
                    
                    # DEBUG: Log first 20 messages
                    if self._msg_count < 20:
                        logger.info(f"[RX] {msg_type}")
                        self._msg_count += 1
                    
                    if msg_type == 'HEARTBEAT':
                        self.current_mode = self._get_mode_name(msg.custom_mode)
                        self.armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                        if self._msg_count == 1:
                            logger.info(f"Mode: {self.current_mode}, Armed: {self.armed}")
                    
                    elif msg_type == 'SYS_STATUS':
                        self.battery_voltage = msg.voltage_battery / 1000.0
                        self.battery_remaining = msg.battery_remaining
                    
                    elif msg_type == 'GPS_RAW_INT':
                        self.gps_fix_type = msg.fix_type
                        self.gps_satellites = msg.satellites_visible
                    
                    elif msg_type == 'ATTITUDE':
                        self.roll = np.degrees(msg.roll)
                        self.pitch = np.degrees(msg.pitch)
                        self.yaw = np.degrees(msg.yaw)
                    
                    elif msg_type == 'GLOBAL_POSITION_INT':
                        self.altitude = msg.relative_alt / 1000.0
                        self.groundspeed = np.sqrt(msg.vx**2 + msg.vy**2) / 100.0
                        self.latitude = msg.lat / 1e7
                        self.longitude = msg.lon / 1e7
                
                # Small delay to prevent CPU spinning
                await asyncio.sleep(0.01)
                
        except asyncio.CancelledError:
            logger.info("Message receive loop cancelled")
        except Exception as e:
            logger.error(f"Error in receive loop: {e}", exc_info=True)
    
    def _get_mode_name(self, custom_mode):
        """Convert ArduPlane custom mode to name"""
        mode_map = {
            0: "MANUAL",
            1: "CIRCLE",
            2: "STABILIZE",
            3: "TRAINING",
            4: "ACRO",
            5: "FBWA",
            6: "FBWB",
            7: "CRUISE",
            8: "AUTOTUNE",
            10: "AUTO",
            11: "RTL",
            12: "LOITER",
            14: "AVOID_ADSB",
            15: "GUIDED",
            16: "INITIALISING",
            17: "QSTABILIZE",
            18: "QHOVER",
            19: "QLOITER",
            20: "QLAND",
            21: "QRTL",
            22: "QAUTOTUNE",
            23: "QACRO"
        }
        return mode_map.get(custom_mode, f"MODE_{custom_mode}")
    
    def get_telemetry(self):
        """Get current telemetry data snapshot"""
        return {
            'mode': self.current_mode,
            'armed': self.armed,
            'battery_voltage': self.battery_voltage,
            'battery_remaining': self.battery_remaining,
            'gps_fix': self.gps_fix_type,
            'gps_satellites': self.gps_satellites,
            'altitude': self.altitude,
            'groundspeed': self.groundspeed,
            'roll': self.roll,
            'pitch': self.pitch,
            'yaw': self.yaw,
            'latitude': self.latitude,
            'longitude': self.longitude
        }
    
    async def disconnect(self):
        """Disconnect from MAVLink"""
        logger.info("Disconnecting telemetry...")
        
        self.connected = False
        
        if self.receive_task:
            self.receive_task.cancel()
            try:
                await self.receive_task
            except asyncio.CancelledError:
                pass
            self.receive_task = None
        
        if self.connection:
            self.connection.close()
            self.connection = None
        
        logger.info("Telemetry disconnected")