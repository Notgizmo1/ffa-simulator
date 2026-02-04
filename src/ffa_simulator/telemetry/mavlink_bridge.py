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
    
    def _get_mode_number(self, mode_name):
        """Convert mode name to ArduPlane custom mode number"""
        mode_map = {
            "MANUAL": 0,
            "STABILIZE": 2,
            "FBWA": 5,
            "AUTO": 10,
            "RTL": 11,
            "LOITER": 12,
            "GUIDED": 15,
            "QSTABILIZE": 17,
            "QHOVER": 18,
            "QLOITER": 19,
            "QLAND": 20,
            "QRTL": 21
        }
        return mode_map.get(mode_name.upper(), None)
    
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
    
    async def send_command(self, command, params):
        """Send flight command to autopilot"""
        if not self.connected:
            logger.error("Cannot send command - not connected")
            return False
        
        try:
            if command == "ARM":
                return await self._arm()
            elif command == "DISARM":
                return await self._disarm()
            elif command == "TAKEOFF":
                altitude = params.get("altitude", 50)
                return await self._takeoff(altitude)
            elif command == "RTL":
                return await self._set_mode("RTL")
            elif command == "LAND":
                return await self._set_mode("QLAND")
            elif command == "AUTO":
                return await self._set_mode("AUTO")
            else:
                logger.warning(f"Unknown command: {command}")
                return False
        except Exception as e:
            logger.error(f"Command execution failed: {e}")
            return False
    
    async def _arm(self):
        """Arm the vehicle"""
        logger.info("Sending ARM command...")
        
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(
            None,
            self.connection.arducopter_arm
        )
        
        logger.info("ARM command sent")
        return True
    
    async def _disarm(self):
        """Disarm the vehicle"""
        logger.info("Sending DISARM command...")
        
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(
            None,
            self.connection.arducopter_disarm
        )
        
        logger.info("DISARM command sent")
        return True
    
    async def _set_mode(self, mode_name):
        """Set flight mode"""
        mode_num = self._get_mode_number(mode_name)
        if mode_num is None:
            logger.error(f"Invalid mode: {mode_name}")
            return False
        
        logger.info(f"Setting mode to {mode_name}...")
        
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(
            None,
            self.connection.set_mode,
            mode_num
        )
        
        logger.info(f"Mode change to {mode_name} sent")
        return True
    
    async def _takeoff(self, altitude):
        """Command VTOL takeoff"""
        logger.info(f"Commanding takeoff to {altitude}m...")
        
        # First switch to QLOITER mode
        await self._set_mode("QLOITER")
        await asyncio.sleep(0.5)
        
        # Then send takeoff command
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(
            None,
            self.connection.mav.command_long_send,
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,  # confirmation
            0, 0, 0, 0,  # params 1-4
            0, 0,  # lat, lon (0 = current position)
            altitude  # altitude
        )
        
        logger.info(f"Takeoff command sent (target: {altitude}m)")
        return True
    
    async def upload_mission(self, waypoints):
        """Upload waypoint mission to autopilot"""
        if not self.connected:
            logger.error("Cannot upload mission - not connected")
            return False
        
        try:
            logger.info(f"Uploading mission with {len(waypoints)} waypoints...")
            
            # Clear existing mission
            loop = asyncio.get_event_loop()
            
            # Send mission count
            await loop.run_in_executor(
                None,
                self.connection.waypoint_clear_all_send
            )
            
            await asyncio.sleep(0.5)
            
            # Upload waypoints
            mission_items = []
            
            # Item 0: Home position (required)
            mission_items.append(mavutil.mavlink.MAVLink_mission_item_message(
                self.connection.target_system,
                self.connection.target_component,
                0,  # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,  # current (0 for all waypoints except first)
                1,  # autocontinue
                0, 0, 0, 0,  # params 1-4
                self.latitude if self.latitude != 0 else 0,
                self.longitude if self.longitude != 0 else 0,
                0  # altitude (home is 0)
            ))
            
            # Add user waypoints
            for i, wp in enumerate(waypoints):
                mission_items.append(mavutil.mavlink.MAVLink_mission_item_message(
                    self.connection.target_system,
                    self.connection.target_component,
                    i + 1,  # seq (starts at 1 after home)
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    0,  # current
                    1,  # autocontinue
                    wp.loiter_time,  # param1 (hold time)
                    wp.acceptance_radius,  # param2
                    0, 0,  # params 3-4
                    wp.lat,
                    wp.lon,
                    wp.alt
                ))
            
            # Send mission count
            total_items = len(mission_items)
            await loop.run_in_executor(
                None,
                self.connection.mav.mission_count_send,
                self.connection.target_system,
                self.connection.target_component,
                total_items
            )
            
            await asyncio.sleep(0.5)
            
            # Send each mission item
            for item in mission_items:
                await loop.run_in_executor(
                    None,
                    self.connection.mav.send,
                    item
                )
                await asyncio.sleep(0.1)
            
            logger.info(f"Mission uploaded successfully ({total_items} items)")
            return True
            
        except Exception as e:
            logger.error(f"Mission upload failed: {e}", exc_info=True)
            return False
    
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