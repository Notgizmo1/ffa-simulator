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
            
            logger.info("Waiting for heartbeat (10 second timeout)...")
            heartbeat = await loop.run_in_executor(
                None,
                self.connection.wait_heartbeat,
                10  # 10 second timeout explicitly set
            )
            
            if heartbeat:
                logger.info(f"Connected! System {self.connection.target_system}, Component {self.connection.target_component}")
                self.connected = True
                
                # Now request data streams
                logger.info("Requesting data streams...")
                await loop.run_in_executor(
                    None,
                    self.connection.mav.request_data_stream_send,
                    self.connection.target_system,
                    self.connection.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_ALL,
                    4,  # 4 Hz
                    1   # start
                )
                
                # Start receive task
                self.receive_task = asyncio.create_task(self._receive_messages())
                
                logger.info("Telemetry monitoring started")
                return True
            else:
                logger.error("No heartbeat received after timeout")
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
                    
                    elif msg.get_type() == 'MISSION_CURRENT':
                        self.current_seq = msg.seq
                        logger.debug(f"Current waypoint: {self.current_seq}")

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
        """Convert ArduCopter custom mode to name"""
        mode_map = {
            0: "STABILIZE",
            1: "ACRO",
            2: "ALT_HOLD",
            3: "AUTO",
            4: "GUIDED",
            5: "LOITER",
            6: "RTL",
            7: "CIRCLE",
            9: "LAND",
            11: "DRIFT",
            13: "SPORT",
            14: "FLIP",
            15: "AUTOTUNE",
            16: "POSHOLD",
            17: "BRAKE",
            18: "THROW",
            19: "AVOID_ADSB",
            20: "GUIDED_NOGPS",
            21: "SMART_RTL",
            22: "FLOWHOLD",
            23: "FOLLOW",
            24: "ZIGZAG",
            25: "SYSTEMID",
            26: "AUTOROTATE"
        }
        return mode_map.get(custom_mode, f"MODE_{custom_mode}")
    
    def _get_mode_number(self, mode_name):
        """Convert mode name to ArduCopter custom mode number"""
        mode_map = {
            "STABILIZE": 0,
            "ACRO": 1,
            "ALT_HOLD": 2,
            "AUTO": 3,
            "GUIDED": 4,
            "LOITER": 5,
            "RTL": 6,
            "CIRCLE": 7,
            "LAND": 9,
            "DRIFT": 11,
            "SPORT": 13,
            "FLIP": 14,
            "AUTOTUNE": 15,
            "POSHOLD": 16,
            "BRAKE": 17,
            "THROW": 18,
            "AVOID_ADSB": 19,
            "GUIDED_NOGPS": 20,
            "SMART_RTL": 21,
            "FLOWHOLD": 22,
            "FOLLOW": 23,
            "ZIGZAG": 24,
            "SYSTEMID": 25,
            "AUTOROTATE": 26,
            "QSTABILIZE": 0,  # Alias for STABILIZE in ArduCopter
            "QLOITER": 5,     # Alias for LOITER in ArduCopter
            "QHOVER": 2,      # Alias for ALT_HOLD in ArduCopter
            "QLAND": 9,       # Alias for LAND in ArduCopter
            "QRTL": 6         # Alias for RTL in ArduCopter
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
            elif command == "ARM_FORCE":
                return await self._arm_force()
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
            elif command == "CHANGE_ALTITUDE":
                altitude = params.get("altitude", 100)
                return await self._change_altitude(altitude)
            elif command == "SET_MODE":
                mode = params.get("mode", "GUIDED")
                return await self._set_mode(mode)
            elif command == "LOITER_HERE":
                return await self._loiter_here()
            elif command == "SET_LOITER_RADIUS":
                radius = params.get("radius", 50)
                return await self._set_loiter_radius(radius)
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
    
    async def _arm_force(self):
        """Force ARM bypassing PreArm checks (for SITL testing)"""
        logger.info("Sending FORCE ARM command (bypassing PreArm checks)...")
        
        # MAV_CMD_COMPONENT_ARM_DISARM with force parameter
        # param1: 1 = ARM, param2: 21196 = force magic number
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(
            None,
            self.connection.mav.command_long_send,
            self.connection.target_system,
            self.connection.target_component,
            400,  # MAV_CMD_COMPONENT_ARM_DISARM
            0,    # confirmation
            1,    # param1: 1 = ARM
            21196,  # param2: 21196 = force ARM (bypasses PreArm)
            0, 0, 0, 0, 0  # unused params
        )
        
        logger.info("FORCE ARM command sent (PreArm checks bypassed)")
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
    
    async def _change_altitude(self, altitude):
        """Change target altitude (for GUIDED or LOITER modes)"""
        logger.info(f"Changing altitude to {altitude}m...")
        
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(
            None,
            self.connection.mav.command_long_send,
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_CHANGE_ALT,
            0,  # confirmation
            altitude,  # param1: target altitude (meters)
            0,  # param2: frame (0 = relative to home)
            0, 0, 0, 0, 0  # unused params
        )
        
        logger.info(f"Altitude change command sent: {altitude}m")
        return True
    
    async def _loiter_here(self):
        """Begin unlimited loiter at current position"""
        logger.info("Commanding loiter at current position...")
        
        # Switch to LOITER mode
        await self._set_mode("LOITER")
        
        logger.info("Loiter command sent")
        return True
    
    async def _set_loiter_radius(self, radius):
        """Set loiter circle radius"""
        logger.info(f"Setting loiter radius to {radius}m...")
        
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(
            None,
            self.connection.mav.command_long_send,
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
            0,  # confirmation
            0,  # param1: ROI mode
            0,  # param2: WP index
            0,  # param3: ROI index
            0,  # param4: empty
            radius,  # param5: radius in meters
            0, 0  # params 6-7: empty
        )
        
        logger.info(f"Loiter radius set to {radius}m")
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