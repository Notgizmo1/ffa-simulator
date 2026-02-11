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
        
        # Mission tracking (Phase 3.2)
        self.current_seq = 0          # Current waypoint sequence from MISSION_CURRENT
        self.mission_waypoints = []   # Stored waypoint objects from upload
        self.total_mission_wps = 0    # Total waypoints in uploaded mission
        
        # Message counter for debug
        self._msg_count = 0
        self._uploading = False  # Pause receive loop during mission upload
        
        logger.info("TelemetryBridge initialized")
    
    def set_mission_waypoints(self, waypoints):
        """Store uploaded mission waypoints for progress tracking.
        
        Args:
            waypoints: List of Waypoint objects with .lat, .lon, .alt attributes
        """
        self.mission_waypoints = list(waypoints)
        self.total_mission_wps = len(waypoints)
        self.current_seq = 0
        logger.info(f"Mission waypoints stored: {self.total_mission_wps} waypoints")
    
    def get_mission_state(self):
        """Get current mission progress state.
        
        Returns:
            dict with current_seq, total_wps, waypoints list, and active flag
        """
        return {
            'current_seq': self.current_seq,
            'total_wps': self.total_mission_wps,
            'waypoints': self.mission_waypoints,
            'mission_active': self.total_mission_wps > 0 and self.current_mode == "AUTO"
        }
    
    def clear_mission(self):
        """Clear stored mission data."""
        self.mission_waypoints = []
        self.total_mission_wps = 0
        self.current_seq = 0
        logger.info("Mission data cleared")
    
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
                # During mission upload, stop reading so upload thread gets protocol messages
                if self._uploading:
                    await asyncio.sleep(0.05)
                    continue
                
                # Receive message (non-blocking)
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
                    
                    elif msg_type == 'MISSION_CURRENT':
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
                return await self._start_mission()
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
    
    async def _start_mission(self):
        """Start mission with smart takeoff sequence.
        
        If on the ground: ARM → GUIDED → TAKEOFF → wait for altitude → AUTO
        If already in air: just switch to AUTO
        """
        # Check if already airborne (altitude > 3m)
        if self.altitude > 3.0:
            logger.info("Vehicle airborne, switching directly to AUTO")
            return await self._set_mode("AUTO")
        
        logger.info("Starting mission from ground - using GUIDED takeoff sequence...")
        
        # Step 1: Make sure we're armed
        if not self.armed:
            logger.info("Vehicle not armed, force arming...")
            await self._arm_force()
            # Wait for ARM confirmation
            for i in range(20):  # 2 second timeout
                await asyncio.sleep(0.1)
                if self.armed:
                    logger.info("Vehicle armed confirmed")
                    break
            else:
                logger.error("Failed to arm vehicle")
                return False
        
        # Step 2: Switch to GUIDED mode for takeoff
        logger.info("Setting GUIDED mode for takeoff...")
        await self._set_mode("GUIDED")
        await asyncio.sleep(0.5)
        
        # Step 3: Send takeoff command (use first WP altitude, or default 50m)
        takeoff_alt = 50
        if self.mission_waypoints:
            takeoff_alt = self.mission_waypoints[0].alt
        
        logger.info(f"Sending GUIDED takeoff to {takeoff_alt}m...")
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(
            None,
            self.connection.mav.command_long_send,
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,  # confirmation
            0, 0, 0, 0,  # params 1-4
            0, 0,  # lat, lon (current position)
            takeoff_alt  # altitude
        )
        
        # Step 4: Wait for vehicle to reach minimum altitude before switching to AUTO
        logger.info("Waiting for vehicle to reach altitude before starting AUTO...")
        min_alt = min(10.0, takeoff_alt * 0.5)  # At least 10m or 50% of target
        
        for i in range(300):  # 30 second timeout
            await asyncio.sleep(0.1)
            if self.altitude >= min_alt:
                logger.info(f"Altitude {self.altitude:.1f}m reached (threshold: {min_alt:.0f}m), switching to AUTO")
                break
            if not self.armed:
                logger.error("Vehicle disarmed during takeoff!")
                return False
        else:
            logger.warning(f"Altitude timeout ({self.altitude:.1f}m), switching to AUTO anyway")
        
        # Step 5: Switch to AUTO to start the mission
        await self._set_mode("AUTO")
        logger.info("Mission started in AUTO mode!")
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
        """Upload waypoint mission to autopilot using proper MAVLink protocol.
        
        MAVLink mission upload protocol:
        1. Send MISSION_COUNT(n)
        2. Autopilot sends MISSION_REQUEST(seq=0)
        3. We send MISSION_ITEM(seq=0)
        ... repeat for all items ...
        N. Autopilot sends MISSION_ACK
        """
        if not self.connected:
            logger.error("Cannot upload mission - not connected")
            return False
        
        try:
            logger.info(f"Uploading mission with {len(waypoints)} waypoints...")
            
            # Store waypoints for mission progress tracking (Phase 3.2)
            self.set_mission_waypoints(waypoints)
            
            # Pause the receive loop so it doesn't steal our protocol messages
            self._uploading = True
            await asyncio.sleep(0.15)  # Let receive loop notice the flag
            
            # Build mission items list
            mission_items = self._build_mission_items(waypoints)
            
            # Run the blocking upload protocol in a thread
            loop = asyncio.get_event_loop()
            result = await loop.run_in_executor(None, self._upload_protocol, mission_items)
            
            self._uploading = False
            return result
            
        except Exception as e:
            logger.error(f"Mission upload failed: {e}", exc_info=True)
            self._uploading = False
            return False
    
    def _build_mission_items(self, waypoints):
        """Build the list of MAVLink mission items."""
        mission_items = []
        
        # Item 0: Home position (required by ArduPilot)
        mission_items.append(mavutil.mavlink.MAVLink_mission_item_int_message(
            self.connection.target_system,
            self.connection.target_component,
            0,  # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 1,  # current, autocontinue
            0, 0, 0, 0,  # params 1-4
            int((self.latitude if self.latitude != 0 else -35.363262) * 1e7),
            int((self.longitude if self.longitude != 0 else 149.165237) * 1e7),
            0  # altitude (home is 0)
        ))
        
        # Item 1: NAV_TAKEOFF (required for AUTO mode from ground)
        takeoff_alt = waypoints[0].alt if waypoints else 50
        mission_items.append(mavutil.mavlink.MAVLink_mission_item_int_message(
            self.connection.target_system,
            self.connection.target_component,
            1,  # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 1,  # current, autocontinue
            0, 0, 0, 0,  # params 1-4
            0, 0,  # lat, lon (ignored for takeoff)
            takeoff_alt
        ))
        logger.info(f"Added NAV_TAKEOFF to {takeoff_alt}m as mission item 1")
        
        # Items 2+: User waypoints
        for i, wp in enumerate(waypoints):
            mission_items.append(mavutil.mavlink.MAVLink_mission_item_int_message(
                self.connection.target_system,
                self.connection.target_component,
                i + 2,  # seq (home=0, takeoff=1, user WPs=2+)
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, 1,  # current, autocontinue
                wp.loiter_time,  # param1 (hold time)
                wp.acceptance_radius,  # param2
                0, 0,  # params 3-4
                int(wp.lat * 1e7),
                int(wp.lon * 1e7),
                wp.alt
            ))
        
        return mission_items
    
    def _upload_protocol(self, mission_items):
        """Execute the MAVLink mission upload handshake (blocking, runs in thread)."""
        total_items = len(mission_items)
        
        try:
            # Drain any pending messages
            while self.connection.recv_match(blocking=False):
                pass
            
            logger.info(f"Sending MISSION_COUNT: {total_items} items")
            
            # Step 1: Send MISSION_COUNT
            self.connection.mav.mission_count_send(
                self.connection.target_system,
                self.connection.target_component,
                total_items
            )
            
            # Step 2: Handle MISSION_REQUEST/ACK handshake
            items_sent = 0
            retries = 0
            max_retries = 3
            import time
            start_time = time.time()
            timeout_seconds = 15
            
            while items_sent < total_items:
                elapsed = time.time() - start_time
                if elapsed > timeout_seconds:
                    logger.error(f"Mission upload timed out after {elapsed:.1f}s (sent {items_sent}/{total_items})")
                    return False
                
                msg = self.connection.recv_match(
                    type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'],
                    blocking=True,
                    timeout=3
                )
                
                if msg is None:
                    retries += 1
                    if retries > max_retries:
                        logger.error(f"No response from autopilot after {max_retries} retries")
                        return False
                    logger.warning(f"No MISSION_REQUEST received (retry {retries}/{max_retries}), resending MISSION_COUNT...")
                    self.connection.mav.mission_count_send(
                        self.connection.target_system,
                        self.connection.target_component,
                        total_items
                    )
                    continue
                
                msg_type = msg.get_type()
                
                if msg_type in ('MISSION_REQUEST', 'MISSION_REQUEST_INT'):
                    seq = msg.seq
                    if seq < total_items:
                        self.connection.mav.send(mission_items[seq])
                        items_sent = max(items_sent, seq + 1)
                        logger.info(f"Sent mission item {seq}/{total_items - 1} (type: {mission_items[seq].command})")
                        retries = 0  # Reset retries on success
                    else:
                        logger.warning(f"Requested seq {seq} out of range (total: {total_items})")
                
                elif msg_type == 'MISSION_ACK':
                    if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                        logger.info(f"MISSION_ACK: ACCEPTED — {total_items} items uploaded successfully")
                        return True
                    else:
                        logger.error(f"MISSION_ACK: REJECTED (type={msg.type})")
                        return False
            
            # All items sent, wait for final ACK
            logger.info(f"All {total_items} items sent, waiting for MISSION_ACK...")
            msg = self.connection.recv_match(
                type=['MISSION_ACK'],
                blocking=True,
                timeout=5
            )
            
            if msg and msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                logger.info(f"MISSION_ACK: ACCEPTED — upload complete")
                return True
            elif msg:
                logger.error(f"MISSION_ACK: REJECTED (type={msg.type})")
                return False
            else:
                logger.error("No MISSION_ACK received after sending all items")
                return False
                
        except Exception as e:
            logger.error(f"Upload protocol error: {e}", exc_info=True)
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
