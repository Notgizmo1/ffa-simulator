"""
MAVLink telemetry bridge for ArduPilot SITL.
Connects to ArduPilot via MAVLink and streams telemetry data.
"""

import asyncio
from typing import Optional, List, Dict, Any
from pymavlink import mavutil
from PySide6.QtCore import QObject, Signal
import numpy as np


class MAVLinkBridge(QObject):
    """Bridge between ArduPilot SITL and the GUI via MAVLink."""
    
    # Telemetry signals
    heartbeat_received = Signal(dict)
    gps_data_received = Signal(dict)
    attitude_received = Signal(dict)
    position_received = Signal(dict)
    mission_progress_received = Signal(dict)  # Mission progress data
    
    # Status signals
    connection_status = Signal(bool, str)
    
    def __init__(self, connection_string: str = "tcp:172.24.119.69:5760"):
        super().__init__()
        self.connection_string = connection_string
        self.connection: Optional[mavutil.mavlink_connection] = None
        self.running = False
        self._message_count = 0
        self._debug_limit = 20
        
        # Mission tracking state
        self.current_waypoint = 0
        self.total_waypoints = 0
        self.distance_to_waypoint = 0.0
        self.bearing_to_waypoint = 0
        
    async def connect(self) -> bool:
        """Establish MAVLink connection to ArduPilot SITL."""
        try:
            print(f"[MAVLink] Connecting to {self.connection_string}...")
            
            loop = asyncio.get_event_loop()
            self.connection = await loop.run_in_executor(
                None,
                lambda: mavutil.mavlink_connection(
                    self.connection_string,
                    source_system=255,
                    source_component=0
                )
            )
            
            print("[MAVLink] Waiting for heartbeat...")
            heartbeat = await loop.run_in_executor(
                None,
                lambda: self.connection.wait_heartbeat(timeout=10)
            )
            
            if heartbeat:
                print(f"[MAVLink] Connected! System: {self.connection.target_system}, "
                      f"Component: {self.connection.target_component}")
                self.connection_status.emit(True, "Connected")
                return True
            else:
                print("[MAVLink] ERROR: No heartbeat received")
                self.connection_status.emit(False, "No heartbeat")
                return False
                
        except Exception as e:
            print(f"[MAVLink] ERROR: Connection failed: {e}")
            self.connection_status.emit(False, str(e))
            return False
    
    async def start_telemetry(self):
        """Start receiving telemetry data."""
        if not self.connection:
            print("[MAVLink] ERROR: Not connected")
            return
        
        self.running = True
        print("[MAVLink] Starting telemetry stream...")
        
        await self._request_data_streams()
        asyncio.create_task(self._receive_messages())
    
    async def _request_data_streams(self):
        """Request specific data streams from ArduPilot."""
        loop = asyncio.get_event_loop()
        
        await loop.run_in_executor(
            None,
            lambda: self.connection.mav.request_data_stream_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                10,
                1
            )
        )
        print("[MAVLink] Requested data streams at 10 Hz")
    
    async def _receive_messages(self):
        """Main message receive loop (non-blocking)."""
        loop = asyncio.get_event_loop()
        
        while self.running:
            try:
                msg = await loop.run_in_executor(
                    None,
                    lambda: self.connection.recv_match(blocking=False)
                )
                
                if msg:
                    self._message_count += 1
                    if self._message_count <= self._debug_limit:
                        print(f"[RX] {msg.get_type()}")
                    
                    await self._process_message(msg)
                else:
                    await asyncio.sleep(0.01)
                    
            except Exception as e:
                print(f"[MAVLink] ERROR receiving message: {e}")
                await asyncio.sleep(0.1)
    
    async def _process_message(self, msg):
        """Process received MAVLink message."""
        msg_type = msg.get_type()
        
        if msg_type == "HEARTBEAT":
            await self._handle_heartbeat(msg)
        elif msg_type == "GPS_RAW_INT":
            await self._handle_gps_raw(msg)
        elif msg_type == "ATTITUDE":
            await self._handle_attitude(msg)
        elif msg_type == "GLOBAL_POSITION_INT":
            await self._handle_global_position(msg)
        elif msg_type == "MISSION_CURRENT":
            await self._handle_mission_current(msg)
        elif msg_type == "NAV_CONTROLLER_OUTPUT":
            await self._handle_nav_controller(msg)
    
    async def _handle_heartbeat(self, msg):
        """Process HEARTBEAT message."""
        data = {
            "type": msg.type,
            "autopilot": msg.autopilot,
            "base_mode": msg.base_mode,
            "custom_mode": msg.custom_mode,
            "system_status": msg.system_status,
            "mavlink_version": msg.mavlink_version
        }
        self.heartbeat_received.emit(data)
    
    async def _handle_gps_raw(self, msg):
        """Process GPS_RAW_INT message."""
        data = {
            "fix_type": msg.fix_type,
            "satellites_visible": msg.satellites_visible,
            "lat": msg.lat / 1e7,
            "lon": msg.lon / 1e7,
            "alt": msg.alt / 1000.0,
            "eph": msg.eph / 100.0,
            "epv": msg.epv / 100.0
        }
        self.gps_data_received.emit(data)
    
    async def _handle_attitude(self, msg):
        """Process ATTITUDE message."""
        data = {
            "roll": np.degrees(msg.roll),
            "pitch": np.degrees(msg.pitch),
            "yaw": np.degrees(msg.yaw),
            "rollspeed": msg.rollspeed,
            "pitchspeed": msg.pitchspeed,
            "yawspeed": msg.yawspeed
        }
        self.attitude_received.emit(data)
    
    async def _handle_global_position(self, msg):
        """Process GLOBAL_POSITION_INT message."""
        data = {
            "lat": msg.lat / 1e7,
            "lon": msg.lon / 1e7,
            "alt": msg.alt / 1000.0,
            "relative_alt": msg.relative_alt / 1000.0,
            "vx": msg.vx / 100.0,
            "vy": msg.vy / 100.0,
            "vz": msg.vz / 100.0,
            "hdg": msg.hdg / 100.0
        }
        data["groundspeed"] = np.sqrt(data["vx"]**2 + data["vy"]**2)
        
        self.position_received.emit(data)
    
    async def _handle_mission_current(self, msg):
        """Process MISSION_CURRENT message - indicates current waypoint."""
        self.current_waypoint = msg.seq
        print(f"[MAVLink] Current waypoint: {self.current_waypoint}")
        self._emit_mission_progress()
    
    async def _handle_nav_controller(self, msg):
        """Process NAV_CONTROLLER_OUTPUT - navigation controller status."""
        self.distance_to_waypoint = msg.wp_dist
        self.bearing_to_waypoint = msg.target_bearing
        self._emit_mission_progress()
    
    def _emit_mission_progress(self):
        """Emit mission progress signal with current state."""
        if self.total_waypoints > 0:
            progress_pct = (self.current_waypoint / self.total_waypoints) * 100
        else:
            progress_pct = 0.0
        
        data = {
            "current_waypoint": self.current_waypoint,
            "total_waypoints": self.total_waypoints,
            "distance_to_waypoint": self.distance_to_waypoint,
            "bearing_to_waypoint": self.bearing_to_waypoint,
            "progress_percent": progress_pct
        }
        self.mission_progress_received.emit(data)
    
    async def send_command(self, command: str, params: Dict[str, Any] = None):
        """Send command to autopilot."""
        if not self.connection:
            print("[MAVLink] ERROR: Not connected")
            return False
        
        params = params or {}
        print(f"[MAVLink] Sending command: {command} with params: {params}")
        
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
                print(f"[MAVLink] ERROR: Unknown command: {command}")
                return False
                
        except Exception as e:
            print(f"[MAVLink] ERROR sending command: {e}")
            return False
    
    async def _arm(self) -> bool:
        """Arm the vehicle."""
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(
            None,
            lambda: self.connection.arducopter_arm()
        )
        print("[MAVLink] ARM command sent")
        return True
    
    async def _disarm(self) -> bool:
        """Disarm the vehicle."""
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(
            None,
            lambda: self.connection.arducopter_disarm()
        )
        print("[MAVLink] DISARM command sent")
        return True
    
    async def _set_mode(self, mode_name: str) -> bool:
        """Set flight mode."""
        mode_number = self._get_mode_number(mode_name)
        if mode_number is None:
            print(f"[MAVLink] ERROR: Unknown mode: {mode_name}")
            return False
        
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(
            None,
            lambda: self.connection.set_mode(mode_number)
        )
        print(f"[MAVLink] Mode change to {mode_name} sent")
        return True
    
    async def _takeoff(self, altitude: float) -> bool:
        """Command takeoff to specified altitude."""
        await self._set_mode("QLOITER")
        await asyncio.sleep(0.5)
        
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(
            None,
            lambda: self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0, 0, 0, 0,
                0, 0,
                altitude
            )
        )
        print(f"[MAVLink] Takeoff command sent (target: {altitude}m)")
        return True
    
    def _get_mode_number(self, mode_name: str) -> Optional[int]:
        """Convert mode name to mode number for ArduPlane."""
        mode_map = {
            "MANUAL": 0, "CIRCLE": 1, "STABILIZE": 2, "TRAINING": 3,
            "ACRO": 4, "FBWA": 5, "FBWB": 6, "CRUISE": 7,
            "AUTOTUNE": 8, "AUTO": 10, "RTL": 11, "LOITER": 12,
            "TAKEOFF": 13, "AVOID_ADSB": 14, "GUIDED": 15,
            "QSTABILIZE": 17, "QHOVER": 18, "QLOITER": 19,
            "QLAND": 20, "QRTL": 21, "QAUTOTUNE": 22, "QACRO": 23,
        }
        return mode_map.get(mode_name)
    
    async def upload_mission(self, waypoints: List[Dict[str, Any]]) -> bool:
        """Upload mission waypoints to autopilot - FIXED VERSION!"""
        if not self.connection:
            print("[MAVLink] ERROR: Not connected")
            return False
        
        try:
            loop = asyncio.get_event_loop()
            
            # Clear existing mission
            print("[MAVLink] Clearing existing mission...")
            await loop.run_in_executor(
                None,
                lambda: self.connection.mav.mission_clear_all_send(
                    self.connection.target_system,
                    self.connection.target_component
                )
            )
            await asyncio.sleep(0.5)
            
            # Send mission count
            total_items = len(waypoints) + 1
            print(f"[MAVLink] Uploading mission with {total_items} waypoints...")
            
            await loop.run_in_executor(
                None,
                lambda: self.connection.mav.mission_count_send(
                    self.connection.target_system,
                    self.connection.target_component,
                    total_items
                )
            )
            
            # Upload home position + waypoints
            all_waypoints = [{"lat": 0, "lon": 0, "alt": 0}] + waypoints
            
            for i, wp in enumerate(all_waypoints):
                # FIXED: Direct attribute access on dict, not object
                loiter_time = wp.get("loiter_time", 0)
                acceptance_radius = wp.get("acceptance_radius", 10)
                
                await loop.run_in_executor(
                    None,
                    lambda idx=i, waypoint=wp, lt=loiter_time, ar=acceptance_radius: 
                        self.connection.mav.mission_item_send(
                            self.connection.target_system,
                            self.connection.target_component,
                            idx,
                            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                            1 if idx == 0 else 0,
                            1,
                            lt,  # param1: hold time
                            ar,  # param2: acceptance radius
                            0, 0,
                            waypoint["lat"],
                            waypoint["lon"],
                            waypoint["alt"]
                        )
                )
                await asyncio.sleep(0.1)
            
            # Update mission tracking state
            self.total_waypoints = len(waypoints)
            self.current_waypoint = 0
            
            print(f"[MAVLink] Mission uploaded successfully! ({total_items} items)")
            return True
            
        except Exception as e:
            print(f"[MAVLink] ERROR uploading mission: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    async def stop(self):
        """Stop telemetry and disconnect."""
        self.running = False
        if self.connection:
            self.connection.close()
            self.connection = None
        print("[MAVLink] Disconnected")
