"""
Process Manager for FFA Simulator.
Handles starting/stopping ArduPilot SITL in WSL2.
"""

import subprocess
import os
from PySide6.QtCore import QObject, Signal


class ProcessManager(QObject):
    """Manages SITL and related processes."""
    
    # Signals
    process_started = Signal(str)  # process_name
    process_stopped = Signal(str)  # process_name
    log_message = Signal(str)      # message
    
    def __init__(self):
        super().__init__()
        self.sitl_process = None
        self.wsl_ip = None
        self.windows_host_ip = None
    
    def start_sitl(self, vehicle: str = "QuadPlane") -> bool:
        """
        Start ArduPilot SITL in WSL2.
        
        Args:
            vehicle: Vehicle type (QuadPlane, Copter, Plane, etc.)
            
        Returns:
            True if started successfully, False otherwise
        """
        try:
            self.log_message.emit(f"Starting {vehicle} SITL in WSL2...")
            
            # Get WSL2 IP address
            self.wsl_ip = self._get_wsl_ip()
            if not self.wsl_ip:
                self.log_message.emit("ERROR: Could not get WSL2 IP address")
                self.log_message.emit("Is WSL2 running? Try: wsl --list --running")
                return False
            
            self.log_message.emit(f"WSL2 IP: {self.wsl_ip}")
            
            # Get Windows host IP for UDP output
            self.windows_host_ip = self._get_windows_host_ip()
            
            # Build SITL command
            sitl_cmd = self._build_sitl_command(vehicle)
            
            # Log the command (truncated for readability)
            cmd_preview = sitl_cmd[:100] + "..." if len(sitl_cmd) > 100 else sitl_cmd
            self.log_message.emit(f"Command: {cmd_preview}")
            
            # Start SITL in WSL2 (non-blocking)
            self.log_message.emit("Starting SITL process...")
            
            # Use Popen for non-blocking subprocess
            self.sitl_process = subprocess.Popen(
                sitl_cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            self.log_message.emit(f"SITL process started (PID: {self.sitl_process.pid})")
            self.log_message.emit("Waiting for SITL to initialize (takes ~15-20 seconds)...")
            self.log_message.emit(f"MAVLink will broadcast UDP to Windows host")
            self.log_message.emit(f"GUI will listen on udpin:0.0.0.0:14550")
            self.log_message.emit("Monitoring SITL output...")
            
            # Start monitoring SITL output in background
            import threading
            self._output_thread = threading.Thread(target=self._monitor_sitl_output, daemon=True)
            self._output_thread.start()
            
            self.process_started.emit("SITL")
            return True
            
        except Exception as e:
            self.log_message.emit(f"ERROR starting SITL: {e}")
            import traceback
            self.log_message.emit(f"Traceback: {traceback.format_exc()}")
            return False
    
    def stop_sitl(self) -> bool:
        """
        Stop SITL process.
        
        Returns:
            True if stopped successfully, False otherwise
        """
        try:
            if self.sitl_process:
                self.log_message.emit("Stopping SITL...")
                
                # Terminate process
                self.sitl_process.terminate()
                
                # Wait for termination (with timeout)
                try:
                    self.sitl_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.log_message.emit("SITL didn't stop gracefully, killing...")
                    self.sitl_process.kill()
                
                self.sitl_process = None
                self.log_message.emit("SITL stopped")
                self.process_stopped.emit("SITL")
                return True
            else:
                self.log_message.emit("No SITL process to stop")
                return False
                
        except Exception as e:
            self.log_message.emit(f"ERROR stopping SITL: {e}")
            return False
    
    def _get_wsl_ip(self) -> str:
        """
        Get WSL2 IP address.
        
        Returns:
            IP address string or None if failed
        """
        try:
            # Get WSL2 IP using hostname -I
            result = subprocess.run(
                ["wsl", "hostname", "-I"],
                capture_output=True,
                text=True,
                timeout=5
            )
            
            if result.returncode == 0:
                # Parse first IP address
                ip_addresses = result.stdout.strip().split()
                if ip_addresses:
                    return ip_addresses[0]
            
            return None
            
        except Exception as e:
            self.log_message.emit(f"ERROR getting WSL2 IP: {e}")
            return None
    
    def _get_windows_host_ip(self) -> str:
        """
        Get Windows host IP address as seen from WSL2.
        This is where MAVProxy should send UDP packets.
        
        Returns:
            Windows host IP address (typically the gateway)
        """
        try:
            # Get default gateway from WSL (this is the Windows host)
            result = subprocess.run(
                ["wsl", "ip", "route", "show", "default"],
                capture_output=True,
                text=True,
                timeout=5
            )
            
            if result.returncode == 0:
                # Parse gateway IP from output like: "default via 172.24.119.1 dev eth0"
                parts = result.stdout.strip().split()
                if len(parts) >= 3 and parts[0] == "default" and parts[1] == "via":
                    host_ip = parts[2]
                    self.log_message.emit(f"Windows host IP: {host_ip}")
                    return host_ip
            
            # Fallback: calculate based on WSL IP
            if self.wsl_ip:
                # Windows host is typically .1 on the same subnet
                ip_parts = self.wsl_ip.split('.')
                ip_parts[-1] = '1'
                host_ip = '.'.join(ip_parts)
                self.log_message.emit(f"Windows host IP (calculated): {host_ip}")
                return host_ip
                
            return "172.24.119.1"  # Common default
            
        except Exception as e:
            self.log_message.emit(f"ERROR getting Windows host IP: {e}")
            return "172.24.119.1"  # Fallback
    
    def _build_sitl_command(self, vehicle: str) -> str:
        """
        Build SITL command for WSL2.
        
        Args:
            vehicle: Vehicle type
            
        Returns:
            Command string
        """
        # Determine vehicle directory
        vehicle_map = {
            "QuadPlane": "ArduPlane",
            "Copter": "ArduCopter",
            "Plane": "ArduPlane",
            "Rover": "ArduRover"
        }
        
        vehicle_dir = vehicle_map.get(vehicle, "ArduPlane")
        
        # Build command
        # Use UDP output to Windows host IP (dynamically detected)
        # UDP is connectionless and more reliable for MAVLink over WSL2
        cmd = (
            f'wsl bash -c "'
            f'cd ~/ardupilot/{vehicle_dir} && '
            f'sim_vehicle.py '
            f'-v {vehicle_dir} '
            f'-I0 '
            f'--no-rebuild '
            f'--custom-location=35.362938,-97.928917,585,0 '
            f'--out=udpout:{self.windows_host_ip}:14550"'
        )
        
        return cmd
    
    def get_mavlink_connection_string(self) -> str:
        """
        Get MAVLink connection string.
        
        Returns:
            Connection string for UDP listener
        """
        # Listen on Windows host for UDP packets from WSL
        # MAVProxy sends to 172.24.119.1:14550, we listen on that port
        return "udpin:0.0.0.0:14550"
    
    def _monitor_sitl_output(self):
        """
        Monitor SITL stdout/stderr and log important messages.
        Runs in background thread.
        """
        if not self.sitl_process:
            return
            
        try:
            # Read stdout line by line
            for line in iter(self.sitl_process.stdout.readline, ''):
                if not line:
                    break
                    
                line = line.strip()
                
                # Log important messages
                if any(keyword in line.lower() for keyword in [
                    'mavlink', 'listening', 'port', 'connected', 
                    'ready', 'initialized', 'ekf', 'gps', 'error', 'failed'
                ]):
                    self.log_message.emit(f"[SITL] {line}")
                    
        except Exception as e:
            self.log_message.emit(f"[SITL Monitor] Error: {e}")
