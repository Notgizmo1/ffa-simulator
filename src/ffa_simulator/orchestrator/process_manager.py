import logging
import asyncio
import sys

logger = logging.getLogger(__name__)

class ProcessManager:
    """Manages ArduPilot SITL simulation processes"""
    
    def __init__(self):
        self.processes = {}
        self.running = False
        self.log_task = None
        self.sitl_built = False
        logger.info("ProcessManager initialized (WSL2 native ArduPilot)")
        
    async def _ensure_sitl_built(self):
        """Ensure ArduPilot SITL is built"""
        logger.info("Checking if ArduPilot SITL is built...")
        
        check_cmd = [
            "wsl", "-e", "bash", "-c",
            "test -f ~/ardupilot/build/sitl/bin/arduplane && echo 'EXISTS' || echo 'MISSING'"
        ]
        
        proc = await asyncio.create_subprocess_exec(
            *check_cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE
        )
        
        stdout, _ = await proc.communicate()
        result = stdout.decode('utf-8', errors='ignore').strip()
        
        if "EXISTS" in result:
            logger.info("✓ ArduPilot SITL already built")
            self.sitl_built = True
            return True
        
        logger.info("ArduPilot SITL not built, building now (5-10 minutes)...")
        
        build_cmd = [
            "wsl", "-e", "bash", "-c",
            "cd ~/ardupilot && ./waf configure --board sitl && ./waf plane"
        ]
        
        build_proc = await asyncio.create_subprocess_exec(
            *build_cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT
        )
        
        while True:
            line = await build_proc.stdout.readline()
            if not line:
                break
            
            text = line.decode('utf-8', errors='ignore').strip()
            if text and any(key in text for key in ['Configuring', 'Build finished', 'Linking']):
                logger.info(f"[BUILD] {text}")
        
        await build_proc.wait()
        
        if build_proc.returncode == 0:
            logger.info("✓ ArduPilot built successfully")
            self.sitl_built = True
            return True
        else:
            raise RuntimeError(f"Build failed (code: {build_proc.returncode})")
    
    async def start_simulation(self, vehicle_type="quadplane", scenario="basic_hover"):
        """Start ArduPilot SITL simulation"""
        logger.info(f"Starting simulation: vehicle={vehicle_type}, scenario={scenario}")
        
        try:
            if not self.sitl_built:
                await self._ensure_sitl_built()
            
            vehicle_map = {
                "quadplane": "quadplane",
                "aether_vtol": "quadplane",
                "vanguard_vtol": "quadplane"
            }
            
            frame = vehicle_map.get(vehicle_type, "quadplane")
            
            # Simplified SITL command that works reliably
            # SITL command WITHOUT MAVProxy (we'll connect via pymavlink directly)
            sitl_cmd = [
                "wsl", "-e", "bash", "-c",
                f"cd ~/ardupilot && python3 ./Tools/autotest/sim_vehicle.py "
                f"-v ArduPlane "
                f"-f {frame} "
                f"--no-rebuild "
                f"--no-mavproxy "
                f"--speedup 1 "
                f"-I0 "
                f"2>&1"
            ]
            
            logger.info("Launching ArduPilot SITL...")
            logger.info(f"Vehicle: ArduPlane, Frame: {frame}")
            
            # Start process
            self.processes['sitl'] = await asyncio.create_subprocess_exec(
                *sitl_cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT,
                stdin=asyncio.subprocess.PIPE
            )
            
            logger.info(f"✓ SITL process started (PID: {self.processes['sitl'].pid})")
            
            # Start log monitoring
            self.log_task = asyncio.create_task(self._monitor_logs())
            
            # Wait for ready
            await self._wait_for_ready()
            
            self.running = True
            logger.info("=" * 60)
            logger.info("✓ SIMULATION READY")
            logger.info("MAVLink: tcp:127.0.0.1:5760")
            logger.info("=" * 60)
            return True
            
        except Exception as e:
            logger.error(f"Failed to start: {e}")
            await self.stop_simulation()
            raise
    
    async def _monitor_logs(self):
        """Monitor SITL output"""
        proc = self.processes.get('sitl')
        if not proc:
            return
        
        try:
            while True:
                line = await proc.stdout.readline()
                if not line:
                    break
                
                # Decode with error handling for special characters
                text = line.decode('utf-8', errors='replace').strip()
                
                if text:
                    # Filter for important messages only
                    important_keywords = [
                        'APM:', 'EKF', 'PreArm', 'ARMED', 'DISARMED',
                        'Flight battery', 'GPS', 'Ready to FLY',
                        'Init', 'STABILIZE', 'Port'
                    ]
                    
                    if any(key in text for key in important_keywords):
                        # Remove special characters that cause encoding issues
                        clean_text = ''.join(c if ord(c) < 128 else ' ' for c in text)
                        logger.info(f"[SITL] {clean_text}")
                    
        except asyncio.CancelledError:
            pass
        except Exception as e:
            logger.error(f"Error monitoring logs: {e}")
    
    async def _wait_for_ready(self, timeout=45):
        """Wait for SITL to initialize"""
        logger.info("Waiting for SITL initialization (20-30 seconds)...")
        
        proc = self.processes.get('sitl')
        if not proc:
            raise RuntimeError("SITL process not found")
        
        start_time = asyncio.get_event_loop().time()
        ready_found = False
        
        while asyncio.get_event_loop().time() - start_time < timeout:
            # Check if process died
            if proc.returncode is not None:
                # Get last output for debugging
                try:
                    remaining = await proc.stdout.read()
                    error_output = remaining.decode('utf-8', errors='replace')[:500]
                    logger.error(f"SITL died. Last output: {error_output}")
                except:
                    pass
                raise RuntimeError(f"SITL died with code {proc.returncode}")
            
            await asyncio.sleep(2)
        
        # After timeout, check if still alive
        if proc.returncode is not None:
            raise RuntimeError("SITL failed to start")
        
        elapsed = asyncio.get_event_loop().time() - start_time
        logger.info(f"✓ SITL initialized ({elapsed:.1f}s)")
        return True
    
    async def stop_simulation(self):
        """Stop SITL simulation"""
        logger.info("Stopping simulation...")
        
        try:
            if self.log_task:
                self.log_task.cancel()
                try:
                    await self.log_task
                except asyncio.CancelledError:
                    pass
                self.log_task = None
            
            if 'sitl' in self.processes:
                proc = self.processes['sitl']
                logger.info(f"Terminating SITL (PID: {proc.pid})...")
                
                try:
                    proc.terminate()
                    
                    try:
                        await asyncio.wait_for(proc.wait(), timeout=5)
                        logger.info("✓ SITL terminated")
                    except asyncio.TimeoutError:
                        logger.warning("Force killing SITL...")
                        proc.kill()
                        await proc.wait()
                        logger.info("✓ SITL killed")
                        
                except ProcessLookupError:
                    logger.info("Process already gone")
                
                del self.processes['sitl']
            
            self.running = False
            logger.info("✓ Simulation stopped")
            
        except Exception as e:
            logger.error(f"Error stopping: {e}")
    
    def get_status(self):
        """Get simulation status"""
        if not self.running:
            return "Stopped"
        if not self.processes:
            return "Starting..."
        
        if 'sitl' in self.processes:
            proc = self.processes['sitl']
            if proc.returncode is not None:
                return "Error: Crashed"
        
        return "Running"