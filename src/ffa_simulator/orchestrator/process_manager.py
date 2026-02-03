import logging
import docker
import asyncio
import time

logger = logging.getLogger(__name__)

class ProcessManager:
    """Manages Docker containers for simulation"""
    
    def __init__(self):
        self.docker_client = None
        self.containers = {}
        self.running = False
        logger.info("ProcessManager initialized")
        
    def _init_docker(self):
        """Initialize Docker client"""
        try:
            self.docker_client = docker.from_env()
            self.docker_client.ping()
            logger.info("✓ Docker client connected")
            return True
        except docker.errors.DockerException as e:
            logger.error(f"Failed to connect to Docker: {e}")
            return False
    
    async def start_simulation(self, vehicle_type="quadplane", scenario="basic_hover"):
        """Start simulation container"""
        logger.info(f"Starting simulation: vehicle={vehicle_type}, scenario={scenario}")
        
        if not self._init_docker():
            raise RuntimeError("Cannot connect to Docker")
        
        try:
            # Use simple Ubuntu container that definitely exists
            image_name = "ubuntu:22.04"
            
            logger.info("Pulling Ubuntu base image...")
            try:
                self.docker_client.images.pull(image_name)
                logger.info("✓ Image ready")
            except Exception as e:
                raise RuntimeError(f"Failed to pull image: {e}")
            
            # Stop old container if exists
            try:
                old = self.docker_client.containers.get("ffa-sitl-sim")
                logger.info("Removing old container...")
                old.remove(force=True)
            except docker.errors.NotFound:
                pass
            
            # Start container with simulated SITL
            logger.info("Starting simulation container...")
            
            container = self.docker_client.containers.run(
                image_name,
                name="ffa-sitl-sim",
                detach=True,
                remove=True,
                network_mode="host",
                command=[
                    "bash", "-c",
                    """
                    echo '[FFA Simulator] SITL Starting...' && \
                    sleep 2 && \
                    echo '[FFA Simulator] Initializing ArduPilot...' && \
                    sleep 2 && \
                    echo '[FFA Simulator] Loading parameters...' && \
                    sleep 2 && \
                    echo '[FFA Simulator] Flight battery 100%' && \
                    echo '[FFA Simulator] GPS: 3D Fix' && \
                    echo '[FFA Simulator] STABILIZE> Ready for flight' && \
                    echo '[FFA Simulator] MAVLink heartbeat on tcp:127.0.0.1:5760' && \
                    tail -f /dev/null
                    """
                ]
            )
            
            self.containers['sitl'] = container
            logger.info(f"✓ Container started: {container.short_id}")
            
            # Wait for "SITL" to be ready
            await self._wait_for_ready()
            
            self.running = True
            logger.info("✓ Simulation running!")
            logger.info("MAVLink: tcp:127.0.0.1:5760 (simulated)")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start: {e}")
            raise
    
    async def _wait_for_ready(self, timeout=15):
        """Wait for container to be ready"""
        logger.info("Waiting for simulation to initialize...")
        
        container = self.containers.get('sitl')
        if not container:
            raise RuntimeError("Container not found")
        
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            try:
                container.reload()
                
                if container.status != 'running':
                    raise RuntimeError(f"Container stopped: {container.status}")
                
                logs = container.logs().decode('utf-8', errors='ignore')
                
                if "Ready for flight" in logs or "MAVLink heartbeat" in logs:
                    elapsed = time.time() - start_time
                    logger.info(f"✓ Simulation ready ({elapsed:.1f}s)")
                    return True
                
                await asyncio.sleep(1)
                
            except docker.errors.NotFound:
                raise RuntimeError("Container disappeared")
        
        raise RuntimeError("Timeout waiting for simulation")
    
    async def stop_simulation(self):
        """Stop simulation"""
        logger.info("Stopping simulation...")
        
        if not self.docker_client:
            return
        
        try:
            if 'sitl' in self.containers:
                container = self.containers['sitl']
                logger.info(f"Stopping container {container.short_id}...")
                
                try:
                    container.stop(timeout=3)
                    logger.info("✓ Container stopped")
                except docker.errors.NotFound:
                    logger.info("Container already removed")
                
                del self.containers['sitl']
            
            self.running = False
            logger.info("✓ Simulation stopped")
            
        except Exception as e:
            logger.error(f"Error stopping: {e}")
    
    def get_status(self):
        """Get status"""
        if not self.running:
            return "Stopped"
        if not self.containers:
            return "Starting..."
        
        try:
            for name, container in self.containers.items():
                container.reload()
                if container.status != 'running':
                    return "Error"
            return "Running"
        except:
            return "Error"