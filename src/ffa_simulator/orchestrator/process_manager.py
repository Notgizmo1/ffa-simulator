import logging

logger = logging.getLogger(__name__)

class ProcessManager:
    def __init__(self):
        logger.info("ProcessManager initialized (stub)")
        
    async def start_simulation(self, vehicle_type="quadplane"):
        logger.info(f"Starting simulation with vehicle: {vehicle_type}")
        pass
        
    async def stop_simulation(self):
        logger.info("Stopping simulation")
        pass