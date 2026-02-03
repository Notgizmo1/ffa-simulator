import logging

logger = logging.getLogger(__name__)

class TelemetryBridge:
    def __init__(self, connection_string='tcp:127.0.0.1:5760'):
        self.connection_string = connection_string
        logger.info(f"TelemetryBridge initialized (stub) - {connection_string}")
        
    def connect(self):
        logger.info("Connecting to MAVLink...")
        pass
        
    def disconnect(self):
        logger.info("Disconnecting from MAVLink...")
        pass