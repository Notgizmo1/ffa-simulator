#!/usr/bin/env python3
import sys
import logging
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

import asyncio
import qasync
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import Qt
from ffa_simulator.gui.main_window import MainWindow
from ffa_simulator.utils.logger import setup_logging

def main():
    setup_logging()
    logger = logging.getLogger(__name__)
    logger.info("=" * 60)
    logger.info("FFA Simulator Starting...")
    logger.info("=" * 60)
    
    app = QApplication(sys.argv)
    app.setApplicationName("FFA Simulator")
    app.setOrganizationName("Forge & Flight Holdings, Inc.")
    
    loop = qasync.QEventLoop(app)
    asyncio.set_event_loop(loop)
    
    window = MainWindow()
    window.show()
    
    logger.info("Main window displayed")
    logger.info("Ready for simulation")
    
    with loop:
        loop.run_forever()

if __name__ == "__main__":
    main()