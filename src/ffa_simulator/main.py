#!/usr/bin/env python3
import sys
import logging
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

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
    
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps)
    
    app = QApplication(sys.argv)
    app.setApplicationName("FFA Simulator")
    app.setOrganizationName("Forge & Flight Holdings, Inc.")
    
    window = MainWindow()
    window.show()
    
    logger.info("Main window displayed")
    logger.info("Ready for simulation")
    
    sys.exit(app.exec())

if __name__ == "__main__":
    main()