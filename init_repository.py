# FFA Simulator - Repository Initialization Script
# Run this after completing environment setup

import os
import sys
from pathlib import Path

def create_directory_structure():
    """Create the complete FFA Simulator directory structure."""
    
    directories = [
        "src/ffa_simulator",
        "src/ffa_simulator/orchestrator",
        "src/ffa_simulator/telemetry",
        "src/ffa_simulator/gui",
        "src/ffa_simulator/gui/widgets",
        "src/ffa_simulator/scenarios",
        "src/ffa_simulator/scenarios/builtin",
        "src/ffa_simulator/training",
        "src/ffa_simulator/utils",
        "src/tests",
        "docker",
        "vehicles/generic_quadplane",
        "vehicles/aether",
        "vehicles/vanguard",
        "worlds",
        "scenarios/beginner",
        "scenarios/intermediate",
        "scenarios/advanced",
        "docs/getting-started",
        "docs/tutorials/beginner",
        "docs/tutorials/intermediate",
        "docs/tutorials/advanced",
        "docs/reference",
        "docs/instructor-guide",
        ".github/ISSUE_TEMPLATE",
        ".github/workflows",
        "installers/windows",
        "installers/linux",
        "installers/macos",
        "scripts",
        "resources/icons",
        "resources/images",
    ]
    
    for directory in directories:
        Path(directory).mkdir(parents=True, exist_ok=True)
        print(f"✓ Created: {directory}")
    
    # Create __init__.py files for Python packages
    python_packages = [
        "src/ffa_simulator",
        "src/ffa_simulator/orchestrator",
        "src/ffa_simulator/telemetry",
        "src/ffa_simulator/gui",
        "src/ffa_simulator/gui/widgets",
        "src/ffa_simulator/scenarios",
        "src/ffa_simulator/scenarios/builtin",
        "src/ffa_simulator/training",
        "src/ffa_simulator/utils",
        "src/tests",
    ]
    
    for package in python_packages:
        init_file = Path(package) / "__init__.py"
        init_file.touch()
        print(f"✓ Created: {init_file}")

def create_gitignore():
    """Create comprehensive .gitignore file."""
    
    gitignore_content = """# Python
__pycache__/
*.py[cod]
*$py.class
*.so
.Python
venv/
env/
ENV/
build/
develop-eggs/
dist/
downloads/
eggs/
.eggs/
lib/
lib64/
parts/
sdist/
var/
wheels/
*.egg-info/
.installed.cfg
*.egg

# IDEs
.vscode/
.idea/
*.swp
*.swo
*~
.DS_Store

# Testing
.pytest_cache/
.coverage
htmlcov/
.tox/

# Logs
*.log
logs/

# Docker
*.tar

# Application specific
*.qmlc
*.jsc
.qmake.stash

# Temporary files
*.tmp
*.temp
temp/
tmp/

# OS
Thumbs.db
.DS_Store
"""
    
    with open(".gitignore", "w") as f:
        f.write(gitignore_content)
    print("✓ Created: .gitignore")

def create_license():
    """Create Apache 2.0 license file."""
    
    license_content = """                                 Apache License
                           Version 2.0, January 2004
                        http://www.apache.org/licenses/

   Copyright 2026 Forge & Flight Holdings, Inc.

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
"""
    
    with open("LICENSE", "w") as f:
        f.write(license_content)
    print("✓ Created: LICENSE")

def create_readme():
    """Create main README.md file."""
    
    readme_content = """# FFA Simulator

**Open-Source Educational Drone Simulation Platform for ArduPilot VTOL Training**

![License](https://img.shields.io/badge/license-Apache%202.0-blue.svg)
![Python](https://img.shields.io/badge/python-3.11-blue.svg)
![Platform](https://img.shields.io/badge/platform-Windows%20%7C%20Linux%20%7C%20macOS-lightgrey.svg)

FFA Simulator is a comprehensive training platform for learning unmanned aircraft systems (UAS) operation, with a focus on VTOL (Vertical Takeoff and Landing) platforms. Built by [Forge & Flight Holdings](https://forgeandflight.com) to democratize drone education.

## 🎯 Key Features

- **One-Click Launch**: Unified GUI orchestrates Gazebo + ArduPilot SITL + Mission Planner
- **Progressive Learning**: Beginner, Intermediate, and Advanced modes
- **Real-Time Visualization**: Live telemetry, servo outputs, VTOL transition states
- **VTOL-Focused**: Specialized training for quadplane and tiltrotor configurations
- **Built-In Scenarios**: Pre-configured training missions from hover to complex transitions
- **Cross-Platform**: Windows, Linux, and macOS support

## 🚀 Quick Start (5 Minutes to First Flight)

### Windows 10/11

```powershell
# Run automated setup (as Administrator)
irm https://ffa-simulator.org/setup-windows.ps1 | iex

# Clone repository
cd $HOME
git clone https://github.com/forgeandflight/ffa-simulator.git
cd ffa-simulator

# Launch simulator
.\\venv\\Scripts\\Activate.ps1
python src/ffa_simulator/main.py
```

### Linux (Ubuntu 22.04/24.04)

```bash
# One-command installation
curl -sSL https://ffa-simulator.org/install.sh | bash

# Clone repository
cd ~
git clone https://github.com/forgeandflight/ffa-simulator.git
cd ffa-simulator

# Launch simulator
source venv/bin/activate
python src/ffa_simulator/main.py
```

### macOS

```bash
# Homebrew installation
brew tap ffa-simulator/tap
brew install --cask ffa-simulator

# Or manual setup
curl -sSL https://ffa-simulator.org/install-macos.sh | bash
```

## 📖 Documentation

- [Installation Guide](docs/getting-started/installation.md)
- [Your First Flight](docs/tutorials/beginner/01-your-first-flight.md)
- [VTOL Transitions](docs/tutorials/intermediate/01-vtol-transitions.md)
- [Instructor Guide](docs/instructor-guide/classroom-setup.md)

## 🎓 Who Is This For?

- **University Students**: Aerospace engineering, robotics, geospatial science programs
- **Drone Builders**: Hobbyists building custom UAS platforms
- **Teenagers**: Summer programs teaching drone technology
- **Professionals**: Forge & Flight customers training on Aether, Vanguard, Titan platforms

## 🏗️ Architecture

FFA Simulator uses a three-tier architecture:
- **Python Orchestrator** (PySide6): Unified GUI managing all processes
- **Docker Containers**: Gazebo Harmonic + ArduPilot SITL
- **MAVLink Bridge** (pymavlink): Real-time telemetry streaming

## 🤝 Contributing

We welcome contributions! See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

Areas we need help:
- Additional vehicle models (Group 1-3 UAS)
- Training scenarios and curriculum development
- Platform-specific installers and testing
- Documentation and tutorials

## 📝 License

Apache License 2.0 - see [LICENSE](LICENSE) for details.

Commercial use permitted with proper attribution.

## 🙏 Acknowledgments

Built on the shoulders of giants:
- [ArduPilot](https://ardupilot.org/) - Open-source autopilot software
- [Gazebo](https://gazebosim.org/) - 3D robotics simulator
- [PX4](https://px4.io/) - Flight control inspiration
- [QGroundControl](http://qgroundcontrol.com/) - GCS design patterns

## 📧 Contact

- **Website**: [ffa-simulator.org](https://ffa-simulator.org)
- **Email**: support@forgeandflight.com
- **Issues**: [GitHub Issues](https://github.com/forgeandflight/ffa-simulator/issues)

---

**Built with ❤️ in North Carolina** - *First in Flight, Innovation Limitless*
"""
    
    with open("README.md", "w") as f:
        f.write(readme_content)
    print("✓ Created: README.md")

def create_requirements_txt():
    """Create requirements.txt for pip installation."""
    
    requirements = """# Core GUI Framework
PySide6==6.6.1
PyQt6==6.6.1

# ArduPilot MAVLink Communication
pymavlink==2.4.41

# Real-time Plotting
pyqtgraph==0.13.3

# Scientific Computing
numpy==1.26.3

# System Monitoring
psutil==5.9.7

# Configuration
pyyaml==6.0.1

# Async Support
asyncio==3.4.3

# 3D Visualization
pyvista==0.43.1

# Testing
pytest==7.4.4
pytest-asyncio==0.23.3
pytest-qt==4.3.1

# Documentation
mkdocs==1.5.3
mkdocs-material==9.5.3
"""
    
    with open("requirements.txt", "w") as f:
        f.write(requirements)
    print("✓ Created: requirements.txt")

def create_setup_py():
    """Create setup.py for pip installation."""
    
    setup_content = """from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="ffa-simulator",
    version="0.1.0",
    author="Forge & Flight Holdings, Inc.",
    author_email="support@forgeandflight.com",
    description="Open-source educational drone simulation platform for ArduPilot VTOL training",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/forgeandflight/ffa-simulator",
    package_dir={"": "src"},
    packages=find_packages(where="src"),
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Education",
        "Intended Audience :: Science/Research",
        "Topic :: Education",
        "Topic :: Scientific/Engineering",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.11",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.11",
    install_requires=[
        "PySide6>=6.6.1",
        "pymavlink>=2.4.41",
        "pyqtgraph>=0.13.3",
        "numpy>=1.26.3",
        "psutil>=5.9.7",
        "pyyaml>=6.0.1",
    ],
    entry_points={
        "console_scripts": [
            "ffa-simulator=ffa_simulator.main:main",
        ],
    },
)
"""
    
    with open("setup.py", "w") as f:
        f.write(setup_content)
    print("✓ Created: setup.py")

def main():
    print("=" * 60)
    print("FFA Simulator - Repository Initialization")
    print("=" * 60)
    print()
    
    # Check if we're in a git repository
    if Path(".git").exists():
        print("⚠ Warning: .git directory already exists")
        response = input("Continue anyway? (y/n): ")
        if response.lower() != 'y':
            print("Aborted.")
            sys.exit(0)
    
    print("Creating directory structure...")
    create_directory_structure()
    
    print("\nCreating configuration files...")
    create_gitignore()
    create_license()
    create_readme()
    create_requirements_txt()
    create_setup_py()
    
    print("\n" + "=" * 60)
    print("✓ Repository structure created successfully!")
    print("=" * 60)
    print("\nNext steps:")
    print("1. Initialize git repository: git init")
    print("2. Create initial commit: git add . && git commit -m 'Initial commit'")
    print("3. Create GitHub repository")
    print("4. Push to GitHub: git remote add origin <url> && git push -u origin main")
    print("\nThen proceed to building the core application components.")

if __name__ == "__main__":
    main()
