# WSL2 Setup Guide for FFA Simulator

## Prerequisites

### Windows Requirements
- Windows 11 (or Windows 10 with WSL2 enabled)
- WSL2 with Ubuntu 24.04 or compatible Linux distribution
- Python 3.10+ installed on both Windows and WSL2

### Required Python Packages (Windows)
```powershell
pip install pymavlink PyQt6 matplotlib numpy
```

### Required Python Packages (WSL2)
```bash
pip3 install pymavlink MAVProxy --break-system-packages
```

## Critical Setup: MAVProxy Wrapper Script

**Problem:** ArduPilot's `sim_vehicle.py` expects `mavproxy.py` to be available as a shell command, but when installed via pip, MAVProxy is only accessible as a Python module.

**Solution:** Create a wrapper script that makes MAVProxy callable as a command.

### Installation Steps

**1. Create the wrapper script:**
```bash
wsl bash -c "cat > ~/.local/bin/mavproxy.py << 'EOF'
#!/bin/bash
exec python3 -m MAVProxy.mavproxy \"\$@\"
EOF"
```

**2. Make it executable:**
```bash
wsl bash -c "chmod +x ~/.local/bin/mavproxy.py"
```

**3. Copy to system-wide location (requires sudo):**
```bash
wsl bash -c "sudo cp ~/.local/bin/mavproxy.py /usr/local/bin/mavproxy.py"
wsl bash -c "sudo chmod +x /usr/local/bin/mavproxy.py"
```

**4. Verify installation:**
```bash
wsl bash -c "which mavproxy.py"
# Should output: /usr/local/bin/mavproxy.py

wsl bash -c "mavproxy.py --help"
# Should show MAVProxy help text
```

## ArduPilot Installation (WSL2)

**1. Clone ArduPilot repository:**
```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
```

**2. Install ArduPilot dependencies:**
```bash
cd ~/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

**3. Build ArduPlane SITL:**
```bash
cd ~/ardupilot/ArduPlane
./waf configure --board sitl
./waf plane
```

**4. Verify SITL can start:**
```bash
cd ~/ardupilot
python3 ./Tools/autotest/sim_vehicle.py -v ArduPlane -f quadplane --no-rebuild -I0
```

You should see:
- "Starting ArduPlane"
- "Run MAVProxy"
- MAVProxy output showing UDP broadcast to Windows

Press Ctrl+C to stop when verified.

## Network Configuration

### Understanding the Connection Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                         WSL2 Ubuntu                          │
│                                                              │
│  ┌──────────┐      ┌──────────┐      ┌─────────────────┐   │
│  │   SITL   │◄────►│ MAVProxy │─────►│ UDP Broadcast   │   │
│  │ (5501)   │ TCP  │ (5760)   │      │ to Windows IP   │   │
│  └──────────┘      └──────────┘      └─────────────────┘   │
│                                              │               │
└──────────────────────────────────────────────┼──────────────┘
                                               │
                                         UDP 14550
                                               │
                                               ▼
                                    ┌──────────────────┐
                                    │  Windows Client  │
                                    │  FFA Simulator   │
                                    │ (0.0.0.0:14550)  │
                                    └──────────────────┘
```

### Key Points

1. **SITL runs on localhost (127.0.0.1) inside WSL2**
2. **MAVProxy connects to SITL via TCP on port 5760**
3. **MAVProxy broadcasts UDP packets to Windows on port 14550**
4. **Windows GUI listens on UDP 14550 for all interfaces (0.0.0.0)**

### Connection String

The FFA Simulator uses:
```python
connection_string = "udpin:0.0.0.0:14550"
```

**NOT:**
- ~~`tcp:172.24.119.69:5760`~~ (direct SITL connection - doesn't work)
- ~~`tcp:127.0.0.1:5760`~~ (localhost - can't reach from Windows)
- ~~`udpout:172.24.112.1:14550`~~ (wrong direction)

## Troubleshooting

### WSL2 IP Address Changes

**Problem:** WSL2 IP address changes on every Windows restart.

**Solution:** The simulator doesn't need the WSL2 IP! MAVProxy automatically broadcasts to the Windows host IP. Just listen on `udpin:0.0.0.0:14550`.

**To find current WSL2 IP (if needed for debugging):**
```bash
wsl bash -c "hostname -I | awk '{print \$1}'"
```

### MAVProxy Not Found Error

**Error:**
```
[Errno 2] No such file or directory: 'mavproxy.py'
```

**Solution:** The wrapper script isn't in PATH. Verify:
```bash
wsl bash -c "which mavproxy.py"
```

Should return `/usr/local/bin/mavproxy.py`. If not, re-run the wrapper script installation steps above.

### No Heartbeat Received

**Symptoms:**
- GUI shows "Waiting for heartbeat..."
- Status stays yellow or red

**Diagnosis:**
```powershell
# Verify SITL is running
wsl bash -c "ps aux | grep arduplane"

# Check if MAVProxy is broadcasting
wsl bash -c "ss -ulnp | grep 14550"

# Test UDP connection manually
python
>>> from pymavlink import mavutil
>>> m = mavutil.mavlink_connection('udpin:0.0.0.0:14550', timeout=10)
>>> m.wait_heartbeat(timeout=30)
```

If manual test works but GUI doesn't, check the GUI connection string in `main_window.py`.

### SITL Crashes Immediately

**Error:**
```
SITL died but no output captured
```

**Common causes:**
1. Invalid SITL parameters (check `process_manager.py` doesn't have custom `--master` or port args)
2. Missing ArduPilot build (run `./waf plane` in ArduPlane directory)
3. Corrupted parameters (delete `~/ardupilot/Tools/autotest/default_params/*.parm` and retry)

**Debug by running SITL manually:**
```bash
cd ~/ardupilot
python3 ./Tools/autotest/sim_vehicle.py -v ArduPlane -f quadplane --no-rebuild -I0
```

Watch for specific error messages.

### Port Already in Use

**Error:**
```
Address already in use
```

**Solution:** Kill existing SITL/MAVProxy processes:
```bash
wsl bash -c "pkill -f arduplane"
wsl bash -c "pkill -f mavproxy"
```

Then restart the simulator.

## Verified Working Configuration

**Date:** February 4, 2026  
**Tested on:**
- Windows 11
- WSL2 Ubuntu 24.04
- ArduPilot master branch (commit 2ef657ed258a4385a099c704a3a1e9ad)
- Python 3.10
- pymavlink 2.4.14
- MAVProxy 1.8.74

**Test results:**
✅ SITL startup  
✅ MAVProxy UDP broadcast  
✅ Heartbeat reception  
✅ Telemetry streaming  
✅ Mission upload  
✅ AUTO mode execution  
✅ Real-time telemetry plots  

## Architecture Notes

### Why UDP Instead of TCP?

**TCP connection attempts failed because:**
1. SITL binds to `127.0.0.1:5760` (localhost only)
2. Windows can't directly connect to WSL2 localhost
3. WSL2 networking is NATed, not bridged
4. Direct TCP connection bypasses MAVProxy's protocol handling

**UDP broadcast works because:**
1. MAVProxy explicitly broadcasts to Windows host IP
2. UDP packets can cross WSL2 NAT boundary
3. `udpin` mode listens for packets from any source
4. MAVProxy handles all protocol details correctly

### Why Not --no-mavproxy?

The `--no-mavproxy` flag was attempted but failed because:
1. SITL alone doesn't broadcast to Windows automatically
2. Direct SITL connection requires complex network bridging
3. MAVProxy provides essential protocol translation
4. Default sim_vehicle.py behavior is designed for MAVProxy

**Lesson learned:** Don't fight ArduPilot's default architecture. Use MAVProxy as designed.

## Quick Reference

**Start SITL manually (for testing):**
```bash
wsl bash -c "cd ~/ardupilot && python3 ./Tools/autotest/sim_vehicle.py -v ArduPlane -f quadplane --no-rebuild -I0"
```

**Test UDP connection:**
```powershell
python
>>> from pymavlink import mavutil
>>> m = mavutil.mavlink_connection('udpin:0.0.0.0:14550', timeout=10)
>>> m.wait_heartbeat(timeout=30)
>>> print(f"Connected to System {m.target_system}")
```

**Stop all SITL processes:**
```bash
wsl bash -c "pkill -f arduplane; pkill -f mavproxy"
```

**Check MAVProxy wrapper:**
```bash
wsl bash -c "cat /usr/local/bin/mavproxy.py"
```

Should output:
```bash
#!/bin/bash
exec python3 -m MAVProxy.mavproxy "$@"
```

## Next Steps

After completing WSL2 setup:

1. Run the FFA Simulator GUI: `python -m ffa_simulator.main`
2. Click "▶ Start Simulation"
3. Wait for green "Status: Running" indicator
4. Switch to "Mission Control" tab
5. Click on map to add waypoints
6. Click "ARM" to arm the aircraft
7. Click "START MISSION (AUTO)" to begin autonomous flight

For detailed usage instructions, see [USER_GUIDE.md](USER_GUIDE.md).
