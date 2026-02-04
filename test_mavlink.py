from pymavlink import mavutil
import time

print("Connecting to tcp:172.24.119.69:5760...")
conn = mavutil.mavlink_connection('tcp:172.24.119.69:5760', timeout=10)

print("Waiting for heartbeat...")
conn.wait_heartbeat()

print(f"✓ Connected! System {conn.target_system}, Component {conn.target_component}")

# Read a few messages
print("\nReceiving messages for 5 seconds...")
start = time.time()
msg_count = 0

while time.time() - start < 5:
    msg = conn.recv_match(blocking=False)
    if msg:
        msg_count += 1
        if msg.get_type() in ['HEARTBEAT', 'SYS_STATUS', 'GPS_RAW_INT']:
            print(f"  {msg.get_type()}")

print(f"\n✓ Received {msg_count} messages total")
conn.close()