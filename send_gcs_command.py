from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('127.0.0.1:14550')

master.wait_heartbeat()
print("Heartbeat received from system (OK)")

# Send custom GCS_COMMAND message
master.mav.gcs_command_send(
    int(time.time() * 1000),  # time_boot_ms
    1                         # command_type (e.g., 1 = perform action)
)

print("GCS_COMMAND message sent.")
