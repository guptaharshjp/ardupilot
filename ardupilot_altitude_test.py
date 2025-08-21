def check_system_status(self):
        """Check system status and pre-arm conditions"""
        self.log_message("Checking system status...")
        
        try:
            # Get system status
            msg = self.connection.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
            if msg:
                self.log_message(f"Battery voltage: {msg.voltage_battery/1000.0:.2f}V")
                self.log_message(f"System sensors: 0x{msg.onboard_control_sensors_health:08x}")
            
            # Check GPS status
            gps_msg = self.connection.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
            if gps_msg:
                self.log_message(f"GPS fix type: {gps_msg.fix_type}, satellites: {gps_msg.satellites_visible}")
            
            # Wait for EKF to be ready
            self.log_message("Waiting for EKF to initialize...")
            ekf_ready = False
            ekf_timeout = 30
            ekf_start = time.time()
            
            while time.time() - ekf_start < ekf_timeout and not ekf_ready:
                ekf_msg = self.connection.recv_match(type='EKF_STATUS_REPORT', blocking=False, timeout=1)
                if ekf_msg:
                    if ekf_msg.flags & 0x01:  # EKF_ATTITUDE bit
                        ekf_ready = True
                        self.log_message("EKF initialized successfully")
                        break
                # Also check HEARTBEAT for arming readiness
                hb_msg = self.connection.recv_match(type='HEARTBEAT', blocking=False, timeout=1)
                if hb_msg and (hb_msg.system_status == mavutil.mavlink.MAV_STATE_ACTIVE):
                    self.log_message("System active and ready")
                    break
                
                time.sleep(1)
            
            return True
            
        except Exception as e:
            self.log_message(f"Error checking system status: {e}", "WARNING")
            return True  # Continue anyway        # Clean up parameter file
        param_file = os.path.expanduser("~/test_params.parm")
        try:
            if os.path.exists(param_file):
                os.remove(param_file)
        except Exception as e:
            self.log_message(f"Could not remove parameter file: {e}", "WARNING")#!/usr/bin/env python3

import time
import subprocess
import threading
import os
import json
import signal
import sys
from datetime import datetime
from pymavlink import mavutil

class ArduPilotAltitudeTest:
    def __init__(self):
        self.connection = None
        self.log_data = []
        self.test_start_time = None
        self.sitl_process = None
        self.current_altitude = 0
        self.current_mode = "UNKNOWN"
        self.mode_switch_detected = False
        self.mode_switch_altitude = 0
        self.mode_switch_time = None
        self.target_altitude = 10.0
        self.altitude_tolerance = 0.5  # ±0.5m tolerance
        self.monitoring_active = True
        self.altitude_readings = []
        self.mode_history = []
        self.ground_altitude = None  # Store ground reference altitude
        self.relative_altitude = 0  # Altitude relative to ground

    def log_message(self, message, level="INFO"):
        """Log message with timestamp"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        log_entry = f"[{timestamp}] {level}: {message}"
        print(log_entry)
        self.log_data.append(log_entry)

    def signal_handler(self, signum, frame):
        """Handle Ctrl+C gracefully"""
        self.log_message("Received interrupt signal, cleaning up...", "WARNING")
        self.monitoring_active = False
        self.cleanup()
        sys.exit(0)

    def start_sitl(self):
        """Start ArduPilot SITL with better error handling and ground-level initialization"""
        try:
            self.log_message("Starting ArduPilot SITL...")
            
            # Check if SITL script exists
            sitl_script = os.path.expanduser("~/ardupilot/Tools/autotest/sim_vehicle.py")
            if not os.path.exists(sitl_script):
                self.log_message(f"SITL script not found at {sitl_script}", "ERROR")
                return False
            
            # SITL command with ground-level start parameters
            sitl_cmd = [
                "python3", sitl_script,
                "-v", "ArduCopter",
                "--console",
                "--map",
                "--out=127.0.0.1:14550",
                "--speedup=1",
                "--no-rebuild",
                "-L", "KSFO",  # Use San Francisco airport as default location (sea level)
                "--add-param-file=" + os.path.expanduser("~/test_params.parm")  # Custom parameters
            ]
            
            # Create a temporary parameter file for ground-level start
            param_file = os.path.expanduser("~/test_params.parm")
            try:
                with open(param_file, 'w') as f:
                    f.write("# Test parameters for ground-level start\n")
                    f.write("SIM_TERRAIN=0\n")  # Disable terrain
                    f.write("AHRS_EKF_TYPE=2\n")  # Use EKF2
                    f.write("EK2_ALT_SOURCE=0\n")  # Use barometer for altitude
                    f.write("EK2_GPS_TYPE=0\n")  # Use GPS
                    f.write("ARMING_CHECK=1\n")  # Enable arming checks but allow testing
                    f.write("FS_GCS_ENABLE=0\n")  # Disable GCS failsafe for testing
            except Exception as e:
                self.log_message(f"Warning: Could not create parameter file: {e}", "WARNING")
            
            # Change to ardupilot directory
            ardupilot_dir = os.path.expanduser("~/ardupilot")
            if not os.path.exists(ardupilot_dir):
                self.log_message(f"ArduPilot directory not found at {ardupilot_dir}", "ERROR")
                return False
            
            self.log_message(f"SITL command: {' '.join(sitl_cmd)}")
            self.sitl_process = subprocess.Popen(
                sitl_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                cwd=ardupilot_dir,
                preexec_fn=os.setsid  # Create new process group for better cleanup
            )
            
            self.log_message("SITL started, waiting for initialization...")
            time.sleep(10)  # Allow more time for SITL to initialize
            
            # Check if SITL is still running
            if self.sitl_process.poll() is not None:
                stdout, stderr = self.sitl_process.communicate()
                self.log_message(f"SITL failed to start. STDOUT: {stdout.decode()}", "ERROR")
                self.log_message(f"SITL STDERR: {stderr.decode()}", "ERROR")
                return False
            
            return True
        except Exception as e:
            self.log_message(f"Failed to start SITL: {e}", "ERROR")
            return False

    def connect_mavlink(self, retries=20, delay=3):
        """Connect to SITL via MAVLink with improved retry logic"""
        attempt = 0
        while attempt < retries and self.monitoring_active:
            try:
                self.log_message(f"Connecting to SITL via MAVLink (attempt {attempt+1}/{retries})...")
                self.connection = mavutil.mavlink_connection(
                    'udp:127.0.0.1:14550',
                    timeout=10,
                    retries=3
                )
                
                self.log_message("Waiting for heartbeat...")
                heartbeat = self.connection.wait_heartbeat(timeout=15)
                if heartbeat:
                    self.log_message(f"MAVLink connection established with system {heartbeat.get_srcSystem()}")
                    
                    # Request data streams for better telemetry
                    self.log_message("Requesting data streams...")
                    self.connection.mav.request_data_stream_send(
                        self.connection.target_system,
                        self.connection.target_component,
                        mavutil.mavlink.MAV_DATA_STREAM_ALL,
                        4,  # 4 Hz
                        1   # Enable
                    )
                    
                    # Wait for system to be fully ready
                    time.sleep(5)
                    return True
                else:
                    self.log_message("No heartbeat received", "WARNING")
                    
            except Exception as e:
                self.log_message(f"MAVLink connection attempt {attempt+1} failed: {e}", "WARNING")
                
            time.sleep(delay)
            attempt += 1
            
        self.log_message("MAVLink connection failed after all attempts", "ERROR")
        return False

    def get_mode_name(self, mode_num):
        """Convert mode number to name for ArduCopter"""
        modes = {
            0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO",
            4: "GUIDED", 5: "LOITER", 6: "RTL", 7: "CIRCLE",
            8: "POSITION", 9: "LAND", 10: "OF_LOITER", 11: "DRIFT",
            13: "SPORT", 14: "FLIP", 15: "AUTOTUNE", 16: "POSHOLD",
            17: "BRAKE", 18: "THROW", 19: "AVOID_ADSB", 20: "GUIDED_NOGPS",
            21: "SMART_RTL", 22: "FLOWHOLD", 23: "FOLLOW", 24: "ZIGZAG"
        }
        return modes.get(mode_num, f"UNKNOWN({mode_num})")

    def wait_for_mode(self, target_mode, timeout=20):
        """Wait for drone to enter specific mode with aggressive retry"""
        start_time = time.time()
        self.log_message(f"Waiting for {target_mode} mode...")
        
        # Get the mode ID first
        mode_mapping = self.connection.mode_mapping()
        target_mode_id = mode_mapping.get(target_mode)
        if target_mode_id is None:
            self.log_message(f"Mode {target_mode} not found in mapping", "ERROR")
            return False
        
        retry_count = 0
        max_retries = 10
        
        while time.time() - start_time < timeout and self.monitoring_active and retry_count < max_retries:
            try:
                # Send mode command
                self.log_message(f"Sending {target_mode} mode command (attempt {retry_count + 1})...")
                self.connection.mav.set_mode_send(
                    self.connection.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    target_mode_id
                )
                
                # Wait for confirmation
                confirmation_timeout = 3
                confirmation_start = time.time()
                
                while time.time() - confirmation_start < confirmation_timeout:
                    msg = self.connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                    if msg:
                        current_mode = self.get_mode_name(msg.custom_mode)
                        self.log_message(f"Current mode: {current_mode}")
                        
                        if current_mode == target_mode:
                            self.log_message(f"Successfully entered {target_mode} mode!")
                            return True
                    
                    time.sleep(0.5)
                
                retry_count += 1
                time.sleep(1)
                
            except Exception as e:
                self.log_message(f"Error setting mode: {e}", "WARNING")
                retry_count += 1
                time.sleep(1)
        
        self.log_message(f"Failed to enter {target_mode} mode after {retry_count} attempts", "ERROR")
        return False

    def check_system_status(self):
        """Check system status and pre-arm conditions"""
        self.log_message("Checking system status...")
        
        try:
            # Get system status
            msg = self.connection.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
            if msg:
                self.log_message(f"Battery voltage: {msg.voltage_battery/1000.0:.2f}V")
                self.log_message(f"System sensors: 0x{msg.onboard_control_sensors_health:08x}")
            
            # Check GPS status
            gps_msg = self.connection.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
            if gps_msg:
                self.log_message(f"GPS fix type: {gps_msg.fix_type}, satellites: {gps_msg.satellites_visible}")
            
            # Wait for EKF to be ready
            self.log_message("Waiting for EKF to initialize...")
            ekf_ready = False
            ekf_timeout = 30
            ekf_start = time.time()
            
            while time.time() - ekf_start < ekf_timeout and not ekf_ready:
                ekf_msg = self.connection.recv_match(type='EKF_STATUS_REPORT', blocking=False, timeout=1)
                if ekf_msg:
                    if ekf_msg.flags & 0x01:  # EKF_ATTITUDE bit
                        ekf_ready = True
                        self.log_message("EKF initialized successfully")
                        break
                
                # Also check HEARTBEAT for arming readiness
                hb_msg = self.connection.recv_match(type='HEARTBEAT', blocking=False, timeout=1)
                if hb_msg and (hb_msg.system_status == mavutil.mavlink.MAV_STATE_ACTIVE):
                    self.log_message("System active and ready")
                    break
                
                time.sleep(1)
            
            return True
            
        except Exception as e:
            self.log_message(f"Error checking system status: {e}", "WARNING")
            return True  # Continue anyway

    def monitor_telemetry(self):
        """Enhanced telemetry monitoring with ground reference and relative altitude"""
        self.log_message("Starting telemetry monitoring...")
        last_alt_log = time.time()
        ground_reference_set = False
        
        while self.monitoring_active:
            try:
                msg = self.connection.recv_match(blocking=False, timeout=0.1)
                if msg is None:
                    time.sleep(0.1)
                    continue

                current_time = time.time()

                # Monitor altitude from multiple sources
                if msg.get_type() == 'VFR_HUD':
                    self.current_altitude = msg.alt
                    
                    # Set ground reference altitude on first reading (when disarmed)
                    if not ground_reference_set and not self.connection.motors_armed():
                        self.ground_altitude = msg.alt
                        ground_reference_set = True
                        self.log_message(f"Ground reference altitude set: {self.ground_altitude:.2f}m")
                    
                    # Calculate relative altitude
                    if self.ground_altitude is not None:
                        self.relative_altitude = msg.alt - self.ground_altitude
                    
                    self.altitude_readings.append({
                        'timestamp': current_time,
                        'altitude_amsl': msg.alt,
                        'altitude_relative': self.relative_altitude,
                        'source': 'VFR_HUD'
                    })
                    
                elif msg.get_type() == 'GLOBAL_POSITION_INT':
                    alt_amsl = msg.alt / 1000.0  # Convert mm to m
                    relative_alt = msg.relative_alt / 1000.0  # Convert mm to m
                    
                    # Use relative altitude from GPS if available
                    if abs(relative_alt) < 1000:  # Sanity check
                        self.relative_altitude = relative_alt
                        self.current_altitude = alt_amsl
                        
                    self.altitude_readings.append({
                        'timestamp': current_time,
                        'altitude_amsl': alt_amsl,
                        'altitude_relative': relative_alt,
                        'source': 'GLOBAL_POSITION_INT'
                    })

                # Monitor mode changes with relative altitude logging
                elif msg.get_type() == 'HEARTBEAT':
                    new_mode = self.get_mode_name(msg.custom_mode)
                    if new_mode != self.current_mode:
                        self.log_message(f"Mode changed: {self.current_mode} → {new_mode} at {self.relative_altitude:.2f}m AGL")
                        self.mode_history.append({
                            'timestamp': current_time,
                            'from_mode': self.current_mode,
                            'to_mode': new_mode,
                            'altitude_amsl': self.current_altitude,
                            'altitude_relative': self.relative_altitude
                        })
                        self.current_mode = new_mode
                        
                        # Check for target mode switch using relative altitude
                        if new_mode == "LOITER" and not self.mode_switch_detected:
                            self.mode_switch_detected = True
                            self.mode_switch_altitude = self.relative_altitude
                            self.mode_switch_time = current_time
                            self.log_message(f"*** TARGET MODE SWITCH DETECTED: LOITER at {self.relative_altitude:.2f}m AGL ***")

                # Log altitude periodically with both absolute and relative
                if current_time - last_alt_log >= 1.0:
                    if self.ground_altitude is not None:
                        self.log_message(f"Altitude: {self.relative_altitude:.2f}m AGL ({self.current_altitude:.2f}m AMSL) | Mode: {self.current_mode}")
                    else:
                        self.log_message(f"Altitude: {self.current_altitude:.2f}m AMSL | Mode: {self.current_mode}")
                    last_alt_log = current_time

            except Exception as e:
                self.log_message(f"Telemetry monitoring error: {e}", "ERROR")
                time.sleep(0.5)

    def arm_and_takeoff(self, target_altitude=10):
        """Enhanced arm and takeoff procedure with better pre-checks"""
        try:
            self.log_message("Starting arm and takeoff sequence...")
            
            # Check system status first
            self.check_system_status()
            
            # Check if already armed
            if self.connection.motors_armed():
                self.log_message("Motors already armed")
            else:
                # Wait for system to be ready for arming
                self.log_message("Waiting for system to be ready for arming...")
                
                # Check arming prerequisites
                ready_timeout = 30
                ready_start = time.time()
                system_ready = False
                
                while time.time() - ready_start < ready_timeout and not system_ready:
                    # Check for pre-arm failures
                    statustext_msg = self.connection.recv_match(type='STATUSTEXT', blocking=False)
                    if statustext_msg:
                        message = statustext_msg.text.decode('utf-8').strip()
                        self.log_message(f"Status: {message}")
                        
                        # Look for "PreArm" messages
                        if "PreArm" in message and "PASS" in message:
                            system_ready = True
                            break
                    
                    time.sleep(1)
                
                # Try to arm multiple times
                arm_attempts = 0
                max_arm_attempts = 15
                while not self.connection.motors_armed() and arm_attempts < max_arm_attempts:
                    self.log_message(f"Arming attempt {arm_attempts + 1}...")
                    
                    # Send arm command
                    self.connection.mav.command_long_send(
                        self.connection.target_system,
                        self.connection.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                        0, 1, 0, 0, 0, 0, 0, 0
                    )
                    
                    # Wait for arming confirmation
                    arm_timeout = 3
                    arm_start = time.time()
                    
                    while time.time() - arm_start < arm_timeout:
                        msg = self.connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                        if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                            self.log_message("Successfully armed!")
                            break
                        
                        # Check for status messages
                        status_msg = self.connection.recv_match(type='STATUSTEXT', blocking=False)
                        if status_msg:
                            status_text = status_msg.text.decode('utf-8').strip()
                            if "Armed" in status_text or "Disarmed" in status_text:
                                self.log_message(f"Arming status: {status_text}")
                    
                    if self.connection.motors_armed():
                        break
                        
                    arm_attempts += 1
                    time.sleep(2)
                
                if not self.connection.motors_armed():
                    self.log_message("Failed to arm after multiple attempts", "ERROR")
                    return False

            # Set mode to GUIDED with better error handling
            self.log_message("Setting mode to GUIDED...")
            if not self.wait_for_mode("GUIDED", timeout=25):
                self.log_message("Failed to enter GUIDED mode, trying ALT_HOLD as fallback", "WARNING")
                if not self.wait_for_mode("ALT_HOLD", timeout=15):
                    self.log_message("Failed to enter any suitable mode", "ERROR")
                    return False
                else:
                    self.log_message("Using ALT_HOLD mode for takeoff")

            # Initiate takeoff
            self.log_message(f"Initiating takeoff to {target_altitude}m...")
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 0, target_altitude
            )

            # Monitor takeoff progress using relative altitude
            takeoff_start = time.time()
            takeoff_timeout = 90  # Increased timeout
            target_reached = False
            
            self.log_message("Waiting for takeoff to begin...")
            time.sleep(3)  # Give time for takeoff to start
            
            while (time.time() - takeoff_start < takeoff_timeout and 
                   self.monitoring_active and 
                   not target_reached):
                
                # Use relative altitude for takeoff monitoring
                current_rel_alt = self.relative_altitude
                
                # Check if we've reached 95% of target altitude (relative to ground)
                if current_rel_alt >= 0.95 * target_altitude:
                    target_reached = True
                    self.log_message(f"Target altitude {target_altitude}m AGL reached! Current: {current_rel_alt:.2f}m AGL")
                    break
                
                # Log progress more frequently during takeoff
                elapsed = int(time.time() - takeoff_start)
                if elapsed % 5 == 0:  # Every 5 seconds
                    self.log_message(f"Takeoff progress: {elapsed}s, altitude: {current_rel_alt:.2f}m AGL (target: {target_altitude}m AGL)")
                
                time.sleep(2)

            if not target_reached:
                final_alt = self.relative_altitude
                self.log_message(f"Takeoff timeout or failed. Final altitude: {final_alt:.2f}m AGL", "ERROR")
                
                # Try emergency takeoff if altitude is very low
                if final_alt < 1.0:
                    self.log_message("Attempting emergency takeoff procedure...", "WARNING")
                    # Switch to manual control briefly
                    if self.wait_for_mode("ALT_HOLD", timeout=10):
                        self.log_message("Switched to ALT_HOLD for manual climb")
                        time.sleep(5)
                        return self.relative_altitude > 1.0
                
                return False

            return True
            
        except Exception as e:
            self.log_message(f"Takeoff procedure failed: {e}", "ERROR")
            return False

    def run_test(self):
        """Main test execution with comprehensive error handling"""
        self.test_start_time = time.time()
        
        # Set up signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        self.log_message("="*60)
        self.log_message("ARDUPILOT ALTITUDE MODE SWITCH TEST STARTED")
        self.log_message(f"Target altitude: {self.target_altitude}m")
        self.log_message(f"Altitude tolerance: ±{self.altitude_tolerance}m")
        self.log_message("="*60)

        try:
            # Start SITL
            if not self.start_sitl():
                self.generate_report(False, ["SITL startup failed"])
                return

            # Connect to MAVLink
            if not self.connect_mavlink():
                self.generate_report(False, ["MAVLink connection failed"])
                return

            # Start telemetry monitoring and wait for ground reference
            telemetry_thread = threading.Thread(target=self.monitor_telemetry, daemon=True)
            telemetry_thread.start()
            
            # Wait for ground reference altitude to be set
            self.log_message("Waiting for ground reference altitude...")
            wait_start = time.time()
            while self.ground_altitude is None and time.time() - wait_start < 15:
                time.sleep(0.5)
            
            if self.ground_altitude is None:
                self.log_message("Warning: Ground reference altitude not set, using current altitude", "WARNING")
                self.ground_altitude = self.current_altitude
            
            self.log_message(f"Ground reference altitude: {self.ground_altitude:.2f}m AMSL")
            time.sleep(3)  # Allow telemetry to stabilize

            # Perform takeoff
            if not self.arm_and_takeoff(self.target_altitude):
                self.generate_report(False, ["Takeoff failed"])
                return

            # Monitor for mode switch
            self.log_message(f"Monitoring for altitude-based mode switch for up to 120 seconds...")
            start_monitor = time.time()
            monitor_timeout = 120  # 2 minutes
            
            while (time.time() - start_monitor < monitor_timeout and 
                   self.monitoring_active and 
                   not self.mode_switch_detected):
                
                # Log progress every 10 seconds
                if int(time.time() - start_monitor) % 10 == 0:
                    elapsed = int(time.time() - start_monitor)
                    self.log_message(f"Monitoring progress: {elapsed}s elapsed, altitude: {self.relative_altitude:.2f}m AGL, mode: {self.current_mode}")
                
                time.sleep(1)

            # Stop monitoring
            self.monitoring_active = False
            time.sleep(1)  # Allow threads to finish

            # Analyze results
            self.analyze_results()

        except Exception as e:
            self.log_message(f"Test execution failed: {e}", "ERROR")
            self.generate_report(False, [f"Test execution error: {str(e)}"])

    def analyze_results(self):
        """Enhanced analysis of test results"""
        self.log_message("="*60)
        self.log_message("ANALYZING TEST RESULTS...")
        self.log_message("="*60)

        test_passed = True
        failure_reasons = []
        analysis_notes = []

        # Check if mode switch was detected
        if not self.mode_switch_detected:
            test_passed = False
            failure_reasons.append("No mode switch to LOITER detected")
            self.log_message("FAIL: Mode switch to LOITER not detected", "ERROR")
            
            # Provide additional analysis
            if self.mode_history:
                self.log_message("Mode changes detected during test:")
                for mode_change in self.mode_history:
                    self.log_message(f"  {mode_change['from_mode']} → {mode_change['to_mode']} at {mode_change['altitude_relative']:.2f}m AGL")
            else:
                analysis_notes.append("No mode changes detected during test")
                
        else:
            # Analyze the mode switch altitude
            altitude_diff = abs(self.mode_switch_altitude - self.target_altitude)
            
            if altitude_diff <= self.altitude_tolerance:
                self.log_message(f"PASS: Mode switch at {self.mode_switch_altitude:.2f}m (target: {self.target_altitude}m, tolerance: ±{self.altitude_tolerance}m)")
                analysis_notes.append(f"Mode switch within tolerance: {altitude_diff:.2f}m difference")
            else:
                test_passed = False
                if self.mode_switch_altitude < (self.target_altitude - self.altitude_tolerance):
                    failure_reasons.append(f"Mode switch too early: {self.mode_switch_altitude:.2f}m < {self.target_altitude - self.altitude_tolerance:.2f}m")
                    self.log_message(f"FAIL: Mode switch too early at {self.mode_switch_altitude:.2f}m", "ERROR")
                else:
                    failure_reasons.append(f"Mode switch too late: {self.mode_switch_altitude:.2f}m > {self.target_altitude + self.altitude_tolerance:.2f}m")
                    self.log_message(f"FAIL: Mode switch too late at {self.mode_switch_altitude:.2f}m", "ERROR")

        # Additional analysis
        if self.altitude_readings:
            max_rel_alt = max(reading.get('altitude_relative', 0) for reading in self.altitude_readings)
            analysis_notes.append(f"Maximum relative altitude reached: {max_rel_alt:.2f}m AGL")
            analysis_notes.append(f"Ground reference altitude: {self.ground_altitude:.2f}m AMSL")
            
        if self.mode_history:
            analysis_notes.append(f"Total mode changes: {len(self.mode_history)}")

        self.generate_report(test_passed, failure_reasons, analysis_notes)

    def generate_report(self, passed, failure_reasons, analysis_notes=None):
        """Generate comprehensive test report"""
        duration = time.time() - self.test_start_time if self.test_start_time else 0
        
        # Prepare report data
        report = {
            "test_info": {
                "test_name": "ArduPilot Altitude Mode Switch Test",
                "timestamp": datetime.now().isoformat(),
                "duration_seconds": round(duration, 2),
                "result": "PASS" if passed else "FAIL"
            },
            "test_parameters": {
                "target_altitude": self.target_altitude,
                "altitude_tolerance": self.altitude_tolerance,
                "target_mode": "LOITER"
            },
            "test_results": {
                "mode_switch_detected": self.mode_switch_detected,
                "actual_switch_altitude": self.mode_switch_altitude,
                "switch_time": self.mode_switch_time - self.test_start_time if self.mode_switch_time else None,
                "failure_reasons": failure_reasons if not passed else [],
                "analysis_notes": analysis_notes or []
            },
            "telemetry_data": {
                "altitude_readings_count": len(self.altitude_readings),
                "mode_changes": self.mode_history,
                "final_altitude_amsl": self.current_altitude,
                "final_altitude_relative": self.relative_altitude,
                "ground_reference_altitude": self.ground_altitude,
                "final_mode": self.current_mode
            },
            "detailed_log": self.log_data
        }

        # Save JSON report
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        report_filename = f"altitude_test_report_{timestamp_str}.json"
        try:
            with open(report_filename, 'w') as f:
                json.dump(report, f, indent=2)
            self.log_message(f"Report saved: {report_filename}")
        except Exception as e:
            self.log_message(f"Failed to save report: {e}", "ERROR")

        # Save text log
        log_filename = f"altitude_test_log_{timestamp_str}.txt"
        try:
            with open(log_filename, 'w') as f:
                f.write("\n".join(self.log_data))
            self.log_message(f"Log saved: {log_filename}")
        except Exception as e:
            self.log_message(f"Failed to save log: {e}", "ERROR")

        # Print summary
        self.log_message("="*60)
        self.log_message(f"TEST RESULT: {'PASS' if passed else 'FAIL'}")
        self.log_message(f"Duration: {duration:.1f} seconds")
        
        if self.mode_switch_detected:
            self.log_message(f"Mode switch altitude: {self.mode_switch_altitude:.2f}m AGL")
            self.log_message(f"Target altitude: {self.target_altitude:.2f}m AGL")
            self.log_message(f"Altitude difference: {abs(self.mode_switch_altitude - self.target_altitude):.2f}m")
        
        if self.ground_altitude is not None:
            self.log_message(f"Ground reference: {self.ground_altitude:.2f}m AMSL")
            self.log_message(f"Final altitude: {self.relative_altitude:.2f}m AGL ({self.current_altitude:.2f}m AMSL)")
        
        if failure_reasons:
            self.log_message("Failure reasons:")
            for reason in failure_reasons:
                self.log_message(f"  - {reason}")
        if analysis_notes:
            self.log_message("Analysis notes:")
            for note in analysis_notes:
                self.log_message(f"  - {note}")
                
        self.log_message(f"Files saved: {report_filename}, {log_filename}")
        self.log_message("="*60)

        # Cleanup
        self.cleanup()

    def cleanup(self):
        """Enhanced cleanup with better process management"""
        self.log_message("Performing cleanup...")
        self.monitoring_active = False
        
        if self.connection:
            try:
                # Try to land the drone if still connected
                if self.current_mode not in ["LAND", "RTL"]:
                    self.log_message("Setting mode to LAND for safe shutdown...")
                    land_mode_id = self.connection.mode_mapping().get("LAND")
                    if land_mode_id:
                        self.connection.mav.set_mode_send(
                            self.connection.target_system,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            land_mode_id
                        )
                        time.sleep(1)
                
                self.connection.close()
                self.log_message("MAVLink connection closed")
            except Exception as e:
                self.log_message(f"Error during MAVLink cleanup: {e}", "WARNING")

        if self.sitl_process:
            try:
                # Send SIGTERM first, then SIGKILL if necessary
                os.killpg(os.getpgid(self.sitl_process.pid), signal.SIGTERM)
                time.sleep(2)
                
                if self.sitl_process.poll() is None:
                    self.log_message("SITL process didn't terminate, sending SIGKILL...")
                    os.killpg(os.getpgid(self.sitl_process.pid), signal.SIGKILL)
                    
                self.sitl_process.wait()
                self.log_message("SITL process terminated")
            except Exception as e:
                self.log_message(f"Error terminating SITL process: {e}", "WARNING")

def main():
    """Main function with better error handling"""
    print("ArduPilot Altitude Mode Switch Automated Test")
    print("This script tests custom firmware that switches to LOITER mode at 10m altitude")
    print("Press Ctrl+C to stop the test at any time\n")
    
    try:
        test = ArduPilotAltitudeTest()
        test.run_test()
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"\nUnexpected error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()