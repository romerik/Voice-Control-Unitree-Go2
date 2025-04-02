#!/usr/bin/env python3
"""
Fully Automated Gesture Control System Launcher
Starts receiver first, then connects to robot and launches streamer.
"""
import subprocess
import time
import os
import signal
import sys
import paramiko
import threading

# Fixed configuration for your specific setup
ROBOT_IP = "192.168.123.18"  # Robot IP
ROBOT_USER = "unitree"
ROBOT_PASSWORD = "123"
ROBOT_PATH = "~/Documents/gaze-attention"
LOCAL_INTERFACE = "en7"
STREAMING_PORT = "8089"
ROBOT_ENV_PATH = "~/py39_env/bin/activate"  # Corrected environment path
LAPTOP_IP = "192.168.123.222"  # Fixed laptop IP address

def start_gesture_receiver(should_exit):
    """Start the gesture receiver on the local machine"""
    try:
        # Command to run the gesture receiver
        cmd = [
            "python", "gesture-receiver.py",
            LOCAL_INTERFACE,
            "--host", "0.0.0.0",
            "--port", STREAMING_PORT
        ]
        
        print(f"Starting gesture receiver: {' '.join(cmd)}")
        
        # Start the process
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
            bufsize=1
        )
        
        # Start a thread to monitor the output
        def monitor_output():
            while True:
                if should_exit[0]:
                    break
                
                output = process.stdout.readline()
                if output:
                    print(f"[Local]: {output.strip()}")
                
                error = process.stderr.readline()
                if error:
                    print(f"[Local Error]: {error.strip()}")
                
                # Check if process has exited
                if process.poll() is not None:
                    # Read any remaining output
                    for line in process.stdout:
                        print(f"[Local]: {line.strip()}")
                    for line in process.stderr:
                        print(f"[Local Error]: {line.strip()}")
                    break
                
                time.sleep(0.1)
        
        # Start the output monitoring in a thread
        monitor_thread = threading.Thread(target=monitor_output)
        monitor_thread.daemon = True
        monitor_thread.start()
        
        return process
    
    except Exception as e:
        print(f"Error starting gesture receiver: {e}")
        return None

def start_robot_camera_streamer():
    """Start the camera streamer on the robot via SSH"""
    print(f"Connecting to robot at {ROBOT_IP}...")
    
    ssh_client = paramiko.SSHClient()
    ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    try:
        # Connect to the robot using the hardcoded password
        ssh_client.connect(ROBOT_IP, username=ROBOT_USER, password=ROBOT_PASSWORD)
        
        print(f"Connected to robot. Starting camera streamer...")
        
        # Command to activate environment and run streamer
        command = f"cd {ROBOT_PATH} && "
        
        # Add environment activation with correct path
        command += f"source {ROBOT_ENV_PATH} && "
        
        # Add the streamer command with fixed laptop IP
        command += f"python improved-robot-streamer.py {LAPTOP_IP} --port {STREAMING_PORT}"
        
        print(f"Executing: {command}")
        
        # Start the command in a new SSH channel
        transport = ssh_client.get_transport()
        channel = transport.open_session()
        channel.get_pty()  # Request PTY to properly handle signals
        channel.exec_command(command)
        
        # Monitor stdout and stderr in separate threads
        def read_stdout():
            while True:
                if channel.recv_ready():
                    output = channel.recv(4096).decode('utf-8')
                    if output:
                        print(f"[Robot]: {output}", end='')
                else:
                    time.sleep(0.1)
                if channel.exit_status_ready():
                    break
        
        def read_stderr():
            while True:
                if channel.recv_stderr_ready():
                    error = channel.recv_stderr(4096).decode('utf-8')
                    if error:
                        print(f"[Robot Error]: {error}", end='')
                else:
                    time.sleep(0.1)
                if channel.exit_status_ready():
                    break
        
        stdout_thread = threading.Thread(target=read_stdout)
        stderr_thread = threading.Thread(target=read_stderr)
        stdout_thread.daemon = True
        stderr_thread.daemon = True
        stdout_thread.start()
        stderr_thread.start()
        
        return ssh_client, channel
    
    except Exception as e:
        print(f"Error connecting to robot: {e}")
        if 'ssh_client' in locals():
            ssh_client.close()
        return None, None

def main():
    print("Starting automated gesture control system...")
    print(f"Using fixed laptop IP: {LAPTOP_IP}")
    
    # Flag to signal threads to exit
    should_exit = [False]
    
    def signal_handler(sig, frame):
        print("\nShutting down gesture control system...")
        should_exit[0] = True
        
    # Set up signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    
    receiver_process = None
    ssh_client = None
    
    try:
        # Start gesture receiver FIRST
        print("Step 1: Starting gesture receiver on laptop...")
        receiver_process = start_gesture_receiver(should_exit)
        if not receiver_process:
            print("Failed to start gesture receiver. Exiting.")
            sys.exit(1)
            
        # Give the receiver a moment to initialize
        print("Waiting for receiver to initialize (3 seconds)...")
        time.sleep(3)
        
        # Now start robot camera streamer
        print("Step 2: Starting camera streamer on robot...")
        ssh_client, channel = start_robot_camera_streamer()
        if not ssh_client:
            print("Failed to start camera streamer on robot. Exiting.")
            # Kill the receiver process
            if receiver_process:
                receiver_process.terminate()
            sys.exit(1)
        
        print("Both components started successfully!")
        
        # Wait for the receiver process to finish
        while receiver_process.poll() is None and not should_exit[0]:
            time.sleep(0.5)
        
    except KeyboardInterrupt:
        print("Keyboard interrupt received. Shutting down...")
    finally:
        # Set exit flag
        should_exit[0] = True
        
        # Terminate the receiver process if it's still running
        if receiver_process and receiver_process.poll() is None:
            print("Terminating gesture receiver...")
            receiver_process.terminate()
            try:
                receiver_process.wait(timeout=5)
            except:
                receiver_process.kill()
        
        # Close SSH client if it exists
        if ssh_client:
            try:
                print("Closing connection to robot...")
                ssh_client.close()
            except:
                pass
                
        print("Gesture control system shutdown complete")

if __name__ == "__main__":
    main()