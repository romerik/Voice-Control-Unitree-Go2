#!/usr/bin/env python3
import os
import cv2
import numpy as np
import socket
import struct
import argparse
import time
import subprocess
import threading
import traceback
import mediapipe as mp
import math

class GestureRobotController:
    def __init__(self, robot_interface, server_ip='0.0.0.0', port=8089):
        # Network settings
        self.server_ip = server_ip
        self.port = port
        self.server_socket = None
        self.client_socket = None
        self.robot_interface = robot_interface
        
        # Control flags
        self.running = True
        self.connection_active = False
        
        # Performance tracking
        self.frame_count = 0
        self.last_frame_time = time.time()
        self.fps = 0
        
        # Latest frame for display thread
        self.current_frame = None
        self.frame_ready = False
        self.frame_lock = threading.Lock()
        
        # Initialize MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,  # Allow detection of two hands for multi-hand gestures
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        
        # Gesture state variables
        self.attention_mode = False
        self.last_gesture = None
        self.gesture_start_time = 0
        self.gesture_cooldown = 1.0  # seconds
        
        # Gesture confirmation system
        self.gesture_confirmation = {
            "current_gesture": None,
            "start_time": 0,
            "required_duration": 1.5,  # Seconds a gesture must be held to confirm
            "confidence": 0.0,         # Current confidence level (0.0 to 1.0)
        }
        
        # Different required durations for different gestures
        self.confirmation_times = {
            "attention": 1.0,       # Activating attention mode
            "rotate_left": 1.2,     # Rotations
            "rotate_right": 1.2,
            "come_closer": 1.2,     # Movement
            "move_away": 1.2,
            "stand_two_feet": 1.5,  # Hello (more complex)
            "gait_3": 1.5,          # Heart gesture
            "gait_4": 1.5,          # Scrape action
            "gait_5": 1.8,          # Dance (longer to confirm)
            "sit_down": 1.5,        # Fist for toggle
            "stand_four_feet": 1.5, # Palm down
            # Multi-hand gestures need slightly longer
            "gait_6": 1.8,
            "gait_7": 1.8,
            "gait_8": 1.8,
            "gait_9": 1.8,
            "gait_10": 1.8
        }
        
        # Robot state and action tracking
        self.robot_sitting = False  # Track if the robot is currently sitting
        self.robot_in_action = False  # Track if robot is currently performing an action
        self.action_start_time = 0  # When the current action started
        
        # Action duration estimates (in seconds)
        self.action_durations = {
            "wiggle_hips": 3.0,     # ID 46
            "rotate_left": 2.0,     # ID 14
            "rotate_right": 2.0,    # ID 15
            "move_forward": 2.0,    # ID 10
            "move_backward": 2.0,   # ID 11
            "hello": 5.0,           # ID 40
            "stand_up": 2.0,        # ID 1
            "sit_down": 2.5,        # ID 41
            "heart": 4.0,           # ID 47
            "scrape": 4.0,          # ID 45
            "dance": 7.0,           # ID 44
            "stretch": 4.0,         # ID 43
            "front_jump": 3.0,      # ID 53
            "walk_upright": 5.0,    # ID 64
            "cross_step": 4.0,      # ID 65
            "free_jump": 3.0        # ID 66
        }
        
        # For simulating depth (using z-coordinate of landmarks)
        self.initial_z = None
        self.finger_positions = []  # Store recent z positions to detect movement
        self.position_history_size = 5
        
    def setup_server(self):
        """Setup server socket to receive camera stream from robot"""
        try:
            if self.server_socket:
                self.server_socket.close()
            
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.server_ip, self.port))
            self.server_socket.listen(1)
            self.server_socket.settimeout(1.0)  # Add timeout to allow checking running flag
            
            print(f"Listening for camera stream on {self.server_ip}:{self.port}")
            return True
        except Exception as e:
            print(f"Server socket setup error: {e}")
            return False
    
    def accept_connection(self):
        """Accept connection with timeout support"""
        print("Waiting for robot to connect...")
        while self.running:
            try:
                self.client_socket, addr = self.server_socket.accept()
                print(f"Connection accepted from {addr}")
                self.connection_active = True
                
                # Setup data buffer
                self.data = b""
                self.payload_size = struct.calcsize("<L")  # Use '<' for little-endian
                return True
            except socket.timeout:
                # This is expected due to the timeout we set
                continue
            except Exception as e:
                print(f"Error accepting connection: {e}")
                time.sleep(1)
                return False
        
        return False
    
    def receive_frames_thread(self):
        """Thread to continuously receive frames from the network"""
        reconnect_delay = 1.0  # Initial reconnect delay
        max_reconnect_delay = 10.0
        
        while self.running:
            try:
                if not self.connection_active:
                    if not self.setup_server() or not self.accept_connection():
                        # Use exponential backoff for reconnection attempts
                        time.sleep(reconnect_delay)
                        reconnect_delay = min(reconnect_delay * 1.5, max_reconnect_delay)
                        continue
                    reconnect_delay = 1.0  # Reset delay after successful connection
                
                # Receive and process frames
                while self.running and self.connection_active:
                    frame = self.receive_frame()
                    if frame is None:
                        print("No frame received or connection lost")
                        self.connection_active = False
                        break
                    
                    # Update the current frame for display thread
                    with self.frame_lock:
                        self.current_frame = frame
                        self.frame_ready = True
                    
                    # Update performance metrics
                    self.frame_count += 1
                    current_time = time.time()
                    elapsed = current_time - self.last_frame_time
                    if elapsed >= 1.0:  # Update FPS once per second
                        self.fps = self.frame_count / elapsed
                        self.frame_count = 0
                        self.last_frame_time = current_time
                
            except Exception as e:
                print(f"Error in receive thread: {e}")
                traceback.print_exc()
                self.connection_active = False
                time.sleep(reconnect_delay)
                reconnect_delay = min(reconnect_delay * 1.5, max_reconnect_delay)
    
    def receive_frame(self):
        """Receive and decode a frame from the network stream"""
        try:
            # Get message size
            while len(self.data) < self.payload_size:
                packet = self.client_socket.recv(4096)
                if not packet:
                    return None
                self.data += packet
            
            # Extract message size
            packed_msg_size = self.data[:self.payload_size]
            self.data = self.data[self.payload_size:]
            msg_size = struct.unpack("<L", packed_msg_size)[0]  # Use '<' for little-endian
            
            # Sanity check for message size
            if msg_size > 10 * 1024 * 1024:  # Limit to 10MB for safety
                print(f"Warning: Unusually large message size: {msg_size} bytes. Resetting connection.")
                return None
            
            # Get frame data
            while len(self.data) < msg_size:
                packet = self.client_socket.recv(4096)
                if not packet:
                    return None
                self.data += packet
            
            # Extract frame data (just JPEG bytes, no pickle)
            jpeg_data = self.data[:msg_size]
            self.data = self.data[msg_size:]
            
            # Decode JPEG directly
            frame = cv2.imdecode(np.frombuffer(jpeg_data, dtype=np.uint8), cv2.IMREAD_COLOR)
            
            return frame
        
        except ConnectionResetError:
            print("Connection reset by peer")
            return None
        except socket.error as e:
            print(f"Socket error: {e}")
            return None
        except Exception as e:
            print(f"Error receiving frame: {e}")
            traceback.print_exc()
            return None
    
    def send_robot_command(self, command_id, additional_args=None, action_name=None):
        """Send command to robot using robot_control.py and track action duration"""
        cmd = ["/opt/miniconda3/envs/unitree_env/bin/python", os.path.join(
            "/Users/romeriklokossou/Downloads/Unitree_go2_dev/unitree_sdk2_python/example/go2/high_level", "robot_control.py"
        ), self.robot_interface, str(command_id)]
        
        if additional_args:
            if isinstance(additional_args, (list, tuple)):
                cmd.extend([str(arg) for arg in additional_args])
            else:
                cmd.append(str(additional_args))
        
        print(f"Sending command: {cmd}")
        
        # Set robot to "in action" state with appropriate duration
        self.robot_in_action = True
        self.action_start_time = time.time()
        
        # Use the provided action name or try to look it up from command_id
        action_duration = 2.0  # Default duration if not specified
        if action_name and action_name in self.action_durations:
            action_duration = self.action_durations[action_name]
        
        # Schedule the action to end after the duration
        action_end_time = self.action_start_time + action_duration
        
        # Run the command asynchronously
        thread = threading.Thread(target=self._execute_command, args=(cmd, action_end_time))
        thread.daemon = True
        thread.start()
    
    def _execute_command(self, cmd, action_end_time):
        """Execute robot control command in a separate thread and track when it completes"""
        try:
            subprocess.run(cmd, check=True)
            
            # Wait until the estimated action duration has passed
            remaining_time = action_end_time - time.time()
            if remaining_time > 0:
                time.sleep(remaining_time)
            
            # Mark the action as complete
            self.robot_in_action = False
            print(f"Action completed after {time.time() - self.action_start_time:.1f} seconds")
            
        except subprocess.CalledProcessError as e:
            print(f"Command failed: {e}")
            self.robot_in_action = False
        except Exception as e:
            print(f"Error executing command: {e}")
            self.robot_in_action = False
    
    def update_gesture_confirmation(self, detected_gesture):
        """
        Track gesture confirmation status over time.
        Returns the confirmed gesture only when it has been held long enough.
        """
        current_time = time.time()
        
        # If no gesture detected, reset confirmation
        if detected_gesture is None:
            self.gesture_confirmation["current_gesture"] = None
            self.gesture_confirmation["confidence"] = 0.0
            return None
        
        # If this is a new gesture, start tracking it
        if detected_gesture != self.gesture_confirmation["current_gesture"]:
            self.gesture_confirmation["current_gesture"] = detected_gesture
            self.gesture_confirmation["start_time"] = current_time
            self.gesture_confirmation["confidence"] = 0.1  # Initial confidence
            return None
        
        # Get required duration for this specific gesture (or default)
        required_duration = self.confirmation_times.get(
            detected_gesture, 
            self.gesture_confirmation["required_duration"]
        )
        
        # Calculate how long the gesture has been held
        elapsed = current_time - self.gesture_confirmation["start_time"]
        
        # Update confidence based on how long gesture has been held
        self.gesture_confirmation["confidence"] = min(1.0, elapsed / required_duration)
        
        # If gesture has been held long enough, confirm it
        if elapsed >= required_duration:
            confirmed_gesture = self.gesture_confirmation["current_gesture"]
            # Reset confidence to prevent immediate re-confirmation
            self.gesture_confirmation["confidence"] = 0.0
            return confirmed_gesture
        
        # Not confirmed yet
        return None

    # The following methods are from the original WebcamGestureController class
    def calculate_finger_direction(self, hand_landmarks):
        """Calculate the direction of the pointing finger."""
        # Index finger points
        index_tip = (hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].x,
                    hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].y)
        index_mcp = (hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP].x,
                    hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP].y)
        
        # Calculate direction vector
        direction_x = index_tip[0] - index_mcp[0]
        direction_y = index_tip[1] - index_mcp[1]
        
        return direction_x, direction_y
    
    def is_pointing(self, hand_landmarks):
        """Check if the hand is making a pointing gesture (index extended, others closed)."""
        # Get landmarks for all fingers
        index_tip_y = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].y
        index_pip_y = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_PIP].y
        
        middle_tip_y = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y
        middle_pip_y = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP].y
        
        ring_tip_y = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_TIP].y
        ring_pip_y = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_PIP].y
        
        pinky_tip_y = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_TIP].y
        pinky_pip_y = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_PIP].y
        
        # Check if index finger is extended (tip above pip)
        index_extended = index_tip_y < index_pip_y
        
        # Check if other fingers are closed (tip below pip)
        others_closed = (
            middle_tip_y > middle_pip_y and
            ring_tip_y > ring_pip_y and
            pinky_tip_y > pinky_pip_y
        )
        
        return index_extended and others_closed
    
    def count_extended_fingers(self, hand_landmarks):
        """Count how many fingers are extended."""
        # Get landmarks for all fingers
        thumb_tip_x = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP].x
        thumb_ip_x = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_IP].x
        
        index_tip_y = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].y
        index_pip_y = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_PIP].y
        
        middle_tip_y = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y
        middle_pip_y = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP].y
        
        ring_tip_y = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_TIP].y
        ring_pip_y = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_PIP].y
        
        pinky_tip_y = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_TIP].y
        pinky_pip_y = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_PIP].y
        
        # Check which fingers are extended
        # For the thumb, we check if it's extended to the side
        # For other fingers, we check if the tip is above the pip (extended upward)
        thumb_extended = thumb_tip_x < thumb_ip_x  # For right hand; might need to adjust based on hand orientation
        index_extended = index_tip_y < index_pip_y
        middle_extended = middle_tip_y < middle_pip_y
        ring_extended = ring_tip_y < ring_pip_y
        pinky_extended = pinky_tip_y < pinky_pip_y
        
        # Count extended fingers
        count = sum([thumb_extended, index_extended, middle_extended, ring_extended, pinky_extended])
        
        return count, [thumb_extended, index_extended, middle_extended, ring_extended, pinky_extended]
    
    def is_specific_fingers(self, hand_landmarks, num_fingers):
        """Check if exactly the specified number of fingers are extended."""
        count, _ = self.count_extended_fingers(hand_landmarks)
        return count == num_fingers
    
    def is_fist(self, hand_landmarks):
        """Check if the hand is making a fist (all fingers closed)."""
        count, _ = self.count_extended_fingers(hand_landmarks)
        return count == 0
    
    def is_two_fingers(self, hand_landmarks):
        """Check if the hand is showing two fingers (index and middle extended, others closed)."""
        _, fingers_extended = self.count_extended_fingers(hand_landmarks)
        
        # Check specifically for index and middle fingers extended (common peace sign)
        if sum(fingers_extended) == 2 and fingers_extended[1] and fingers_extended[2]:
            return True
        
        return False
    
    def is_palm_down(self, hand_landmarks):
        """Check if the palm is facing down (parallel to ground)."""
        # Use wrist and middle finger MCP to determine palm orientation
        wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
        middle_mcp = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP]
        
        # Palm is down if the y coordinates are similar (horizontal orientation)
        return abs(wrist.y - middle_mcp.y) < 0.05
    
    def detect_forward_backward_motion(self, hand_landmarks):
        """Detect forward and backward hand motion using z-coordinate changes."""
        # Get z-coordinate of index finger tip
        z_value = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].z
        
        # Store the z position
        self.finger_positions.append(z_value)
        if len(self.finger_positions) > self.position_history_size:
            self.finger_positions.pop(0)
        
        # Need enough history to detect motion
        if len(self.finger_positions) < self.position_history_size:
            return None
        
        # Calculate motion direction from z-coordinate changes
        # Note: z becomes more negative as hand moves closer to camera
        first_half = self.finger_positions[:self.position_history_size//2]
        second_half = self.finger_positions[self.position_history_size//2:]
        
        avg_first = sum(first_half) / len(first_half)
        avg_second = sum(second_half) / len(second_half)
        
        z_diff = avg_second - avg_first
        
        # Threshold for motion detection
        threshold = 0.05
        
        if z_diff < -threshold:  # Hand moving toward camera
            return "come_closer"
        elif z_diff > threshold:  # Hand moving away from camera
            return "move_away"
        
        return None
    
    def detect_gesture(self, hand_landmarks, multi_hand_landmarks=None):
        """Detect which gesture is being performed."""
        # First, check if attention mode is activated
        if not self.attention_mode:
            # Check if the user is pointing directly at the camera to get attention
            if self.is_pointing(hand_landmarks):
                direction_x, direction_y = self.calculate_finger_direction(hand_landmarks)
                # If pointing roughly toward camera (small x movement, negative y direction)
                if abs(direction_x) < 0.1 and direction_y < -0.1:
                    self.attention_mode = True
                    return "attention"
            return None
        
        # If we're in attention mode, detect various gestures
        
        # Check for two-hand gestures if multiple hands are detected
        if multi_hand_landmarks and len(multi_hand_landmarks) >= 2:
            # Count total extended fingers across both hands
            total_fingers = 0
            for landmarks in multi_hand_landmarks:
                count, _ = self.count_extended_fingers(landmarks)
                total_fingers += count
            
            # Gestures for combined finger counts (6-10)
            if 6 <= total_fingers <= 10:
                return f"gait_{total_fingers}"
        
        # Single hand gestures
        if self.is_pointing(hand_landmarks):
            direction_x, direction_y = self.calculate_finger_direction(hand_landmarks)
            
            # Check horizontal direction for rotation commands
            if direction_x > 0.1:
                return "rotate_right"
            elif direction_x < -0.1:
                return "rotate_left"
            
            # Check for forward/backward motion
            motion = self.detect_forward_backward_motion(hand_landmarks)
            if motion:
                return motion
        
        # Check for fist (sit down)        
        elif self.is_fist(hand_landmarks):
            return "sit_down"
                
        # Check for specific finger counts (2-5)
        else:
            finger_count, _ = self.count_extended_fingers(hand_landmarks)
            
            if finger_count == 2 and self.is_two_fingers(hand_landmarks):
                return "stand_two_feet"
            elif finger_count == 3:
                return "gait_3"
            elif finger_count == 4:
                return "gait_4"
            elif finger_count == 5:
                return "gait_5"
        
        # Check for palm down gesture (always check this last as it might overlap with other gestures)
        if self.is_palm_down(hand_landmarks):
            return "stand_four_feet"
        
        return None
    
    def execute_gesture_command(self, gesture):
        """Execute the command based on the detected gesture."""
        # Avoid too frequent commands
        current_time = time.time()
        if current_time - self.gesture_start_time < self.gesture_cooldown:
            return
            
        # Don't process new gestures if the robot is currently performing an action
        if self.robot_in_action:
            # Calculate how much time remains in the current action
            elapsed = current_time - self.action_start_time
            action_duration = 2.0  # Default duration
            if self.last_gesture in self.action_durations:
                action_duration = self.action_durations[self.last_gesture]
            
            remaining = action_duration - elapsed
            if remaining > 0:
                print(f"Robot busy with '{self.last_gesture}'. {remaining:.1f}s remaining.")
                return
            else:
                # Action should be done, but the flag wasn't reset (safety measure)
                self.robot_in_action = False
        
        if gesture != self.last_gesture:
            self.last_gesture = gesture
            self.gesture_start_time = current_time
            
            # Execute the appropriate robot command
            if gesture == "attention":
                print("Robot attention activated! (Wiggle hips)")
                self.send_robot_command(46, action_name="wiggle_hips")
                
            elif gesture == "rotate_right":
                print("Rotating left (opposite of finger direction)")
                self.send_robot_command(14, action_name="rotate_left")
                
            elif gesture == "rotate_left":
                print("Rotating right (opposite of finger direction)")
                self.send_robot_command(15, action_name="rotate_right")
                
            elif gesture == "come_closer":
                print("Moving forward")
                self.send_robot_command(10, action_name="move_forward")
                
            elif gesture == "move_away":
                print("Moving backward")
                self.send_robot_command(11, action_name="move_backward")
                
            elif gesture == "stand_two_feet":
                print("Hello gesture (shake hands)")
                self.send_robot_command(40, action_name="hello")
                
            elif gesture == "stand_four_feet":
                print("Standing on four feet")
                self.send_robot_command(1, action_name="stand_up")
                
            elif gesture == "sit_down":
                # Toggle between sit and stand
                if self.robot_sitting:
                    print("Standing up from sit")
                    self.send_robot_command(1, action_name="stand_up")
                    # Only set sitting to false after sending the stand up command
                    time.sleep(0.5)  # Brief delay to allow command to process
                    self.robot_sitting = False
                else:
                    print("Sitting down")
                    self.send_robot_command(41, action_name="sit_down")
                    self.robot_sitting = True
            
            # Finger count based gestures
            elif gesture == "gait_3":
                print("Show heart gesture (3 fingers)")
                self.send_robot_command(47, action_name="heart")
                
            elif gesture == "gait_4":
                print("Scrape action (4 fingers)")
                self.send_robot_command(45, action_name="scrape")
                
            elif gesture == "gait_5":
                print("Dance (5 fingers)")
                self.send_robot_command(44, action_name="dance")
                
            # Multi-hand gestures
            elif gesture.startswith("gait_") and int(gesture[5:]) >= 6:
                gait_number = int(gesture[5:])
                print(f"Multi-hand gesture detected: {gait_number} fingers")
                
                # Map multi-hand gestures to different actions
                if gait_number == 6:
                    self.send_robot_command(43, action_name="stretch")
                elif gait_number == 7:
                    self.send_robot_command(53, action_name="front_jump")
                elif gait_number == 8:
                    self.send_robot_command(64, action_name="walk_upright")
                elif gait_number == 9:
                    self.send_robot_command(65, action_name="cross_step")
                elif gait_number == 10:
                    self.send_robot_command(66, action_name="free_jump")
    
    def process_frame_for_gestures(self, frame):
        """Process a frame to detect and execute hand gestures"""
        if frame is None:
            return frame
        
        # Create a copy of the frame for drawing
        display_frame = frame.copy()
        
        # Flip the image horizontally for a more natural view
        display_frame = cv2.flip(display_frame, 1)
        
        # Convert the BGR image to RGB for MediaPipe
        rgb_image = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
        
        # Process the image and detect hands
        results = self.hands.process(rgb_image)
        
        # Track the detected gesture but don't execute yet
        detected_gesture = None
        
        # Only process gestures if robot is not currently in an action
        if not self.robot_in_action:
            if results.multi_hand_landmarks:
                # Drawing code stays the same
                for hand_landmarks in results.multi_hand_landmarks:
                    self.mp_drawing.draw_landmarks(
                        display_frame, 
                        hand_landmarks, 
                        self.mp_hands.HAND_CONNECTIONS,
                        self.mp_drawing_styles.get_default_hand_landmarks_style(),
                        self.mp_drawing_styles.get_default_hand_connections_style()
                    )
                
                # Count and display number of extended fingers for each hand
                hand_info = []
                for i, hand_landmarks in enumerate(results.multi_hand_landmarks):
                    count, _ = self.count_extended_fingers(hand_landmarks)
                    hand_info.append(f"Hand {i+1}: {count} fingers")
                    
                    # Display finger count above each hand
                    wrist_x = int(hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST].x * display_frame.shape[1])
                    wrist_y = int(hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST].y * display_frame.shape[0])
                    cv2.putText(display_frame, f"{count}", (wrist_x, wrist_y - 10), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
                
                # Display all hand information in top-right corner
                for i, info in enumerate(hand_info):
                    cv2.putText(display_frame, info, (display_frame.shape[1] - 200, 90 + i*30), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                # Detect gesture but don't execute it yet
                if len(results.multi_hand_landmarks) == 1:
                    detected_gesture = self.detect_gesture(results.multi_hand_landmarks[0])
                # If there are multiple hands, check both individual and multi-hand gestures
                else:
                    # First, try to detect multi-hand gestures
                    detected_gesture = self.detect_gesture(results.multi_hand_landmarks[0], results.multi_hand_landmarks)
                
                # Update confirmation system with the detected gesture
                confirmed_gesture = self.update_gesture_confirmation(detected_gesture)
                
                # Display the detected gesture (not yet confirmed)
                if detected_gesture:
                    # Calculate confidence percentage
                    confidence = self.gesture_confirmation["confidence"] * 100
                    
                    # Show detected gesture with confidence
                    cv2.putText(display_frame, f"Detected: {detected_gesture}", (10, 90), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 165, 0), 2)
                    
                    # Show a confidence bar
                    bar_x = 10
                    bar_y = 115
                    bar_width = 200
                    bar_height = 15
                    
                    # Background bar (gray)
                    cv2.rectangle(display_frame, (bar_x, bar_y), (bar_x + bar_width, bar_y + bar_height), 
                                (100, 100, 100), -1)
                    
                    # Confidence bar (changes color based on level)
                    filled_width = int(bar_width * self.gesture_confirmation["confidence"])
                    if confidence < 50:
                        color = (0, 0, 255)  # Red (low confidence)
                    elif confidence < 90:
                        color = (0, 255, 255)  # Yellow (medium confidence)
                    else:
                        color = (0, 255, 0)  # Green (high confidence)
                        
                    cv2.rectangle(display_frame, (bar_x, bar_y), (bar_x + filled_width, bar_y + bar_height),
                                color, -1)
                    
                    # Confidence text
                    cv2.putText(display_frame, f"Confidence: {confidence:.0f}%", (bar_x + bar_width + 10, bar_y + bar_height - 2),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    
                    # If gesture is confirmed, now execute it
                    if confirmed_gesture:
                        cv2.putText(display_frame, f"CONFIRMED: {confirmed_gesture}!", (10, 145), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        self.execute_gesture_command(confirmed_gesture)
            else:
                # No hands detected, reset confirmation
                self.update_gesture_confirmation(None)
        else:
            # If robot is in action, display a message
            elapsed = time.time() - self.action_start_time
            cv2.putText(display_frame, "Robot in action - Not detecting gestures", (10, 90), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                      
            # Display time remaining
            action_duration = 2.0
            if self.last_gesture in self.action_durations:
                action_duration = self.action_durations[self.last_gesture]
            
            remaining = max(0, action_duration - elapsed)
            cv2.putText(display_frame, f"Action: {self.last_gesture} ({remaining:.1f}s remaining)", (10, 120), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Still draw hand landmarks if detected, just don't process gestures
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    self.mp_drawing.draw_landmarks(
                        display_frame, 
                        hand_landmarks, 
                        self.mp_hands.HAND_CONNECTIONS,
                        self.mp_drawing_styles.get_default_hand_landmarks_style(),
                        self.mp_drawing_styles.get_default_hand_connections_style()
                    )
        
        # Display attention mode status
        status_text = "Attention: ON" if self.attention_mode else "Attention: OFF"
        cv2.putText(display_frame, status_text, (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Add gesture instructions
        height, width = display_frame.shape[:2]
        line_height = 18
        start_y = height - 180
        
        # Basic instructions
        cv2.putText(display_frame, "GESTURE INSTRUCTIONS:", (10, start_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(display_frame, "1. Point at camera to activate attention (wiggle hips)", (10, start_y + line_height*1), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(display_frame, "2. Point left/right: robot rotates opposite direction", (10, start_y + line_height*2), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(display_frame, "3. Move finger toward/away for forward/backward", (10, start_y + line_height*3), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Gesture instructions
        cv2.putText(display_frame, "4. Fist (0 fingers): Toggle sit/stand", (10, start_y + line_height*4), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(display_frame, "5. Two fingers: Hello (shake hands)", (10, start_y + line_height*5), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(display_frame, "6. Three fingers: Heart gesture", (10, start_y + line_height*6), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # More finger counts
        cv2.putText(display_frame, "7. Four fingers: Scrape action", (10, start_y + line_height*7), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(display_frame, "8. Five fingers: Dance", (10, start_y + line_height*8), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Add connection status
        status_text = f"Connection: {'ACTIVE' if self.connection_active else 'RECONNECTING'}"
        cv2.putText(display_frame, status_text, (10, 30), 
                  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if self.connection_active else (0, 0, 255), 2)
        
        # Add FPS counter
        fps_text = f"FPS: {self.fps:.1f}"
        cv2.putText(display_frame, fps_text, (width - 150, 30), 
                  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        return display_frame
    
    def run(self):
        """Main function to start display and receiver threads"""
        try:
            # Start the receiver thread
            receiver_thread = threading.Thread(target=self.receive_frames_thread)
            receiver_thread.daemon = True
            receiver_thread.start()
            
            # Main display loop
            last_key_time = 0
            key_cooldown = 0.2  # Seconds between keypresses to avoid command flooding
            
            while self.running:
                # Check if we have a frame to display
                display_frame = None
                with self.frame_lock:
                    if self.frame_ready:
                        display_frame = self.current_frame.copy()
                        self.frame_ready = False
                
                if display_frame is not None:
                    # Process frame for gestures and get the display version
                    display_frame = self.process_frame_for_gestures(display_frame)
                    
                    # Display the frame
                    cv2.imshow("Robot Gesture Control", display_frame)
                else:
                    # Display a waiting message if no frames are available
                    waiting_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                    if self.connection_active:
                        message = "Waiting for video stream..."
                    else:
                        message = "Connection lost. Attempting to reconnect..."
                    
                    cv2.putText(waiting_frame, message, (50, 240), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    cv2.imshow("Robot Gesture Control", waiting_frame)
                
                # Handle keyboard input for robot control (as backup to gestures)
                key = cv2.waitKey(1) & 0xFF
                
                # Only process key if enough time has passed since last key
                current_time = time.time()
                if current_time - last_key_time >= key_cooldown:
                    if key == ord('q'):
                        print("Quitting application")
                        self.running = False
                        break
                    elif key == ord('r'):
                        print("Resetting attention mode")
                        self.attention_mode = False
                        self.last_gesture = None
                        last_key_time = current_time
                    
                    # Only send commands if connection is active
                    if self.connection_active and not self.robot_in_action:
                        if key == ord('w'):
                            print("Moving forward")
                            self.send_robot_command(10, action_name="move_forward")
                            last_key_time = current_time
                        elif key == ord('s'):
                            print("Moving backward")
                            self.send_robot_command(11, action_name="move_backward")
                            last_key_time = current_time
                        elif key == ord('a'):
                            print("Turning left")
                            self.send_robot_command(14, action_name="rotate_left")
                            last_key_time = current_time
                        elif key == ord('d'):
                            print("Turning right")
                            self.send_robot_command(15, action_name="rotate_right")
                            last_key_time = current_time
                        elif key == ord('1'):
                            print("Standing up")
                            self.send_robot_command(1, action_name="stand_up")
                            self.robot_sitting = False
                            last_key_time = current_time
                        elif key == ord('2'):
                            print("Sitting down")
                            self.send_robot_command(41, action_name="sit_down")
                            self.robot_sitting = True
                            last_key_time = current_time
                        elif key == ord('3'):
                            print("Dancing")
                            self.send_robot_command(44, action_name="dance")
                            last_key_time = current_time
                
                # Small sleep to prevent CPU hogging
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("Stopped by user")
        except Exception as e:
            print(f"Error in main loop: {e}")
            traceback.print_exc()
        finally:
            # Release resources
            self.running = False
            if self.client_socket:
                self.client_socket.close()
            if self.server_socket:
                self.server_socket.close()
            cv2.destroyAllWindows()
            print("Resources released and application terminated")

def main():
    parser = argparse.ArgumentParser(description='Gesture-based robot control via network camera')
    parser.add_argument('robot_interface', type=str, help='Network interface connected to the robot (e.g., enp114s0)')
    parser.add_argument('--host', type=str, default='0.0.0.0', help='IP address to listen on (default: 0.0.0.0)')
    parser.add_argument('--port', type=int, default=8089, help='Port to listen on (default: 8089)')
    
    args = parser.parse_args()
    
    controller = GestureRobotController(
        robot_interface=args.robot_interface,
        server_ip=args.host,
        port=args.port
    )
    controller.run()

if __name__ == "__main__":
    main()