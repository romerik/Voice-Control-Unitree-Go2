import yfinance as yf
import chainlit as cl
import plotly
import json
import time
from typing import Optional, List
import subprocess
import os
import asyncio
import chainlit as cl
from typing import Optional, Dict, Any
import random

# Keep the existing stock and chart tools
query_stock_price_def = {
    "name": "query_stock_price",
    "description": "Queries the latest stock price information for a given stock symbol.",
    "parameters": {
      "type": "object",
      "properties": {
        "symbol": {
          "type": "string",
          "description": "The stock symbol to query (e.g., 'AAPL' for Apple Inc.)"
        },
        "period": {
          "type": "string",
          "description": "The time period for which to retrieve stock data (e.g., '1d' for one day, '1mo' for one month)"
        }
      },
      "required": ["symbol", "period"]
    }
}

async def query_stock_price_handler(symbol, period):
    """
    Queries the latest stock price information for a given stock symbol.
    """
    try:
        stock = yf.Ticker(symbol)
        hist = stock.history(period=period)
        if hist.empty:
            return {"error": "No data found for the given symbol."}
        return hist.to_json()
 
    except Exception as e:
        return {"error": str(e)}

query_stock_price = (query_stock_price_def, query_stock_price_handler)

draw_plotly_chart_def = {
    "name": "draw_plotly_chart",
    "description": "Draws a Plotly chart based on the provided JSON figure and displays it with an accompanying message.",
    "parameters": {
      "type": "object",
      "properties": {
        "message": {
          "type": "string",
          "description": "The message to display alongside the chart"
        },
        "plotly_json_fig": {
          "type": "string",
          "description": "A JSON string representing the Plotly figure to be drawn"
        }
      },
      "required": ["message", "plotly_json_fig"]
    }
}

async def draw_plotly_chart_handler(message: str, plotly_json_fig):
    fig = plotly.io.from_json(plotly_json_fig)
    elements = [cl.Plotly(name="chart", figure=fig, display="inline")]

    await cl.Message(content=message, elements=elements).send()
    
draw_plotly_chart = (draw_plotly_chart_def, draw_plotly_chart_handler)

# New Robot Control Tools

# Mock implementation of the robot client (to be replaced with actual implementation)
# class RobotClient:
#     def __init__(self):
#         self.status = "initialized"
        
#     async def execute_command(self, command, params=None):
#         # This would be replaced with actual robot control code
#         await cl.Message(content=f"🤖 Robot executing: {command} with parameters: {params}").send()
#         # Simulate command execution
#         for i in range(5):
#           print(f"Executing command: {command} with params: {params} ({i+1}/5)")
#           time.sleep(1)
#         return {"status": "success", "command": command, "params": params}

# class RobotClient:
#     def __init__(self, robot_script_path=None, network_interface=None, python_env=None):
#         """
#         Initialize the RobotClient with path and network settings.
        
#         Args:
#             robot_script_path: Path to the robot_control.py script
#             network_interface: Network interface to use for robot communication
#             python_env: Path to Python interpreter to use
#         """
#         # Set defaults for Romeric's environment if not provided
#         self.robot_script_path = robot_script_path or os.path.join(
#             "/Users/romeriklokossou/Downloads/Unitree_go2_dev/unitree_sdk2_python/example/go2/high_level", "robot_control.py"
#         )
#         self.network_interface = network_interface or "en7"
#         self.python_env = python_env or "/opt/miniconda3/envs/unitree_env/bin/python"
#         self.status = "initialized"
        
#         # Command ID mapping from action names to IDs
#         self.command_map = {
#             # Basic movements
#             "Move": 3,  # Default to forward movement
#             "MoveForward": 3,
#             "MoveLateral": 4,
#             "MoveRotate": 5,
#             "MoveBackward": 20,
            
#             # Postures
#             "StandUp": 1,
#             "StandDown": 2,
#             "BalanceStand": 9,
#             "RecoveryStand": 10,
#             "Damp": 0,
            
#             # Special actions
#             "LeftFlip": 11,
#             "BackFlip": 12,
#             "FreeWalk": 13,
#             "FreeBound": 14,
#             "FreeAvoid": 15,
#             "WalkStair": 16,
#             "WalkUpright": 17,
#             "CrossStep": 18,
#             "FreeJump": 19,
            
#             # Gaits
#             "SwitchGait": 7,  # Default to trot (0)
#             "SwitchGait0": 7,
#             "SwitchGait1": 8,
            
#             # Other commands
#             "StopMove": 6,
#         }
    
#     async def execute_command(self, command: str, params: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
#         """
#         Execute a robot command by calling the robot_control.py script.
        
#         Args:
#             command: The command name to execute
#             params: Optional parameters for the command
            
#         Returns:
#             Dict with status and message
#         """
#         if params is None:
#             params = {}
        
#         # Determine the command ID based on the command and params
#         command_id = self._get_command_id(command, params)
        
#         if command_id is None:
#             return {
#                 "status": "error", 
#                 "message": f"Unknown command: {command} with params: {params}"
#             }
        
#         # Log the command being executed
#         await cl.Message(content=f"🤖 Robot executing: {command} (ID: {command_id})").send()
        
#         try:
#             # Build the command to run the robot_control.py script
#             cmd = [
#                 self.python_env,
#                 self.robot_script_path,
#                 self.network_interface,
#                 str(command_id)
#             ]
            
#             # Execute the command
#             process = await asyncio.create_subprocess_exec(
#                 *cmd,
#                 stdout=asyncio.subprocess.PIPE,
#                 stderr=asyncio.subprocess.PIPE
#             )
            
#             # Wait for the command to complete and get output
#             stdout, stderr = await process.communicate()
            
#             # Check if the command was successful
#             if process.returncode != 0:
#                 error_msg = stderr.decode() if stderr else "Unknown error"
#                 return {
#                     "status": "error",
#                     "message": f"Command failed with code {process.returncode}: {error_msg}"
#                 }
            
#             # Log the output
#             output = stdout.decode()
#             print(f"Command output: {output}")
            
#             return {
#                 "status": "success",
#                 "message": f"Robot executed {command} successfully",
#                 "output": output
#             }
            
#         except Exception as e:
#             return {
#                 "status": "error",
#                 "message": f"Error executing command: {str(e)}"
#             }
    
#     def _get_command_id(self, command: str, params: Dict[str, Any]) -> Optional[int]:
#         """
#         Determine the appropriate command ID based on the command name and parameters.
        
#         Args:
#             command: The command name
#             params: Command parameters
            
#         Returns:
#             The command ID or None if not found
#         """
#         # Special handling for Move command
#         if command == "Move" and "forward_speed" in params:
#             # Determine direction based on speed values
#             forward = params.get("forward_speed", 0)
#             lateral = params.get("lateral_speed", 0)
#             rotation = params.get("rotation_speed", 0)
            
#             # Prioritize based on largest value
#             if abs(forward) >= max(abs(lateral), abs(rotation)):
#                 return 3 if forward > 0 else 20  # Forward or backward
#             elif abs(lateral) >= abs(rotation):
#                 return 4  # Lateral
#             else:
#                 return 5  # Rotation
        
#         # Special handling for SwitchGait
#         if command == "SwitchGait" and "gait_type" in params:
#             gait_type = params["gait_type"]
#             return 7 if gait_type == 0 else 8
        
#         # Special handling for action commands that toggle on/off
#         if command in ["FreeWalk", "FreeBound", "FreeAvoid", "WalkStair", 
#                       "WalkUpright", "CrossStep", "FreeJump"]:
#             # Note: For toggle commands, we're just sending the activation command
#             # The robot_control.py script handles the timing and deactivation
#             if "activate" in params and not params["activate"]:
#                 print(f"Warning: Deactivation handled internally by robot_control.py for {command}")
                
#             return self.command_map.get(command)
            
#         # Default lookup
#         return self.command_map.get(command)

class RobotClient:
    def __init__(self, robot_script_path=None, network_interface=None, python_env=None):
        """
        Initialize the RobotClient with path and network settings.
        
        Args:
            robot_script_path: Path to the robot_control.py script
            network_interface: Network interface to use for robot communication
            python_env: Path to Python interpreter to use
        """
        # Set defaults for Romeric's environment if not provided
        self.robot_script_path = robot_script_path or os.path.join(
            "/Users/romeriklokossou/Downloads/Unitree_go2_dev/unitree_sdk2_python/example/go2/high_level", "robot_control.py"
        )
        self.network_interface = network_interface or "en7"
        self.python_env = python_env or "/opt/miniconda3/envs/unitree_env/bin/python"
        self.status = "initialized"
        
        # Command ID mapping from action names to IDs
        self.command_map = {
            # Basic movements
            "Move": 3,  # Default to forward movement
            "MoveForward": 3,
            "MoveLateral": 4,
            "MoveRotate": 5,
            "MoveBackward": 20,
            
            # Postures
            "StandUp": 1,
            "StandDown": 2,
            "BalanceStand": 9,
            "RecoveryStand": 10,
            "Damp": 0,
            "Sit": 22,
            "RiseSit": 23,
            
            # Special actions
            "LeftFlip": 11,
            "BackFlip": 12,
            "FreeWalk": 13,
            "FreeBound": 14,
            "FreeAvoid": 15,
            "WalkStair": 16,
            "WalkUpright": 17,
            "CrossStep": 18,
            "FreeJump": 19,
            "Hello": 21,
            "Stretch": 24,
            "Dance": 25,
            "Scrape": 26,
            "FrontFlip": 27,
            "FrontJump": 28,
            "FrontPounce": 29,
            "WiggleHips": 30,
            "Heart": 31,
            
            # Gaits
            "SwitchGait": 7,  # Default to trot (0)
            "SwitchGait0": 7,
            "SwitchGait1": 8,
            
            # Other commands
            "StopMove": 6,
        }
    
    async def execute_command(self, command: str, params: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Execute a robot command by calling the robot_control.py script.
        
        Args:
            command: The command name to execute
            params: Optional parameters for the command
            
        Returns:
            Dict with status and message
        """
        if params is None:
            params = {}
        
        # Special handling for Dance command
        if command == "Dance":
            # We'll let the robot_control.py script handle the random dance selection
            command_id = self.command_map.get("Dance")
        else:
            # Determine the command ID based on the command and params
            command_id = self._get_command_id(command, params)
        
        if command_id is None:
            return {
                "status": "error", 
                "message": f"Unknown command: {command} with params: {params}"
            }
        
        # Log the command being executed
        await cl.Message(content=f"🤖 Robot executing: {command} (ID: {command_id})").send()
        
        try:
            # Build the command to run the robot_control.py script
            cmd = [
                self.python_env,
                self.robot_script_path,
                self.network_interface,
                str(command_id)
            ]
            
            # Execute the command
            process = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE
            )
            
            # Wait for the command to complete and get output
            stdout, stderr = await process.communicate()
            
            # Check if the command was successful
            if process.returncode != 0:
                error_msg = stderr.decode() if stderr else "Unknown error"
                return {
                    "status": "error",
                    "message": f"Command failed with code {process.returncode}: {error_msg}"
                }
            
            # Log the output
            output = stdout.decode()
            print(f"Command output: {output}")
            
            return {
                "status": "success",
                "message": f"Robot executed {command} successfully",
                "output": output
            }
            
        except Exception as e:
            return {
                "status": "error",
                "message": f"Error executing command: {str(e)}"
            }
    
    def _get_command_id(self, command: str, params: Dict[str, Any]) -> Optional[int]:
        """
        Determine the appropriate command ID based on the command name and parameters.
        
        Args:
            command: The command name
            params: Command parameters
            
        Returns:
            The command ID or None if not found
        """
        # Special handling for Move command
        if command == "Move" and "forward_speed" in params:
            # Determine direction based on speed values
            forward = params.get("forward_speed", 0)
            lateral = params.get("lateral_speed", 0)
            rotation = params.get("rotation_speed", 0)
            
            # Prioritize based on largest value
            if abs(forward) >= max(abs(lateral), abs(rotation)):
                return 3 if forward > 0 else 20  # Forward or backward
            elif abs(lateral) >= abs(rotation):
                return 4  # Lateral
            else:
                return 5  # Rotation
        
        # Special handling for SwitchGait
        if command == "SwitchGait" and "gait_type" in params:
            gait_type = params["gait_type"]
            return 7 if gait_type == 0 else 8
        
        # Special handling for action commands that toggle on/off
        if command in ["FreeWalk", "FreeBound", "FreeAvoid", "WalkStair", 
                      "WalkUpright", "CrossStep", "FreeJump"]:
            # Note: For toggle commands, we're just sending the activation command
            # The robot_control.py script handles the timing and deactivation
            if "activate" in params and not params["activate"]:
                print(f"Warning: Deactivation handled internally by robot_control.py for {command}")
                
            return self.command_map.get(command)
            
        # Default lookup
        return self.command_map.get(command)

robot_client = RobotClient()

# Robot movement tool
robot_move_def = {
    "name": "robot_move",
    "description": "Commands the robot to move in a specified direction and speed.",
    "parameters": {
      "type": "object",
      "properties": {
        "forward_speed": {
          "type": "number",
          "description": "Forward/backward movement speed (-2 to 2). Positive is forward, negative is backward."
        },
        "lateral_speed": {
          "type": "number",
          "description": "Left/right movement speed (-1 to 1). Positive is right, negative is left."
        },
        "rotation_speed": {
          "type": "number",
          "description": "Rotation speed (-1 to 1). Positive is clockwise, negative is counter-clockwise."
        }
      },
      "required": ["forward_speed", "lateral_speed", "rotation_speed"]
    }
}

# async def robot_move_handler(forward_speed: float, lateral_speed: float, rotation_speed: float):
#     """
#     Commands the robot to move with the specified speeds.
#     """
#     try:
#         # Validate input parameters
#         forward_speed = max(min(forward_speed, 2), -2)  # Clamp between -2 and 2
#         lateral_speed = max(min(lateral_speed, 1), -1)  # Clamp between -1 and 1
#         rotation_speed = max(min(rotation_speed, 1), -1)  # Clamp between -1 and 1
        
#         params = {
#             "forward_speed": forward_speed,
#             "lateral_speed": lateral_speed,
#             "rotation_speed": rotation_speed
#         }
        
#         # In a real implementation, this would call sport_client.Move()
#         result = await robot_client.execute_command("Move", params)
        
#         return {
#             "status": "success", 
#             "message": f"Robot moving with forward speed: {forward_speed}, lateral speed: {lateral_speed}, rotation speed: {rotation_speed}"
#         }
#     except Exception as e:
#         return {"status": "error", "message": str(e)}

# async def robot_move_handler(forward_speed: float, lateral_speed: float, rotation_speed: float):
#     """
#     Commands the robot to move with the specified speeds.
#     """
#     try:
#         # Validate input parameters
#         forward_speed = max(min(forward_speed, 2), -2)  # Clamp between -2 and 2
#         lateral_speed = max(min(lateral_speed, 1), -1)  # Clamp between -1 and 1
#         rotation_speed = max(min(rotation_speed, 1), -1)  # Clamp between -1 and 1
        
#         params = {
#             "forward_speed": forward_speed,
#             "lateral_speed": lateral_speed,
#             "rotation_speed": rotation_speed
#         }
        
#         # Call the robot client with the Move command
#         result = await robot_client.execute_command("Move", params)
        
#         return {
#             "status": result.get("status", "error"), 
#             "message": result.get("message", "Unknown error occurred")
#         }
#     except Exception as e:
#         return {"status": "error", "message": str(e)}

# Robot movement tool handler
async def robot_move_handler(forward_speed: float, lateral_speed: float, rotation_speed: float):
    """
    Commands the robot to move with the specified speeds.
    """
    try:
        # Validate input parameters
        forward_speed = max(min(forward_speed, 2), -2)  # Clamp between -2 and 2
        lateral_speed = max(min(lateral_speed, 1), -1)  # Clamp between -1 and 1
        rotation_speed = max(min(rotation_speed, 1), -1)  # Clamp between -1 and 1
        
        params = {
            "forward_speed": forward_speed,
            "lateral_speed": lateral_speed,
            "rotation_speed": rotation_speed
        }
        
        # Call the robot client with the Move command
        result = await robot_client.execute_command("Move", params)
        
        return {
            "status": result.get("status", "error"), 
            "message": result.get("message", "Unknown error occurred")
        }
    except Exception as e:
        return {"status": "error", "message": str(e)}


robot_move = (robot_move_def, robot_move_handler)

# Robot posture tool
# robot_posture_def = {
#     "name": "robot_posture",
#     "description": "Controls the robot's posture or stance.",
#     "parameters": {
#       "type": "object",
#       "properties": {
#         "posture": {
#           "type": "string",
#           "description": "The posture to adopt.",
#           "enum": ["stand_up", "stand_down", "balanced_stand", "recovery_stand", "damp"]
#         }
#       },
#       "required": ["posture"]
#     }
# }

# Update the robot posture tool definition to include sit and rise_sit
robot_posture_def = {
    "name": "robot_posture",
    "description": "Controls the robot's posture or stance.",
    "parameters": {
      "type": "object",
      "properties": {
        "posture": {
          "type": "string",
          "description": "The posture to adopt.",
          "enum": ["stand_up", "stand_down", "balanced_stand", "recovery_stand", "damp", "sit", "rise_sit"]
        }
      },
      "required": ["posture"]
    }
}

# async def robot_posture_handler(posture: str):
#     """
#     Commands the robot to adopt a specific posture.
#     """
#     try:
#         posture_map = {
#             "stand_up": "StandUp",
#             "stand_down": "StandDown",
#             "balanced_stand": "BalanceStand", 
#             "recovery_stand": "RecoveryStand",
#             "damp": "Damp"
#         }
        
#         if posture not in posture_map:
#             return {"status": "error", "message": f"Unknown posture: {posture}"}
        
#         command = posture_map[posture]
        
#         # In a real implementation, this would call the appropriate sport_client method
#         result = await robot_client.execute_command(command)
        
#         return {"status": "success", "message": f"Robot has adopted the '{posture}' posture"}
#     except Exception as e:
#         return {"status": "error", "message": str(e)}

# Robot posture tool handler
# async def robot_posture_handler(posture: str):
#     """
#     Commands the robot to adopt a specific posture.
#     """
#     try:
#         posture_map = {
#             "stand_up": "StandUp",
#             "stand_down": "StandDown",
#             "balanced_stand": "BalanceStand", 
#             "recovery_stand": "RecoveryStand",
#             "damp": "Damp"
#         }
        
#         if posture not in posture_map:
#             return {"status": "error", "message": f"Unknown posture: {posture}"}
        
#         command = posture_map[posture]
        
#         # Call the robot client with the appropriate posture command
#         result = await robot_client.execute_command(command)
        
#         return {
#             "status": result.get("status", "error"),
#             "message": result.get("message", f"Robot has adopted the '{posture}' posture")
#         }
#     except Exception as e:
#         return {"status": "error", "message": str(e)}

# Robot posture tool handler
async def robot_posture_handler(posture: str):
    """
    Commands the robot to adopt a specific posture.
    """
    try:
        posture_map = {
            "stand_up": "StandUp",
            "stand_down": "StandDown",
            "balanced_stand": "BalanceStand", 
            "recovery_stand": "RecoveryStand",
            "damp": "Damp",
            "sit": "Sit",
            "rise_sit": "RiseSit"
        }
        
        if posture not in posture_map:
            return {"status": "error", "message": f"Unknown posture: {posture}"}
        
        command = posture_map[posture]
        
        # Call the robot client with the appropriate posture command
        result = await robot_client.execute_command(command)
        
        return {
            "status": result.get("status", "error"),
            "message": result.get("message", f"Robot has adopted the '{posture}' posture")
        }
    except Exception as e:
        return {"status": "error", "message": str(e)}

robot_posture = (robot_posture_def, robot_posture_handler)

# Robot special action tool
# robot_special_action_def = {
#     "name": "robot_special_action",
#     "description": "Commands the robot to perform a special action or movement pattern.",
#     "parameters": {
#       "type": "object",
#       "properties": {
#         "action": {
#           "type": "string",
#           "description": "The special action to perform.",
#           "enum": ["left_flip", "back_flip", "free_walk", "free_bound", "free_avoid", 
#                   "walk_stair", "walk_upright", "cross_step", "free_jump"]
#         },
#         "activate": {
#           "type": "boolean",
#           "description": "Whether to start (true) or stop (false) the action."
#         },
#         "duration": {
#           "type": "number",
#           "description": "Duration in seconds to perform the action (for actions that require timing)."
#         }
#       },
#       "required": ["action", "activate"]
#     }
# }

# Robot special action tool definition
robot_special_action_def = {
    "name": "robot_special_action",
    "description": "Commands the robot to perform a special action or movement pattern.",
    "parameters": {
      "type": "object",
      "properties": {
        "action": {
          "type": "string",
          "description": "The special action to perform.",
          "enum": [
              "left_flip", "back_flip", "free_walk", "free_bound", "free_avoid", 
              "walk_stair", "walk_upright", "cross_step", "free_jump",
              "hello", "stretch", "dance", "scrape", "front_flip",
              "front_jump", "front_pounce", "wiggle_hips", "heart"
          ]
        },
        "activate": {
          "type": "boolean",
          "description": "Whether to start (true) or stop (false) the action."
        },
        "duration": {
          "type": "number",
          "description": "Duration in seconds to perform the action (for actions that require timing)."
        }
      },
      "required": ["action", "activate"]
    }
}

# async def robot_special_action_handler(action: str, activate: bool, duration: Optional[float] = None):
#     """
#     Commands the robot to perform a special action or movement pattern.
#     """
#     try:
#         action_map = {
#             "left_flip": "LeftFlip",
#             "back_flip": "BackFlip",
#             "free_walk": "FreeWalk",
#             "free_bound": "FreeBound",
#             "free_avoid": "FreeAvoid",
#             "walk_stair": "WalkStair",
#             "walk_upright": "WalkUpright",
#             "cross_step": "CrossStep",
#             "free_jump": "FreeJump"
#         }
        
#         if action not in action_map:
#             return {"status": "error", "message": f"Unknown action: {action}"}
        
#         command = action_map[action]
#         params = {"activate": activate}
        
#         # In a real implementation, this would call the appropriate sport_client method
#         result = await robot_client.execute_command(command, params)
        
#         # If duration is specified and action is activated, wait and then deactivate
#         if duration and activate and duration > 0:
#             await cl.Message(content=f"🤖 Action will run for {duration} seconds...").send()
#             # In real implementation, we would wait and then deactivate
#             time.sleep(min(duration, 0.5))  # Simulate waiting (capped at 0.5s for demo)
#             deactivate_result = await robot_client.execute_command(command, {"activate": False})
#             return {
#                 "status": "success", 
#                 "message": f"Robot performed '{action}' for {duration} seconds and then stopped"
#             }
        
#         action_state = "started" if activate else "stopped"
#         return {"status": "success", "message": f"Robot has {action_state} the '{action}' action"}
#     except Exception as e:
#         return {"status": "error", "message": str(e)}

# async def robot_special_action_handler(action: str, activate: bool, duration: Optional[float] = None):
#     """
#     Commands the robot to perform a special action or movement pattern.
#     """
#     try:
#         action_map = {
#             "left_flip": "LeftFlip",
#             "back_flip": "BackFlip",
#             "free_walk": "FreeWalk",
#             "free_bound": "FreeBound",
#             "free_avoid": "FreeAvoid",
#             "walk_stair": "WalkStair",
#             "walk_upright": "WalkUpright",
#             "cross_step": "CrossStep",
#             "free_jump": "FreeJump"
#         }
        
#         if action not in action_map:
#             return {"status": "error", "message": f"Unknown action: {action}"}
        
#         command = action_map[action]
        
#         # For actions that require activation/deactivation, we're letting the robot_control.py
#         # script handle the timing and deactivation, as it already has built-in timing
#         result = await robot_client.execute_command(command, {"activate": activate})
        
#         action_state = "started" if activate else "stopped"
#         return {
#             "status": result.get("status", "error"),
#             "message": result.get("message", f"Robot has {action_state} the '{action}' action")
#         }
#     except Exception as e:
#         return {"status": "error", "message": str(e)}

# Robot special action tool handler
async def robot_special_action_handler(action: str, activate: bool, duration: Optional[float] = None):
    """
    Commands the robot to perform a special action or movement pattern.
    """
    try:
        action_map = {
            "left_flip": "LeftFlip",
            "back_flip": "BackFlip",
            "free_walk": "FreeWalk",
            "free_bound": "FreeBound",
            "free_avoid": "FreeAvoid",
            "walk_stair": "WalkStair",
            "walk_upright": "WalkUpright",
            "cross_step": "CrossStep",
            "free_jump": "FreeJump",
            "hello": "Hello",
            "stretch": "Stretch",
            "dance": "Dance",
            "scrape": "Scrape",
            "front_flip": "FrontFlip",
            "front_jump": "FrontJump",
            "front_pounce": "FrontPounce",
            "wiggle_hips": "WiggleHips",
            "heart": "Heart"
        }
        
        if action not in action_map:
            return {"status": "error", "message": f"Unknown action: {action}"}
        
        command = action_map[action]
        
        # For actions that require activation/deactivation, we're letting the robot_control.py
        # script handle the timing and deactivation, as it already has built-in timing
        result = await robot_client.execute_command(command, {"activate": activate})
        
        action_state = "started" if activate else "stopped"
        return {
            "status": result.get("status", "error"),
            "message": result.get("message", f"Robot has {action_state} the '{action}' action")
        }
    except Exception as e:
        return {"status": "error", "message": str(e)}

robot_special_action = (robot_special_action_def, robot_special_action_handler)

# Robot gait tool
robot_gait_def = {
    "name": "robot_gait",
    "description": "Changes the robot's gait (movement pattern).",
    "parameters": {
      "type": "object",
      "properties": {
        "gait_type": {
          "type": "integer",
          "description": "The type of gait (0 for trot, 1 for bound).",
          "enum": [0, 1]
        }
      },
      "required": ["gait_type"]
    }
}

# async def robot_gait_handler(gait_type: int):
#     """
#     Changes the robot's gait (movement pattern).
#     """
#     try:
#         if gait_type not in [0, 1]:
#             return {"status": "error", "message": f"Invalid gait type: {gait_type}. Must be 0 or 1."}
        
#         gait_names = {0: "trot", 1: "bound"}
        
#         # In a real implementation, this would call sport_client.SwitchGait()
#         result = await robot_client.execute_command("SwitchGait", {"gait_type": gait_type})
        
#         return {"status": "success", "message": f"Robot has switched to {gait_names[gait_type]} gait"}
#     except Exception as e:
#         return {"status": "error", "message": str(e)}

# Robot gait tool handler
# async def robot_gait_handler(gait_type: int):
#     """
#     Changes the robot's gait (movement pattern).
#     """
#     try:
#         if gait_type not in [0, 1]:
#             return {"status": "error", "message": f"Invalid gait type: {gait_type}. Must be 0 or 1."}
        
#         gait_names = {0: "trot", 1: "bound"}
        
#         # Call the robot client with the SwitchGait command
#         result = await robot_client.execute_command("SwitchGait", {"gait_type": gait_type})
        
#         return {
#             "status": result.get("status", "error"),
#             "message": result.get("message", f"Robot has switched to {gait_names[gait_type]} gait")
#         }
#     except Exception as e:
#         return {"status": "error", "message": str(e)}

# Robot gait tool handler
async def robot_gait_handler(gait_type: int):
    """
    Changes the robot's gait (movement pattern).
    """
    try:
        if gait_type not in [0, 1]:
            return {"status": "error", "message": f"Invalid gait type: {gait_type}. Must be 0 or 1."}
        
        gait_names = {0: "trot", 1: "bound"}
        
        # Call the robot client with the SwitchGait command
        result = await robot_client.execute_command("SwitchGait", {"gait_type": gait_type})
        
        return {
            "status": result.get("status", "error"),
            "message": result.get("message", f"Robot has switched to {gait_names[gait_type]} gait")
        }
    except Exception as e:
        return {"status": "error", "message": str(e)}

robot_gait = (robot_gait_def, robot_gait_handler)

# Stop movement tool
robot_stop_def = {
    "name": "robot_stop",
    "description": "Stops all robot movement immediately.",
    "parameters": {
      "type": "object",
      "properties": {}
    }
}

# async def robot_stop_handler():
#     """
#     Stops all robot movement immediately.
#     """
#     try:
#         # In a real implementation, this would call sport_client.StopMove()
#         result = await robot_client.execute_command("StopMove")
        
#         return {"status": "success", "message": "Robot has stopped all movement"}
#     except Exception as e:
#         return {"status": "error", "message": str(e)}

# Stop movement tool handler

# async def robot_stop_handler():
#     """
#     Stops all robot movement immediately.
#     """
#     try:
#         # Call the robot client with the StopMove command
#         result = await robot_client.execute_command("StopMove")
        
#         return {
#             "status": result.get("status", "error"),
#             "message": result.get("message", "Robot has stopped all movement")
#         }
#     except Exception as e:
#         return {"status": "error", "message": str(e)}

# Stop movement tool handler
async def robot_stop_handler():
    """
    Stops all robot movement immediately.
    """
    try:
        # Call the robot client with the StopMove command
        result = await robot_client.execute_command("StopMove")
        
        return {
            "status": result.get("status", "error"),
            "message": result.get("message", "Robot has stopped all movement")
        }
    except Exception as e:
        return {"status": "error", "message": str(e)}

robot_stop = (robot_stop_def, robot_stop_handler)

# Combine all tools
tools = [
    # Original tools
    query_stock_price,
    draw_plotly_chart,
    
    # New robot control tools
    robot_move,
    robot_posture,
    robot_special_action,
    robot_gait,
    robot_stop
]