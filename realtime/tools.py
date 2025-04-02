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
# query_stock_price_def = {
#     "name": "query_stock_price",
#     "description": "Queries the latest stock price information for a given stock symbol.",
#     "parameters": {
#       "type": "object",
#       "properties": {
#         "symbol": {
#           "type": "string",
#           "description": "The stock symbol to query (e.g., 'AAPL' for Apple Inc.)"
#         },
#         "period": {
#           "type": "string",
#           "description": "The time period for which to retrieve stock data (e.g., '1d' for one day, '1mo' for one month)"
#         }
#       },
#       "required": ["symbol", "period"]
#     }
# }

# async def query_stock_price_handler(symbol, period):
#     """
#     Queries the latest stock price information for a given stock symbol.
#     """
#     try:
#         stock = yf.Ticker(symbol)
#         hist = stock.history(period=period)
#         if hist.empty:
#             return {"error": "No data found for the given symbol."}
#         return hist.to_json()
 
#     except Exception as e:
#         return {"error": str(e)}

# query_stock_price = (query_stock_price_def, query_stock_price_handler)

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
        self.network_interface = "en7"
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

# # Robot movement tool
# robot_move_def = {
#     "name": "robot_move",
#     "description": "Commands the robot to move in a specified direction and speed.",
#     "parameters": {
#       "type": "object",
#       "properties": {
#         "forward_speed": {
#           "type": "number",
#           "description": "Forward/backward movement speed (-2 to 2). Positive is forward, negative is backward."
#         },
#         "lateral_speed": {
#           "type": "number",
#           "description": "Left/right movement speed (-1 to 1). Positive is right, negative is left."
#         },
#         "rotation_speed": {
#           "type": "number",
#           "description": "Rotation speed (-1 to 1). Positive is clockwise, negative is counter-clockwise."
#         }
#       },
#       "required": ["forward_speed", "lateral_speed", "rotation_speed"]
#     }
# }


# Robot movement tool with distance support
robot_move_def = {
    "name": "robot_move",
    "description": "Commands the robot to move in a specified direction with precise distance or angle control.",
    "parameters": {
      "type": "object",
      "properties": {
        "movement_type": {
          "type": "string",
          "description": "Type of movement to perform",
          "enum": ["forward", "backward", "lateral_left", "lateral_right", "rotate_left", "rotate_right"]
        },
        "value": {
          "type": "number",
          "description": "Distance in meters (for forward/backward/lateral) or angle in degrees (for rotation)"
        },
        "language": {
          "type": "string",
          "description": "Language of the command (for response formatting)",
          "enum": ["english", "french", "kinyarwanda", "other"]
        }
      },
      "required": ["movement_type", "value"]
    }
}


# Robot movement tool handler
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


# robot_move = (robot_move_def, robot_move_handler)

async def robot_move_handler(movement_type: str, value: float, language: str = "english"):
    """
    Commands the robot to move with precise distance or angle.
    """
    try:
        # Map movement type to command IDs in robot_control.py
        command_map = {
            "forward": 20,  # move_forward_distance
            "backward": 21,  # move_backward_distance
            "lateral_left": 22,  # move_left_distance
            "lateral_right": 23,  # move_right_distance
            "rotate_left": 24,  # rotate_left_angle
            "rotate_right": 25,  # rotate_right_angle
        }
        
        if movement_type not in command_map:
            return {"status": "error", "message": f"Unknown movement type: {movement_type}"}
        
        command_id = command_map[movement_type]
        
        # Validate input parameters
        if movement_type in ["forward", "backward", "lateral_left", "lateral_right"]:
            # For distance-based movements
            value = max(min(value, 5), 0.1)  # Limit distance between 0.1 and 5 meters
            unit = "meters"
        else:
            # For angle-based movements
            value = max(min(value, 360), 5)  # Limit angles between 5 and 360 degrees
            unit = "degrees"
        
        # Execute command with the appropriate ID and value
        cmd = [
            robot_client.python_env,
            robot_client.robot_script_path,
            robot_client.network_interface,
            str(command_id),
            str(value)
        ]
        
        # Format response based on language
        movement_descriptions = {
            "forward": {
                "english": f"Moving forward {value} meters",
                "french": f"Avançant de {value} mètres",
                "kinyarwanda": f"Gusubiranya imbere metero {value}"
            },
            "backward": {
                "english": f"Moving backward {value} meters",
                "french": f"Reculant de {value} mètres",
                "kinyarwanda": f"Gusubiranya inyuma metero {value}"
            },
            "rotate_left": {
                "english": f"Rotating left {value} degrees",
                "french": f"Tournant à gauche de {value} degrés",
                "kinyarwanda": f"Gukota ibumoso dogere {value}"
            },
            "rotate_right": {
                "english": f"Rotating right {value} degrees",
                "french": f"Tournant à droite de {value} degrés",
                "kinyarwanda": f"Gukota iburyo dogere {value}"
            },
            "lateral_left": {
                "english": f"Moving left {value} meters",
                "french": f"Se déplaçant à gauche de {value} mètres",
                "kinyarwanda": f"Kugendera ibumoso metero {value}"
            },
            "lateral_right": {
                "english": f"Moving right {value} meters",
                "french": f"Se déplaçant à droite de {value} mètres",
                "kinyarwanda": f"Kugendera iburyo metero {value}"
            }
        }
        
        # Use English as fallback for other languages
        lang_key = language if language in ["english", "french", "kinyarwanda"] else "english"
        description = movement_descriptions.get(movement_type, {}).get(lang_key, f"Moving {movement_type} {value} {unit}")
        
        # Log the command being executed
        await cl.Message(content=f"🤖 {description}").send()
        
        # Execute the command
        process = await asyncio.create_subprocess_exec(
            *cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE
        )
        
        stdout, stderr = await process.communicate()
        
        if process.returncode != 0:
            error_msg = stderr.decode() if stderr else "Unknown error"
            return {"status": "error", "message": f"Command failed: {error_msg}"}
        
        return {
            "status": "success",
            "message": description
        }
    except Exception as e:
        return {"status": "error", "message": str(e)}

robot_move = (robot_move_def, robot_move_handler)

# Update the robot posture tool definition to include sit and rise_sit
# robot_posture_def = {
#     "name": "robot_posture",
#     "description": "Controls the robot's posture or stance.",
#     "parameters": {
#       "type": "object",
#       "properties": {
#         "posture": {
#           "type": "string",
#           "description": "The posture to adopt.",
#           "enum": ["stand_up", "stand_down", "balanced_stand", "recovery_stand", "damp", "sit", "rise_sit"]
#         }
#       },
#       "required": ["posture"]
#     }
# }


# Update robot posture tool to support multiple languages
robot_posture_def = {
    "name": "robot_posture",
    "description": "Controls the robot's posture or stance with multilingual support.",
    "parameters": {
      "type": "object",
      "properties": {
        "posture": {
          "type": "string",
          "description": "The posture to adopt.",
          "enum": ["stand_up", "stand_down", "balanced_stand", "recovery_stand", "damp", "sit", "rise_sit"]
        },
        "language": {
          "type": "string",
          "description": "Language of the command (for response formatting)",
          "enum": ["english", "french", "kinyarwanda", "other"]
        }
      },
      "required": ["posture"]
    }
}

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
#             "damp": "Damp",
#             "sit": "Sit",
#             "rise_sit": "RiseSit"
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

# robot_posture = (robot_posture_def, robot_posture_handler)

async def robot_posture_handler(posture: str, language: str = "english"):
    """
    Commands the robot to adopt a specific posture with multilingual feedback.
    """
    try:
        # Map from posture names to command IDs
        posture_map = {
            "stand_up": 1,      # stand_up
            "stand_down": 2,    # stand_down
            "balanced_stand": 6, # balanced_stand
            "recovery_stand": 7, # recovery
            "damp": 0,          # damp
            "sit": 41,          # sit
            "rise_sit": 42      # rise_sit
        }
        
        if posture not in posture_map:
            return {"status": "error", "message": f"Unknown posture: {posture}"}
        
        command_id = posture_map[posture]
        
        # Multilingual descriptions
        posture_descriptions = {
            "stand_up": {
                "english": "Standing up",
                "french": "Se lève",
                "kinyarwanda": "Guhagarara"
            },
            "stand_down": {
                "english": "Standing down",
                "french": "S'abaisse",
                "kinyarwanda": "Kwicara hasi"
            },
            "balanced_stand": {
                "english": "Adopting balanced stance",
                "french": "Adopte une posture équilibrée",
                "kinyarwanda": "Guhagarara mu buryo buringaniye"
            },
            "recovery_stand": {
                "english": "Recovering to standing position",
                "french": "Récupérant la position debout",
                "kinyarwanda": "Gusubira mu mwanya wo guhagarara"
            },
            "damp": {
                "english": "Relaxing motors (damping)",
                "french": "Relaxation des moteurs",
                "kinyarwanda": "Kuruhura moteri"
            },
            "sit": {
                "english": "Sitting down",
                "french": "S'assoit",
                "kinyarwanda": "Kwicara"
            },
            "rise_sit": {
                "english": "Rising from sitting position",
                "french": "Se lève de la position assise",
                "kinyarwanda": "Kuvaho aho yari yicaye"
            }
        }
        
        # Use English as fallback for other languages
        lang_key = language if language in ["english", "french", "kinyarwanda"] else "english"
        description = posture_descriptions.get(posture, {}).get(lang_key, f"Adopting {posture} posture")
        
        # Execute command with the appropriate ID
        cmd = [
            robot_client.python_env,
            robot_client.robot_script_path,
            robot_client.network_interface,
            str(command_id)
        ]
        
        # Log the command being executed
        await cl.Message(content=f"🤖 {description}").send()
        
        # Execute the command
        process = await asyncio.create_subprocess_exec(
            *cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE
        )
        
        stdout, stderr = await process.communicate()
        
        if process.returncode != 0:
            error_msg = stderr.decode() if stderr else "Unknown error"
            return {"status": "error", "message": f"Command failed: {error_msg}"}
        
        return {
            "status": "success",
            "message": description
        }
    except Exception as e:
        return {"status": "error", "message": str(e)}

robot_posture = (robot_posture_def, robot_posture_handler)


# Robot special action tool definition
# robot_special_action_def = {
#     "name": "robot_special_action",
#     "description": "Commands the robot to perform a special action or movement pattern.",
#     "parameters": {
#       "type": "object",
#       "properties": {
#         "action": {
#           "type": "string",
#           "description": "The special action to perform.",
#           "enum": [
#               "left_flip", "back_flip", "free_walk", "free_bound", "free_avoid", 
#               "walk_stair", "walk_upright", "cross_step", "free_jump",
#               "hello", "stretch", "dance", "scrape", "front_flip",
#               "front_jump", "front_pounce", "wiggle_hips", "heart"
#           ]
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


# # Robot special action tool handler
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
#             "free_jump": "FreeJump",
#             "hello": "Hello",
#             "stretch": "Stretch",
#             "dance": "Dance",
#             "scrape": "Scrape",
#             "front_flip": "FrontFlip",
#             "front_jump": "FrontJump",
#             "front_pounce": "FrontPounce",
#             "wiggle_hips": "WiggleHips",
#             "heart": "Heart"
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

# robot_special_action = (robot_special_action_def, robot_special_action_handler)

robot_special_action_def = {
    "name": "robot_special_action",
    "description": "Commands the robot to perform a special action or trick with AI mode recommendations.",
    "parameters": {
      "type": "object",
      "properties": {
        "action": {
          "type": "string",
          "description": "The special action to perform.",
          "enum": [
              "left_flip", "back_flip", "free_avoid", 
              "walk_stair", "walk_upright", "cross_step",
              "hello", "stretch", "dance", "scrape", "front_flip",
              "front_jump", "front_pounce", "wiggle_hips", "heart"
          ]
        },
        "current_mode": {
          "type": "string",
          "description": "The current mode the robot is in (for AI mode recommendation)",
          "enum": ["normal", "ai", "unknown"]
        },
        "language": {
          "type": "string",
          "description": "Language of the command (for response formatting)",
          "enum": ["english", "french", "kinyarwanda", "other"]
        }
      },
      "required": ["action"]
    }
}

async def robot_special_action_handler(action: str, current_mode: str = "unknown", language: str = "english"):
    """
    Commands the robot to perform a special action with AI mode recommendations and multilingual support.
    """
    try:
        # Map from action names to command IDs
        action_map = {
            "left_flip": 50,
            "back_flip": 51,
            "free_avoid": 62,
            "walk_stair": 63,
            "walk_upright": 64,
            "cross_step": 65,
            "hello": 40,
            "stretch": 43,
            "dance": 44,
            "scrape": 45,
            "front_flip": 52,
            "front_jump": 53,
            "front_pounce": 54,
            "wiggle_hips": 46,
            "heart": 47
        }
        
        # List of actions that work better in AI mode
        ai_recommended_actions = ["left_flip", "back_flip", "walk_upright", "cross_step"]
        
        if action not in action_map:
            return {"status": "error", "message": f"Unknown action: {action}"}
        
        command_id = action_map[action]
        
        # Check if action is recommended for AI mode
        ai_mode_recommended = action in ai_recommended_actions
        
        # Multilingual descriptions
        action_descriptions = {
            "left_flip": {
                "english": "Performing left flip",
                "french": "Exécute un flip à gauche",
                "kinyarwanda": "Gukora flip ibumoso"
            },
            "back_flip": {
                "english": "Performing back flip",
                "french": "Exécute un flip arrière",
                "kinyarwanda": "Gukora flip inyuma"
            },
            "free_avoid": {
                "english": "Activating obstacle avoidance",
                "french": "Active l'évitement d'obstacles",
                "kinyarwanda": "Gutangiza kwirinda inzitizi"
            },
            "walk_stair": {
                "english": "Walking up stairs",
                "french": "Monte les escaliers",
                "kinyarwanda": "Kuzamuka amadarajya"
            },
            "walk_upright": {
                "english": "Walking upright",
                "french": "Marche debout",
                "kinyarwanda": "Kugenda uhagaze"
            },
            "cross_step": {
                "english": "Performing cross steps",
                "french": "Exécute des pas croisés",
                "kinyarwanda": "Gukora intambwe zibangikanye"
            },
            "hello": {
                "english": "Waving hello",
                "french": "Fait signe de la main",
                "kinyarwanda": "Kuramutsa"
            },
            "stretch": {
                "english": "Stretching",
                "french": "S'étire",
                "kinyarwanda": "Kwiyagura"
            },
            "dance": {
                "english": "Dancing",
                "french": "Danse",
                "kinyarwanda": "Kubyina"
            },
            "scrape": {
                "english": "Scraping",
                "french": "Gratte",
                "kinyarwanda": "Gukunguta"
            },
            "front_flip": {
                "english": "Performing front flip",
                "french": "Exécute un flip avant",
                "kinyarwanda": "Gukora flip imbere"
            },
            "front_jump": {
                "english": "Jumping forward",
                "french": "Saute en avant",
                "kinyarwanda": "Gusimbuka imbere"
            },
            "front_pounce": {
                "english": "Pouncing forward",
                "french": "Bondit en avant",
                "kinyarwanda": "Gusimbukira imbere"
            },
            "wiggle_hips": {
                "english": "Wiggling hips",
                "french": "Remue les hanches",
                "kinyarwanda": "Kunyeganyega inkoro"
            },
            "heart": {
                "english": "Showing heart gesture",
                "french": "Montre un geste de cœur",
                "kinyarwanda": "Kwerekana umutima"
            }
        }
        
        # AI mode recommendations
        ai_recommendations = {
            "english": "This action works best in AI mode. Consider switching modes if performance is not optimal.",
            "french": "Cette action fonctionne mieux en mode IA. Envisagez de changer de mode si les performances ne sont pas optimales.",
            "kinyarwanda": "Iyi action ikora neza mu buryo bwa AI. Tekereza guhindura uburyo niba imikorere idahagije."
        }
        
        # Use English as fallback for other languages
        lang_key = language if language in ["english", "french", "kinyarwanda"] else "english"
        description = action_descriptions.get(action, {}).get(lang_key, f"Performing {action}")
        
        # Execute command with the appropriate ID
        cmd = [
            robot_client.python_env,
            robot_client.robot_script_path,
            robot_client.network_interface,
            str(command_id)
        ]
        
        # Log the command being executed
        await cl.Message(content=f"🤖 {description}").send()
        
        # Execute the command
        process = await asyncio.create_subprocess_exec(
            *cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE
        )
        
        stdout, stderr = await process.communicate()
        
        if process.returncode != 0:
            error_msg = stderr.decode() if stderr else "Unknown error"
            return {"status": "error", "message": f"Command failed: {error_msg}"}
        
        response_message = description
        
        # Add AI mode recommendation if needed
        if ai_mode_recommended and current_mode != "ai":
            response_message += f"\n\n{ai_recommendations.get(lang_key)}"
        
        return {
            "status": "success",
            "message": response_message
        }
    except Exception as e:
        return {"status": "error", "message": str(e)}

robot_special_action = (robot_special_action_def, robot_special_action_handler)


# Mode switching tool with multilingual support
robot_mode_def = {
    "name": "robot_mode",
    "description": "Switches the robot between normal and AI modes.",
    "parameters": {
      "type": "object",
      "properties": {
        "mode": {
          "type": "string",
          "description": "The mode to switch to.",
          "enum": ["normal", "ai"]
        },
        "language": {
          "type": "string",
          "description": "Language of the command (for response formatting)",
          "enum": ["english", "french", "kinyarwanda", "other"]
        }
      },
      "required": ["mode"]
    }
}

async def robot_mode_handler(mode: str, language: str = "english"):
    """
    Switches the robot between normal and AI modes with multilingual support.
    """
    try:
        # Map from mode names to command IDs
        mode_map = {
            "normal": 30,  # switch_to_normal_mode
            "ai": 31       # switch_to_ai_mode
        }
        
        if mode not in mode_map:
            return {"status": "error", "message": f"Unknown mode: {mode}"}
        
        command_id = mode_map[mode]
        
        # Multilingual descriptions
        mode_descriptions = {
            "normal": {
                "english": "Switching to normal mode",
                "french": "Passage en mode normal",
                "kinyarwanda": "Guhindukira mu buryo busanzwe"
            },
            "ai": {
                "english": "Switching to AI mode",
                "french": "Passage en mode IA",
                "kinyarwanda": "Guhindukira mu buryo bwa AI"
            }
        }
        
        # Use English as fallback for other languages
        lang_key = language if language in ["english", "french", "kinyarwanda"] else "english"
        description = mode_descriptions.get(mode, {}).get(lang_key, f"Switching to {mode} mode")
        
        # Execute command with the appropriate ID
        cmd = [
            robot_client.python_env,
            robot_client.robot_script_path,
            robot_client.network_interface,
            str(command_id)
        ]
        
        # Log the command being executed
        await cl.Message(content=f"🤖 {description}").send()
        
        # Execute the command
        process = await asyncio.create_subprocess_exec(
            *cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE
        )
        
        stdout, stderr = await process.communicate()
        
        if process.returncode != 0:
            error_msg = stderr.decode() if stderr else "Unknown error"
            return {"status": "error", "message": f"Command failed: {error_msg}"}
        
        return {
            "status": "success",
            "message": description
        }
    except Exception as e:
        return {"status": "error", "message": str(e)}

robot_mode = (robot_mode_def, robot_mode_handler)


# # Robot gait tool
# robot_gait_def = {
#     "name": "robot_gait",
#     "description": "Changes the robot's gait (movement pattern).",
#     "parameters": {
#       "type": "object",
#       "properties": {
#         "gait_type": {
#           "type": "integer",
#           "description": "The type of gait (0 for trot, 1 for bound).",
#           "enum": [0, 1]
#         }
#       },
#       "required": ["gait_type"]
#     }
# }

# # Robot gait tool handler
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

# robot_gait = (robot_gait_def, robot_gait_handler)

# # Stop movement tool
# robot_stop_def = {
#     "name": "robot_stop",
#     "description": "Stops all robot movement immediately.",
#     "parameters": {
#       "type": "object",
#       "properties": {}
#     }
# }

# # Stop movement tool handler
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

# robot_stop = (robot_stop_def, robot_stop_handler)

# Stop tool with multilingual support
robot_stop_def = {
    "name": "robot_stop",
    "description": "Stops all robot movement immediately.",
    "parameters": {
      "type": "object",
      "properties": {
        "language": {
          "type": "string",
          "description": "Language of the command (for response formatting)",
          "enum": ["english", "french", "kinyarwanda", "other"]
        }
      }
    }
}

async def robot_stop_handler(language: str = "english"):
    """
    Stops all robot movement immediately with multilingual support.
    """
    try:
        # Command ID for stopping movement
        command_id = 3  # stop_move
        
        # Multilingual descriptions
        stop_descriptions = {
            "english": "Stopping all movement",
            "french": "Arrêt de tous les mouvements",
            "kinyarwanda": "Guhagarika imyigendanire yose"
        }
        
        # Use English as fallback for other languages
        lang_key = language if language in ["english", "french", "kinyarwanda"] else "english"
        description = stop_descriptions.get(lang_key, "Stopping all movement")
        
        # Execute command with the appropriate ID
        cmd = [
            robot_client.python_env,
            robot_client.robot_script_path,
            robot_client.network_interface,
            str(command_id)
        ]
        
        # Log the command being executed
        await cl.Message(content=f"🤖 {description}").send()
        
        # Execute the command
        process = await asyncio.create_subprocess_exec(
            *cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE
        )
        
        stdout, stderr = await process.communicate()
        
        if process.returncode != 0:
            error_msg = stderr.decode() if stderr else "Unknown error"
            return {"status": "error", "message": f"Command failed: {error_msg}"}
        
        return {
            "status": "success",
            "message": description
        }
    except Exception as e:
        return {"status": "error", "message": str(e)}

robot_stop = (robot_stop_def, robot_stop_handler)

# Combine all tools
# Combine all tools
tools = [
    # Original tools
    draw_plotly_chart,
    
    # Updated robot control tools
    robot_move,
    robot_posture,
    robot_special_action,
    robot_mode,
    robot_stop
]

# Language detection utility function
def detect_language(text):
    """
    Simple language detection function.
    Returns "english", "french", "kinyarwanda", or "other"
    """
    # Sample words in different languages (could be expanded)
    english_words = ["move", "forward", "backward", "left", "right", "stop", "dance", "sit", "stand", "hello"]
    french_words = ["avance", "recule", "gauche", "droite", "arrête", "danse", "assis", "debout", "bonjour"]
    kinyarwanda_words = ["genda", "imbere", "inyuma", "ibumoso", "iburyo", "hagarara", "byina", "icara", "hagarara", "muraho"]
    
    # Count words in each language
    english_count = sum(1 for word in english_words if word in text.lower())
    french_count = sum(1 for word in french_words if word in text.lower())
    kinyarwanda_count = sum(1 for word in kinyarwanda_words if word in text.lower())
    
    # Return the language with the most matches
    counts = {
        "english": english_count,
        "french": french_count,
        "kinyarwanda": kinyarwanda_count
    }
    
    max_lang = max(counts, key=counts.get)
    
    # If no clear match, default to English
    if counts[max_lang] == 0:
        return "english"
    
    return max_lang