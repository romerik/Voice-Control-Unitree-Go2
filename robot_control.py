import time
import sys
import random
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.go2.sport.sport_client import (
    SportClient,
    PathPoint,
    SPORT_PATH_POINT_SIZE,
)
import math
from dataclasses import dataclass

@dataclass
class TestOption:
    name: str
    id: int

option_list = [
    TestOption(name="damp", id=0),         
    TestOption(name="stand_up", id=1),     
    TestOption(name="stand_down", id=2),   
    TestOption(name="move_forward", id=3),         
    TestOption(name="move_lateral", id=4),    
    TestOption(name="move_rotate", id=5),  
    TestOption(name="stop_move", id=6),  
    TestOption(name="switch_gait_0", id=7),    
    TestOption(name="switch_gait_1", id=8),
    TestOption(name="balanced_stand", id=9),     
    TestOption(name="recovery", id=10),      
    TestOption(name="left_flip", id=11),      
    TestOption(name="back_flip", id=12),
    TestOption(name="free_walk", id=13),  
    TestOption(name="free_bound", id=14), 
    TestOption(name="free_avoid", id=15),  
    TestOption(name="walk_stair", id=16), 
    TestOption(name="walk_upright", id=17),
    TestOption(name="cross_step", id=18),
    TestOption(name="free_jump", id=19),
    TestOption(name="move_backward", id=20),
    # New commands
    TestOption(name="hello", id=21),         
    TestOption(name="sit", id=22),     
    TestOption(name="rise_sit", id=23),   
    TestOption(name="stretch", id=24),         
    TestOption(name="dance", id=25),    
    TestOption(name="scrape", id=26),  
    TestOption(name="front_flip", id=27),  
    TestOption(name="front_jump", id=28),    
    TestOption(name="front_pounce", id=29),
    TestOption(name="wiggle_hips", id=30),     
    TestOption(name="heart", id=31),   
]

def execute_command(command_id, interface_name):
    """Execute a single command based on the provided ID"""
    
    print("WARNING: Please ensure there are no obstacles around the robot.")
    
    # Initialize the channel
    ChannelFactoryInitialize(0, interface_name)

    # Find the command by ID
    command_name = None
    for option in option_list:
        if option.id == command_id:
            command_name = option.name
            break
    
    if command_name is None:
        print(f"Error: Invalid command ID: {command_id}")
        print("Available commands:")
        for option in option_list:
            print(f"{option.id}: {option.name}")
        return False
    
    print(f"Executing command: {command_name} (ID: {command_id})")
    
    # Initialize Sport Client
    sport_client = SportClient()
    sport_client.SetTimeout(10.0)
    sport_client.Init()
    
    # Execute the specific command
    ret = None
    if command_id == 0:
        sport_client.Damp()
    elif command_id == 1:
        sport_client.StandUp()
    elif command_id == 2:
        sport_client.StandDown()
    elif command_id == 3:
        sport_client.Move(2, 0, 0)
    elif command_id == 4:
        sport_client.Move(0, 0.3, 0)
    elif command_id == 5:
        sport_client.Move(0, 0, 0.5)
    elif command_id == 6:
        sport_client.StopMove()
    elif command_id == 7:
        sport_client.SwitchGait(0)
    elif command_id == 8:
        sport_client.SwitchGait(1)
    elif command_id == 9:
        sport_client.BalanceStand()
    elif command_id == 10:
        sport_client.RecoveryStand()
    elif command_id == 11:
        ret = sport_client.LeftFlip()
        print("ret: ", ret)
    elif command_id == 12:
        ret = sport_client.BackFlip()
        print("ret: ", ret)
    elif command_id == 13:
        ret = sport_client.FreeWalk(True)
        print("ret: ", ret)
    elif command_id == 14:
        ret = sport_client.FreeBound(True)
        print("ret: ", ret)
        time.sleep(2)
        ret = sport_client.FreeBound(False)
        print("ret: ", ret)
    elif command_id == 15:
        ret = sport_client.FreeAvoid(True)
        print("ret: ", ret)
        time.sleep(2)
        ret = sport_client.FreeAvoid(False)
        print("ret: ", ret)
    elif command_id == 16:
        ret = sport_client.WalkStair(True)
        print("ret: ", ret)
        time.sleep(10)
        ret = sport_client.WalkStair(False)
        print("ret: ", ret)
    elif command_id == 17:
        ret = sport_client.WalkUpright(True)
        print("ret: ", ret)
        time.sleep(4)
        ret = sport_client.WalkUpright(False)
        print("ret: ", ret)
    elif command_id == 18:
        ret = sport_client.CrossStep(True)
        print("ret: ", ret)
        time.sleep(4)
        ret = sport_client.CrossStep(False)
        print("ret: ", ret)
    elif command_id == 19:
        ret = sport_client.FreeJump(True)
        print("ret: ", ret)
        time.sleep(4)
        ret = sport_client.FreeJump(False)
        print("ret: ", ret)
    elif command_id == 20:
        sport_client.Move(-0.3, 0, 0)
    # New commands
    elif command_id == 21:
        ret = sport_client.Hello()
        print("ret: ", ret)
    elif command_id == 22:
        ret = sport_client.Sit()
        print("ret: ", ret)
    elif command_id == 23:
        ret = sport_client.RiseSit()
        print("ret: ", ret)
    elif command_id == 24:
        ret = sport_client.Stretch()
        print("ret: ", ret)
    elif command_id == 25:
        # Randomly choose between Dance1 and Dance2
        dance_choice = random.choice([1, 2])
        if dance_choice == 1:
            ret = sport_client.Dance1()
            print("Dance1 ret: ", ret)
        else:
            ret = sport_client.Dance2()
            print("Dance2 ret: ", ret)
    elif command_id == 26:
        ret = sport_client.Scrape()
        print("ret: ", ret)
    elif command_id == 27:
        ret = sport_client.FrontFlip()
        print("ret: ", ret)
    elif command_id == 28:
        ret = sport_client.FrontJump()
        print("ret: ", ret)
    elif command_id == 29:
        ret = sport_client.FrontPounce()
        print("ret: ", ret)
    elif command_id == 30:
        ret = sport_client.WiggleHips()
        print("ret: ", ret)
    elif command_id == 31:
        ret = sport_client.Heart()
        print("ret: ", ret)
    
    return True

def print_available_commands():
    """Print all available commands with their IDs"""
    print("Available commands:")
    for option in option_list:
        print(f"{option.id}: {option.name}")

if __name__ == "__main__":
    # Check arguments
    if len(sys.argv) < 3:
        print(f"Usage: python3 {sys.argv[0]} <network_interface> <command_id>")
        print("Example: python3 robot_control.py eth0 1")
        print_available_commands()
        sys.exit(1)
    
    # Get network interface and command ID from arguments
    interface_name = sys.argv[1]
    
    try:
        command_id = int(sys.argv[2])
    except ValueError:
        print(f"Error: Command ID must be an integer")
        print_available_commands()
        sys.exit(1)
    
    # Execute the command
    success = execute_command(command_id, interface_name)
    
    if not success:
        sys.exit(1)