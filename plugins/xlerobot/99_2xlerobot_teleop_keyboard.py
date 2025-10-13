# To Run on the host
'''python
PYTHONPATH=src python -m lerobot.robots.xlerobot.xlerobot_host --robot.id=my_xlerobot
'''

# To Run the teleop:
'''python
PYTHONPATH=src python -m examples.xlerobot.teleoperate_Keyboard
'''

import time
import numpy as np
import math
import threading

from lerobot.robots.xlerobot import XLerobotClient, XLerobotConfig, XLerobot
from lerobot.utils.robot_utils import busy_wait
#from lerobot.utils.visualization_utils import _init_rerun, log_rerun_data
from lerobot.model.SO101Robot import SO101Kinematics
#from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig

import json
import zmq

action_cmd = {}

def zmq_thread():
    zmq_context = zmq.Context()
    zmq_cmd_socket = zmq_context.socket(zmq.PULL)
    zmq_cmd_socket.setsockopt(zmq.CONFLATE, 1)
    zmq_cmd_socket.bind(f"tcp://*:5558")
    global action_cmd
    while True:
        try:
            msg = zmq_cmd_socket.recv_string()
            print(msg)
            action_cmd = dict(json.loads(msg))
        except zmq.Again:
            #action_cmd = {}
            pass
        except Exception as e:
            action_cmd = {}
            print("Message fetching failed: %s", e)

t = threading.Thread(target=zmq_thread, daemon=True)
t.start()

# Keymaps (semantic action: key)
LEFT_KEYMAP = {
    'shoulder_pan+': 'd', 'shoulder_pan-': 'a',
    'wrist_roll+': '1', 'wrist_roll-': '2',
    'gripper+': 'x', 'gripper-': 'z',
    'x+': 'w', 'x-': 's', 'y+': 'q', 'y-': 'e',
    'pitch+': 'f', 'pitch-': 'r',
    'reset': 'c',
    # For head motors
    "head_motor_1+": "8", "head_motor_1-": "7",
    "head_motor_2+": "9", "head_motor_2-": "0",
    
    'triangle': 'm',  # Rectangle trajectory key
}
RIGHT_KEYMAP = {
    'shoulder_pan+': 'k', 'shoulder_pan-': 'h',
    'wrist_roll+': '5', 'wrist_roll-': '6',
    'gripper+': 'n', 'gripper-': 'b',
    'x+': 'u', 'x-': 'j', 'y+': 'y', 'y-': 'i',
    'pitch+': 'g', 'pitch-': 't',
    'reset': 'p',

    'triangle': 'o',  # Rectangle trajectory key
}

LEFT_JOINT_MAP = {
    "shoulder_pan": "left_arm_shoulder_pan",
    "shoulder_lift": "left_arm_shoulder_lift",
    "elbow_flex": "left_arm_elbow_flex",
    "wrist_flex": "left_arm_wrist_flex",
    "wrist_roll": "left_arm_wrist_roll",
    "gripper": "left_arm_gripper",
}
RIGHT_JOINT_MAP = {
    "shoulder_pan": "right_arm_shoulder_pan",
    "shoulder_lift": "right_arm_shoulder_lift",
    "elbow_flex": "right_arm_elbow_flex",
    "wrist_flex": "right_arm_wrist_flex",
    "wrist_roll": "right_arm_wrist_roll",
    "gripper": "right_arm_gripper",
}

# Head motor mapping
HEAD_MOTOR_MAP = {
    "head_motor_1": "head_motor_1",
    "head_motor_2": "head_motor_2",
}

class RectangularTrajectory:
    """
    Generates a rectangular trajectory on the x-y plane with sinusoidal velocity profiles.
    The rectangle is divided into 4 line segments, each with smooth acceleration/deceleration.
    """
    def __init__(self, width=0.06, height=0.06, segment_duration=0.91):
        """
        Initialize rectangular trajectory parameters.
        
        Args:
            width: Rectangle width in meters
            height: Rectangle height in meters  
            segment_duration: Time for each line segment in seconds
        """
        self.width = width
        self.height = height
        self.segment_duration = segment_duration
        self.total_duration = 4 * segment_duration
        
    def get_trajectory_point(self, current_x, current_y, t):
        """
        Get the target x, y position at time t for the rectangular trajectory.
        
        Args:
            current_x: Starting x position
            current_y: Starting y position
            t: Time since trajectory start (0 to total_duration)
            
        Returns:
            tuple: (target_x, target_y)
        """
        # Determine which segment we're in
        segment = int(t / self.segment_duration)
        segment_t = t % self.segment_duration
        
        # Normalize segment time (0 to 1)
        normalized_t = segment_t / self.segment_duration
        
        # Sinusoidal velocity profile: smooth acceleration and deceleration
        # s(t) = 0.5 * (1 - cos(π * t)) gives smooth 0 to 1 transition
        smooth_t = 0.5 * (1 - math.cos(math.pi * normalized_t))
        
        # Define rectangle corners relative to starting position
        corners = [
            (current_x, current_y),                           # Start (bottom-left)
            (current_x + self.width, current_y),              # Bottom-right
            (current_x + self.width, current_y + self.height), # Top-right  
            (current_x, current_y + self.height),             # Top-left
            (current_x, current_y)                            # Back to start
        ]
        
        # Clamp segment to valid range
        segment = max(0, min(3, segment))
        
        # Interpolate between current corner and next corner
        start_corner = corners[segment]
        end_corner = corners[segment + 1]
        
        target_x = start_corner[0] + smooth_t * (end_corner[0] - start_corner[0])
        target_y = start_corner[1] + smooth_t * (end_corner[1] - start_corner[1])
        
        return target_x, target_y

class SimpleHeadControl:
    def __init__(self, initial_obs, kp=0.81):
        self.kp = kp
        self.degree_step = 1
        self.head_motor_1_min = -90.0
        self.head_motor_1_max = 90.0
        self.head_motor_2_min = -95.0
        self.head_motor_2_max = 85.0
        # Initialize head motor positions
        self.target_positions = {
            "head_motor_1": initial_obs.get("head_motor_1.pos", 0.0),
            "head_motor_2": initial_obs.get("head_motor_2.pos", 0.0),
        }
        self.zero_pos = {"head_motor_1": 0.0, "head_motor_2": 0.0}

    def move_to_zero_position(self, robot):
        self.target_positions = self.zero_pos.copy()
        action = self.p_control_action(robot)
        robot.send_action(action)

    def handle_keys(self, key_state):
        if key_state.get('head_motor_1+'):
            self.target_positions["head_motor_1"] += self.degree_step
            self.target_positions["head_motor_1"] = min(self.target_positions["head_motor_1"], self.head_motor_1_max)
            print(f"[HEAD] head_motor_1: {self.target_positions['head_motor_1']}")
        if key_state.get('head_motor_1-'):
            self.target_positions["head_motor_1"] -= self.degree_step
            self.target_positions["head_motor_1"] = max(self.target_positions["head_motor_1"], self.head_motor_1_min)
            print(f"[HEAD] head_motor_1: {self.target_positions['head_motor_1']}")
        if key_state.get('head_motor_2+'):
            self.target_positions["head_motor_2"] += self.degree_step
            self.target_positions["head_motor_2"] = min(self.target_positions["head_motor_2"], self.head_motor_2_max)
            print(f"[HEAD] head_motor_2: {self.target_positions['head_motor_2']}")
        if key_state.get('head_motor_2-'):
            self.target_positions["head_motor_2"] -= self.degree_step
            self.target_positions["head_motor_2"] = max(self.target_positions["head_motor_2"], self.head_motor_2_min)
            print(f"[HEAD] head_motor_2: {self.target_positions['head_motor_2']}")

    def p_control_action(self, robot):
        obs = robot.get_observation()
        action = {}
        for motor in self.target_positions:
            current = obs.get(f"{HEAD_MOTOR_MAP[motor]}.pos", 0.0)
            error = self.target_positions[motor] - current
            control = self.kp * error
            action[f"{HEAD_MOTOR_MAP[motor]}.pos"] = current + control
        return action

class SimpleTeleopArm:
    def __init__(self, kinematics, joint_map, initial_obs, prefix="left", kp=0.81):
        self.kinematics = kinematics
        self.joint_map = joint_map
        self.prefix = prefix  # To distinguish left and right arm
        self.kp = kp
        self.gripper_min = 0.0
        self.gripper_max = 95.0
        # Initial joint positions
        self.joint_positions = {
            "shoulder_pan": initial_obs[f"{prefix}_arm_shoulder_pan.pos"],
            "shoulder_lift": initial_obs[f"{prefix}_arm_shoulder_lift.pos"],
            "elbow_flex": initial_obs[f"{prefix}_arm_elbow_flex.pos"],
            "wrist_flex": initial_obs[f"{prefix}_arm_wrist_flex.pos"],
            "wrist_roll": initial_obs[f"{prefix}_arm_wrist_roll.pos"],
            "gripper": initial_obs[f"{prefix}_arm_gripper.pos"],
        }
        # Set initial x/y to fixed values
        self.current_x = 0.1629
        self.current_y = 0.1131
        self.pitch = 0.0
        # Set the degree step and xy step
        self.degree_step = 1
        self.xy_step = 0.0021
        # Set target positions to zero for P control
        self.target_positions = {
            "shoulder_pan": 0.0,
            "shoulder_lift": 0.0,
            "elbow_flex": 0.0,
            "wrist_flex": 0.0,
            "wrist_roll": 0.0,
            "gripper": 0.0,
        }
        self.zero_pos = {
            'shoulder_pan': 0.0,
            'shoulder_lift': 0.0,
            'elbow_flex': 0.0,
            'wrist_flex': 0.0,
            'wrist_roll': 0.0,
            'gripper': 0.0
        }
        
        # Rectangular trajectory instance
        self.rectangular_trajectory = RectangularTrajectory(
            width=0.06,          # 6cm wide rectangle
            height=0.06,         # 4cm tall rectangle  
            segment_duration=1.01 # 3 seconds per line segment
        )

    def move_to_zero_position(self, robot):
        print(f"[{self.prefix}] Moving to Zero Position: {self.zero_pos} ......")
        self.target_positions = self.zero_pos.copy()  # Use copy to avoid reference issues
        
        # Reset kinematic variables to their initial state
        self.current_x = 0.1629
        self.current_y = 0.1131
        self.pitch = 0.0
        
        # Don't let handle_keys recalculate wrist_flex - set it explicitly
        self.target_positions["wrist_flex"] = 0.0
        
        action = self.p_control_action(robot)
        robot.send_action(action)

    def execute_rectangular_trajectory(self, robot, fps=30):
        """
        Execute a blocking rectangular trajectory on the x-y plane.
        
        Args:
            robot: Robot instance to send actions to
            fps: Control loop frequency
        """
        print(f"[{self.prefix}] Starting rectangular trajectory...")
        print(f"[{self.prefix}] Rectangle: {self.rectangular_trajectory.width:.3f}m x {self.rectangular_trajectory.height:.3f}m")
        print(f"[{self.prefix}] Duration: {self.rectangular_trajectory.total_duration:.3f}s total")
        
        # Store starting position
        start_x = self.current_x
        start_y = self.current_y
        
        # Execute trajectory
        start_time = time.time()
        dt = 1.0 / fps
        
        while True:
            current_time = time.time()
            elapsed_time = current_time - start_time
            
            # Check if trajectory is complete
            if elapsed_time >= self.rectangular_trajectory.total_duration:
                print(f"[{self.prefix}] Rectangular trajectory completed!")
                break
                
            # Get target position from trajectory
            target_x, target_y = self.rectangular_trajectory.get_trajectory_point(
                start_x, start_y, elapsed_time
            )
            
            # Update current position
            self.current_x = target_x
            self.current_y = target_y
            
            # Calculate inverse kinematics
            try:
                joint2, joint3 = self.kinematics.inverse_kinematics(self.current_x, self.current_y)
                self.target_positions["shoulder_lift"] = joint2
                self.target_positions["elbow_flex"] = joint3
                
                # Update wrist_flex coupling
                self.target_positions["wrist_flex"] = (
                    -self.target_positions["shoulder_lift"]
                    -self.target_positions["elbow_flex"]
                    + self.pitch
                )
                
                # Get action
                action = self.p_control_action(robot)
                
                # Determine which arm is executing and send appropriate action structure
                if self.prefix == "left":
                    # Send left arm action with empty actions for other components
                    robot_action = {**action, **{}, **{}, **{}}
                elif self.prefix == "right":
                    # Send right arm action with empty actions for other components
                    robot_action = {**{}, **action, **{}, **{}}
                
                # Send action to robot
                robot.send_action(robot_action)
                
                # Get observation and log data
                obs = robot.get_observation()
                #log_rerun_data(obs, robot_action)
                
            except Exception as e:
                print(f"[{self.prefix}] IK failed at x={self.current_x:.4f}, y={self.current_y:.4f}: {e}")
                break
                
            # Maintain control frequency
            # busy_wait(dt)
        
        print(f"[{self.prefix}] Trajectory execution finished.")

    def handle_keys(self, key_state):
        # Joint increments
        if key_state.get('shoulder_pan+'):
            self.target_positions["shoulder_pan"] += self.degree_step
            print(f"[{self.prefix}] shoulder_pan: {self.target_positions['shoulder_pan']}")
        if key_state.get('shoulder_pan-'):
            self.target_positions["shoulder_pan"] -= self.degree_step
            print(f"[{self.prefix}] shoulder_pan: {self.target_positions['shoulder_pan']}")
        if key_state.get('wrist_roll+'):
            self.target_positions["wrist_roll"] += self.degree_step
            print(f"[{self.prefix}] wrist_roll: {self.target_positions['wrist_roll']}")
        if key_state.get('wrist_roll-'):
            self.target_positions["wrist_roll"] -= self.degree_step
            print(f"[{self.prefix}] wrist_roll: {self.target_positions['wrist_roll']}")
        if key_state.get('gripper+'):
            self.target_positions["gripper"] += self.degree_step
            self.target_positions["gripper"] = min(self.target_positions["gripper"], self.gripper_max)
            print(f"[{self.prefix}] gripper: {self.target_positions['gripper']}")
        if key_state.get('gripper-'):
            self.target_positions["gripper"] -= self.degree_step
            self.target_positions["gripper"] = max(self.target_positions["gripper"], self.gripper_min)
            print(f"[{self.prefix}] gripper: {self.target_positions['gripper']}")
        if key_state.get('pitch+'):
            self.pitch += self.degree_step
            print(f"[{self.prefix}] pitch: {self.pitch}")
        if key_state.get('pitch-'):
            self.pitch -= self.degree_step
            print(f"[{self.prefix}] pitch: {self.pitch}")

        # XY plane (IK)
        moved = False
        if key_state.get('x+'):
            self.current_x += self.xy_step
            moved = True
            print(f"[{self.prefix}] x+: {self.current_x:.4f}, y: {self.current_y:.4f}")
        if key_state.get('x-'):
            self.current_x -= self.xy_step
            moved = True
            print(f"[{self.prefix}] x-: {self.current_x:.4f}, y: {self.current_y:.4f}")
        if key_state.get('y+'):
            self.current_y += self.xy_step
            moved = True
            print(f"[{self.prefix}] x: {self.current_x:.4f}, y+: {self.current_y:.4f}")
        if key_state.get('y-'):
            self.current_y -= self.xy_step
            moved = True
            print(f"[{self.prefix}] x: {self.current_x:.4f}, y-: {self.current_y:.4f}")
        if moved:
            joint2, joint3 = self.kinematics.inverse_kinematics(self.current_x, self.current_y)
            self.target_positions["shoulder_lift"] = joint2
            self.target_positions["elbow_flex"] = joint3
            print(f"[{self.prefix}] shoulder_lift: {joint2}, elbow_flex: {joint3}")

        # Wrist flex is always coupled to pitch and the other two
        self.target_positions["wrist_flex"] = (
            -self.target_positions["shoulder_lift"]
            -self.target_positions["elbow_flex"]
            + self.pitch
        )
        # print(f"[{self.prefix}] wrist_flex: {self.target_positions['wrist_flex']}")

    def p_control_action(self, robot):
        obs = robot.get_observation()
        current = {j: obs[f"{self.prefix}_arm_{j}.pos"] for j in self.joint_map}
        action = {}
        for j in self.target_positions:
            error = self.target_positions[j] - current[j]
            control = self.kp * error
            action[f"{self.joint_map[j]}.pos"] = current[j] + control
        return action
    

def main():
    # Teleop parameters
    FPS = 50
    # ip = "192.168.1.123"  # This is for zmq connection
    ip = "localhost"  # This is for local/wired connection
    robot_name = "my_xlerobot_pc"

    # For zmq connection
    # robot_config = XLerobotClientConfig(remote_ip=ip, id=robot_name)
    # robot = XLerobotClient(robot_config)    

    # For local/wired connection
    robot_config = XLerobotConfig()
    robot_config.teleop_keys = {
            # Movement
            "forward": "arrowdown",
            "backward": "arrowup",
            "left": "arrowright",
            "right": "arrowleft",
            "rotate_left": ".",
            "rotate_right": "/",
            # Speed control
            "speed_up": "rqq",
            "speed_down": "fqq",
            # quit teleop
            "quit": "qqq",
        }

    robot = XLerobot(robot_config)
    
    try:
        robot.connect()
        print(f"[MAIN] Successfully connected to robot")
    except Exception as e:
        print(f"[MAIN] Failed to connect to robot: {e}")
        print(left_arm.target_positions)
        print(robot_config)
        print(robot)
        return
        
    #_init_rerun(session_name="xlerobot_teleop_v2")

    #Init the keyboard instance
    #keyboard_config = KeyboardTeleopConfig()
    #keyboard = KeyboardTeleop(keyboard_config)
    #keyboard.connect()

    # Init the arm and head instances
    obs = robot.get_observation()
    kin_left = SO101Kinematics()
    kin_right = SO101Kinematics()
    left_arm = SimpleTeleopArm(kin_left, LEFT_JOINT_MAP, obs, prefix="left")
    right_arm = SimpleTeleopArm(kin_right, RIGHT_JOINT_MAP, obs, prefix="right")
    head_control = SimpleHeadControl(obs)

    # Move both arms and head to zero position at start
    left_arm.move_to_zero_position(robot)
    right_arm.move_to_zero_position(robot)
    global action_cmd
    try:
        while True:
            #pressed_keys = set(keyboard.get_action().keys())
            if 'arm_wrist_roll.pos' not in action_cmd:
                pressed_keys = {k: v for k, v in action_cmd.items() if v}
            print(pressed_keys)
            left_key_state = {action: (key in pressed_keys) for action, key in LEFT_KEYMAP.items()}
            right_key_state = {action: (key in pressed_keys) for action, key in RIGHT_KEYMAP.items()}

            # Handle rectangular trajectory for left arm (y key)
            if left_key_state.get('triangle'):
                print("[MAIN] Left arm rectangular trajectory triggered!")
                left_arm.execute_rectangular_trajectory(robot, fps=FPS)
                continue

            # Handle rectangular trajectory for right arm (Y key)  
            if right_key_state.get('triangle'):
                print("[MAIN] Right arm rectangular trajectory triggered!")
                right_arm.execute_rectangular_trajectory(robot, fps=FPS)
                continue

            # Handle reset for left arm
            if left_key_state.get('reset'):
                left_arm.move_to_zero_position(robot)
                continue  

            # Handle reset for right arm
            if right_key_state.get('reset'):
                right_arm.move_to_zero_position(robot)
                continue

            # Handle reset for head motors with '?'
            if '?' in pressed_keys:
                head_control.move_to_zero_position(robot)
                continue

            left_arm.handle_keys(left_key_state)
            right_arm.handle_keys(right_key_state)
            head_control.handle_keys(left_key_state)  # Head controlled by left arm keymap

            left_action = left_arm.p_control_action(robot)
            right_action = right_arm.p_control_action(robot)
            head_action = head_control.p_control_action(robot)

            # Base action
            keyboard_keys = np.array(list(pressed_keys))
            base_action = robot._from_keyboard_to_base_action(keyboard_keys) or {}

            action = {**left_action, **right_action, **head_action, **base_action}
            robot.send_action(action)

            obs = robot.get_observation()
            head1_angle = obs.get("head_motor_1.pos", 0.0)
            head2_angle = obs.get("head_motor_2.pos", 0.0)
            left_gripper_angle = obs.get("left_arm_gripper.pos", 0.0)
            right_gripper_angle = obs.get("right_arm_gripper.pos", 0.0)
            print(f"head1: {head1_angle:.2f}°head2: {head2_angle:.2f}°")
            print(f"left_angle: {left_gripper_angle:.2f}°right_angl: {right_gripper_angle:.2f}")
            # print(f"[MAIN] Observation: {obs}")
            #log_rerun_data(obs, action)
            # busy_wait(1.0 / FPS)
    finally:
        robot.disconnect()
        #keyboard.disconnect()
        print("Teleoperation ended.")

if __name__ == "__main__":
    main()
