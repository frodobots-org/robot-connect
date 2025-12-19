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
from abc import ABC, abstractmethod
#11.11
import csv
from datetime import datetime
import os
#11.11

import json
import zmq

action_cmd = {}
#new11.06
FIXED_X = 0
FIXED_Y = 0
FIXED_PITCH = 0
#new 11.06
#new10.29
last_cmd_time = time.time()
TIMEOUT = 1
#new10.29
is_running = True

ROBOT_TYPE = "xlerobot"
#ROBOT_TYPE = "lekiwi"

class TelearmsRobot(ABC):
    type: str
    x_speed: float = 0.0
    y_speed: float = 0.0
    theta_speed: float = 0.0

    def get_speed_setting(self):
        return {"x": self.x_speed, "y": self.y_speed, "theta": self.theta_speed}

    def get_axis_value(self, axes, index):
        return axes[index] if index < len(axes) else 0

    def get_button_value(self, buttons, index):
        return buttons[index] if index < len(buttons) else 0

    def handle_keyboard_action(self, data):
        action = {}
        self.x_speed = 0.1
        self.y_speed = 0.1
        self.theta_speed = 30
        try:
            for key, value in data.items():
                if not (isinstance(value, bool) and value is False):
                    action[key] = value
        except:
            pass
        return action

    @abstractmethod
    def handle_gamepad_action(self, data):
        pass

class TelearmsXlerobot(TelearmsRobot):
    type = "xlerobot"

    def handle_gamepad_action(self, data):
        action = {}

        # helpers
        btn = lambda i: self.get_button_value(data["buttons"], i)
        axis = lambda i: self.get_axis_value(data["axes"], i)

        right_mode = 0
        left_mode = 0

        # --- RIGHT HAND ---

        # combo p
        if btn(7) > 0 and btn(2) > 0:
            action["p"] = True

        # mode detect
        if btn(5) > 0 and btn(7) > 0:
            right_mode = 2
        elif btn(5) > 0:
            right_mode = 1
        elif btn(7) > 0:
            right_mode = 3

        # mode 0: base control
        if right_mode == 0:
            if axis(2) > 0.75:
                action["/"] = True
            if axis(2) < -0.75:
                action["."] = True

            self.theta_speed = 80 * abs(axis(2)) / 2

        # mode 1: arm translation
        elif right_mode == 1:
            if axis(3) > 0.8:
                action["j"] = True
            elif axis(3) < -0.8:
                action["u"] = True
            elif axis(2) > 0.8:
                action["k"] = True
            elif axis(2) < -0.8:
                action["h"] = True

            if btn(3) > 0:
                action["y"] = True
            if btn(0) > 0:
                action["i"] = True

        # mode 2: arm rotation
        elif right_mode == 2:
            if axis(3) > 0.8:
                action["g"] = True
            elif axis(3) < -0.8:
                action["t"] = True
            elif axis(2) > 0.8:
                action["6"] = True
            elif axis(2) < -0.8:
                action["5"] = True

        # mode 3: head control
        elif right_mode == 3:
            if axis(3) > 0.8:
                action["9"] = True
            elif axis(3) < -0.8:
                action["0"] = True
            elif axis(2) > 0.8:
                action["8"] = True
            elif axis(2) < -0.8:
                action["7"] = True

        # right gripper
        if btn(1) > 0:
            action["n"] = True
        if btn(2) > 0:
            action["b"] = True

        # --- LEFT HAND ---

        # combo c
        if btn(6) > 0 and btn(15) > 0:
            action["c"] = True

        # mode detect
        if btn(4) > 0 and btn(6) > 0:
            left_mode = 2
        elif btn(4) > 0:
            left_mode = 1

        # mode 0: base control
        if left_mode == 0:
            if axis(1) > 0.1:
                action["arrowdown"] = True
            if axis(1) < -0.1:
                action["arrowup"] = True
            self.x_speed = abs(axis(1)) * 0.4

            if axis(0) > 0.1:
                action["arrowright"] = True
            if axis(0) < -0.1:
                action["arrowleft"] = True
            self.y_speed = abs(axis(0)) * 0.4

        # mode 1: arm translation
        elif left_mode == 1:
            if axis(1) > 0.8:
                action["s"] = True
            elif axis(1) < -0.8:
                action["w"] = True
            elif axis(0) > 0.8:
                action["d"] = True
            elif axis(0) < -0.8:
                action["a"] = True

            if btn(12) > 0:
                action["q"] = True
            if btn(13) > 0:
                action["e"] = True

        # mode 2: arm rotation
        elif left_mode == 2:
            if axis(1) > 0.8:
                action["f"] = True
            elif axis(1) < -0.8:
                action["r"] = True
            elif axis(0) > 0.8:
                action["2"] = True
            elif axis(0) < -0.8:
                action["1"] = True

        # left gripper
        if btn(15) > 0:
            action["x"] = True
        if btn(14) > 0:
            action["z"] = True

        return action

class TelearmsLekiwi(TelearmsRobot):
    type = "lekiwi"

    def handle_gamepad_action(self, data):
        action = {}
        btn = lambda i: self.get_button_value(data["buttons"], i)
        axis = lambda i: self.get_axis_value(data["axes"], i)

        mode = 0

        # q
        if btn(7) > 0 and btn(2) > 0:
            action["q"] = True

        # mode switch
        if btn(5) > 0 and btn(7) > 0:
            mode = 2
        elif btn(5) > 0:
            mode = 1

        # mode 0: base control
        if mode == 0:
            if axis(1) > 0.1:
                action["s"] = True
            if axis(1) < -0.1:
                action["w"] = True
            self.x_speed = abs(axis(1)) * 0.6

            if axis(0) > 0.1:
                action["d"] = True
            if axis(0) < -0.1:
                action["a"] = True
            self.y_speed = abs(axis(0)) * 0.6

            if axis(2) > 0.1:
                action["x"] = True
            if axis(2) < -0.1:
                action["z"] = True
            self.theta_speed = 80 * abs(axis(2)) / 2

        # mode 1: arm translation
        elif mode == 1:
            if axis(3) > 0.8:
                action["j"] = True
            elif axis(3) < -0.8:
                action["u"] = True
            elif axis(2) > 0.8:
                action["k"] = True
            elif axis(2) < -0.8:
                action["h"] = True

            if btn(3) > 0:
                action["y"] = True
            if btn(0) > 0:
                action["i"] = True

        # mode 2: arm rotation
        elif mode == 2:
            if axis(3) > 0.8:
                action["g"] = True
            elif axis(3) < -0.8:
                action["t"] = True
            elif axis(2) > 0.8:
                action["r"] = True
            elif axis(2) < -0.8:
                action["f"] = True

        # gripper
        if btn(1) > 0:
            action["n"] = True
        if btn(2) > 0:
            action["b"] = True
        return action

if ROBOT_TYPE == "lekiwi":
    telearms_robot = TelearmsLekiwi()
else:
    telearms_robot = TelearmsXlerobot()

def get_action_cmd(msg):
    global telearms_robot
    action_cmd = {}
    if msg["type"] == "gamepad":
        action_cmd = telearms_robot.handle_gamepad_action(msg["data"])
    elif msg["type"] == "keyboard":
        action_cmd = telearms_robot.handle_keyboard_action(msg["data"])

    return action_cmd

def zmq_thread():
    zmq_context = zmq.Context()
    zmq_cmd_socket = zmq_context.socket(zmq.PULL)
    zmq_cmd_socket.setsockopt(zmq.CONFLATE, 1)
    zmq_cmd_socket.bind(f"tcp://*:5558")
    global action_cmd,last_cmd_time,is_running  #new10.29
    while is_running:
        try:
            msg = zmq_cmd_socket.recv_string()
            action_cmd = get_action_cmd(dict(json.loads(msg)))
            last_cmd_time = time.time() #new10.29
        except zmq.Again:
            #action_cmd = {}
            pass
        except Exception as e:
            action_cmd = {}
            print("Message fetching failed: %s", e)

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

SINGLE_ARM_JOINT_MAP = {
    "shoulder_pan": "arm_shoulder_pan",
    "shoulder_lift": "arm_shoulder_lift",
    "elbow_flex": "arm_elbow_flex",
    "wrist_flex": "arm_wrist_flex",
    "wrist_roll": "arm_wrist_roll",
    "gripper": "arm_gripper",
}
# Head motor mapping
HEAD_MOTOR_MAP = {
    "head_motor_1": "head_motor_1",
    "head_motor_2": "head_motor_2",
}

LEKIWI_BASE_KEYMAP = {
    "forward": "w",
    "backward": "s",
    "left": "a",
    "right": "d",
    "rotate_left": "z",
    "rotate_right": "x",
    "speed_up": "rqq",
    "speed_down": "fqq",
    "quit": "qqq",
}

XLEROBOT_BASE_KEYMAP = {
    "forward": "arrowdown",
    "backward": "arrowup",
    "left": "arrowright",
    "right": "arrowleft",
    "rotate_left": ".",
    "rotate_right": "/",
    "speed_up": "rqq",
    "speed_down": "fqq",
    "quit": "qqq",
}

LEKIWI_DATA_ORDER = [
    'arm_shoulder_pan.pos',
    'arm_shoulder_lift.pos',
    'arm_elbow_flex.pos',
    'arm_wrist_flex.pos',
    'arm_wrist_roll.pos',
    'arm_gripper.pos',
    'x.vel',
    'y.vel',
    'theta.vel'
]

XLEROBOT_DATA_ORDER = [
    'left_arm_shoulder_pan.pos',
    'left_arm_shoulder_lift.pos',
    'left_arm_elbow_flex.pos',
    'left_arm_wrist_flex.pos',
    'left_arm_wrist_roll.pos',
    'left_arm_gripper.pos',
    'right_arm_shoulder_pan.pos',
    'right_arm_shoulder_lift.pos',
    'right_arm_elbow_flex.pos',
    'right_arm_wrist_flex.pos',
    'right_arm_wrist_roll.pos',
    'right_arm_gripper.pos',
    'head_motor_1.pos',
    'head_motor_2.pos',
    'x.vel',
    'y.vel',
    'theta.vel'
]

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
    def __init__(self, kinematics, joint_map, initial_obs, prefix="left_", kp=0.81):
        self.kinematics = kinematics
        self.joint_map = joint_map
        self.prefix = prefix  # To distinguish left and right arm
        self.kp = kp
        self.gripper_min = 0.0
        self.gripper_max = 95.0
        # Initial joint positions
        self.joint_positions = {
            "shoulder_pan": initial_obs[f"{prefix}arm_shoulder_pan.pos"],
            "shoulder_lift": initial_obs[f"{prefix}arm_shoulder_lift.pos"],
            "elbow_flex": initial_obs[f"{prefix}arm_elbow_flex.pos"],
            "wrist_flex": initial_obs[f"{prefix}arm_wrist_flex.pos"],
            "wrist_roll": initial_obs[f"{prefix}arm_wrist_roll.pos"],
            "gripper": initial_obs[f"{prefix}arm_gripper.pos"],
        }
        # Set initial x/y to fixed values
        self.current_x = 0.1629
        self.current_y = 0.1131
        self.pitch = 0.0
        # Set the degree step and xy step
        self.degree_step = 1
        self.xy_step = 0.0042
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
        # FIXME: lekiwi not handle data without base command
        action['x.vel'] = 0
        action['y.vel'] = 0
        action['theta.vel'] = 0
        robot.send_action(action)

    #new11.06
    def move_to_fixed_position(self, robot):
        if self.prefix == "left":
            self.current_x = FIXED_X
            self.current_y = FIXED_Y
            self.pitch = FIXED_PITCH
        else:
            self.current_x = FIXED_X
            self.current_y = FIXED_Y
            self.pitch = FIXED_PITCH
        
        joint2, joint3 = self.kinematics.inverse_kinematics(self.current_x, self.current_y)
        self.target_positions["shoulder_lift"] = joint2
        self.target_positions["elbow_flex"] = joint3

        self.target_positions["wrist_flex"] = -joint2 - joint3 + self.pitch
        
        action = self.p_control_action(robot)
        # FIXME: lekiwi not handle data without base command
        action['x.vel'] = 0
        action['y.vel'] = 0
        action['theta.vel'] = 0
        robot.send_action(action)
    #new11.06

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
        current = {j: obs[f"{self.prefix}arm_{j}.pos"] for j in self.joint_map}
        action = {}
        for j in self.target_positions:
            error = self.target_positions[j] - current[j]
            control = self.kp * error
            action[f"{self.joint_map[j]}.pos"] = current[j] + control
        return action

class SimpleBaseControl:
    def __init__(self, keymap):
        self.teleop_keys = keymap
        self.speed_levels = [
            {"xy": 0.8, "theta": 90},  # slow
            {"xy": 0.2, "theta": 60},  # medium
            {"xy": 0.3, "theta": 90},  # fast
        ]
        self.speed_index = 0  # Start at slow
    def _from_keyboard_to_base_action(self, pressed_keys: np.ndarray, speed_setting): 
        x_speed = speed_setting["x"]
        y_speed = speed_setting["y"]
        theta_speed = speed_setting["theta"]

        x_cmd = 0.0  # m/s forward/backward
        y_cmd = 0.0  # m/s lateral
        theta_cmd = 0.0  # deg/s rotation

        if self.teleop_keys["forward"] in pressed_keys:
            x_cmd = x_speed
        if self.teleop_keys["backward"] in pressed_keys:
            x_cmd = -x_speed
        if self.teleop_keys["left"] in pressed_keys:
            y_cmd = y_speed
        if self.teleop_keys["right"] in pressed_keys:
            y_cmd = -y_speed
        if self.teleop_keys["rotate_left"] in pressed_keys:
            theta_cmd = theta_speed
        if self.teleop_keys["rotate_right"] in pressed_keys:
            theta_cmd = -theta_speed
        return {
            "x.vel": x_cmd,
            "y.vel": y_cmd,
            "theta.vel": theta_cmd,
        }

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
    if ROBOT_TYPE == "lekiwi":
        # FIXME: different code base
        from lerobot.robots.lekiwi import LeKiwi, LeKiwiConfig
        from lerobot.utils.robot_utils import busy_wait
        from lerobot.model.SO101Robot import SO101Kinematics
        robot_config = LeKiwiConfig()  
        robot = LeKiwi(robot_config)
        all_motors = list(robot.bus.motors.keys())
    else:
        from lerobot.robots.xlerobot import XLerobot, XLerobotConfig
        from lerobot.utils.robot_utils import busy_wait
        from lerobot.model.SO101Robot import SO101Kinematics

        robot_config = XLerobotConfig()
        robot = XLerobot(robot_config)
        all_motors = list(robot.bus1.motors.keys()) + list(robot.bus2.motors.keys())

    try:
        robot.connect()
        print(f"[MAIN] Successfully connected to robot")
    except Exception as e:
        print(f"[MAIN] Failed to connect to robot: {e}")
        print(left_arm.target_positions)
        print(robot_config)
        print(robot)
        return
    

    action_thread = threading.Thread(target=zmq_thread, daemon=True)
    action_thread.start()

    #11.11 
    current_fields = [f"{motor}_current" for motor in all_motors]
    temp_fields = [f"{motor}_temp" for motor in all_motors]
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    save_dir = "motor_current_data"
    os.makedirs(save_dir, exist_ok=True)
    csv_path = os.path.join(save_dir, f"current_data_{timestamp}.csv")

    with open(csv_path, mode='w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=["timestamp"] + current_fields + temp_fields)
        writer.writeheader()

    csv_file = open(csv_path, mode='a', newline='')
    csv_writer = csv.DictWriter(csv_file, fieldnames=["timestamp"] + current_fields + temp_fields)

    #Read the angle
    angle_save_dir = "motor_angle_data"
    os.makedirs(angle_save_dir, exist_ok=True)
    angle_fields = ["timestamp", "obs"]
    angle_csv_path = os.path.join(angle_save_dir, f"angle_data_{timestamp}.csv")

    #with open(angle_csv_path, mode='w', newline='') as f:
    #    angle_writer = csv.DictWriter(f, fieldnames=angle_fields)
    #    angle_writer.writeheader()
    #angle_csv_file = open(angle_csv_path, mode='a', newline='')
    #angle_csv_writer = csv.DictWriter(angle_csv_file, fieldnames=angle_fields)
    #11.11
        
    #_init_rerun(session_name="xlerobot_teleop_v2")

    #Init the keyboard instance
    #keyboard_config = KeyboardTeleopConfig()
    #keyboard = KeyboardTeleop(keyboard_config)
    #keyboard.connect()

    # Init the arm and head instances
    obs = robot.get_observation()

    right_arm = None
    left_arm = None
    head_control = None

    # Lekiwi or XLerobot
    if ROBOT_TYPE == "lekiwi":
        kin_right = SO101Kinematics()
        right_arm = SimpleTeleopArm(kin_right, SINGLE_ARM_JOINT_MAP, obs, prefix="")
        base_control = SimpleBaseControl(LEKIWI_BASE_KEYMAP)
    else:
        kin_right = SO101Kinematics()
        right_arm = SimpleTeleopArm(kin_right, RIGHT_JOINT_MAP, obs, prefix="right_")
        kin_left = SO101Kinematics()
        left_arm = SimpleTeleopArm(kin_left, LEFT_JOINT_MAP, obs, prefix="left_")
        head_control = SimpleHeadControl(obs)
        base_control = SimpleBaseControl(XLEROBOT_BASE_KEYMAP)

    # Move both arms and head to zero position at start
    if right_arm:
        right_arm.move_to_zero_position(robot)
    if left_arm:
        left_arm.move_to_zero_position(robot)

    global action_cmd

    #11.11Temperature alarm
    temp_threshold = 65
    consecutive_over_temp = 0
    over_temp_alert = False
    cooldown_needed = False
    #11.11Temperature alarm

    #Electric Current Warning
    current_high_threshold = 400
    current_medium_threshold = 200 
    consecutive_over_current = 0 

    # report data to teleop agent
    context = zmq.Context()
    socket = context.socket(zmq.PUSH)
    socket.bind("tcp://*:5559")

    try:
        while True:
            #new 10.29
            current_time = time.time()
            if current_time - last_cmd_time > TIMEOUT:
                #11.06
                if left_arm:
                    left_arm.move_to_fixed_position(robot)
                if right_arm:
                    right_arm.move_to_fixed_position(robot)
                #11.06
                time.sleep(1.0 / FPS)
                continue
                #new 10.29
            #pressed_keys = set(keyboard.get_action().keys())
            if 'arm_wrist_roll.pos' not in action_cmd:
                pressed_keys = {k: v for k, v in action_cmd.items() if v}
            print(pressed_keys)
            left_key_state = {action: (key in pressed_keys) for action, key in LEFT_KEYMAP.items()}
            right_key_state = {action: (key in pressed_keys) for action, key in RIGHT_KEYMAP.items()}

            # 11.11Temperature alarm
            if cooldown_needed:
                all_below_threshold = all(temp <= temp_threshold for temp in all_temp.values())
                
                if all_below_threshold:
                    print(f"[ALERT] The temperature has dropped to a safe level ({temp_threshold}°C以下)，恢复控制")
                    cooldown_needed = False
                    consecutive_over_temp = 0 
                else:
                    print(f"[ALERT] Wait for the temperature to cool down, the current maximum temperature: {max(all_temp.values())}°C")
                    time.sleep(1) 
                    continue 
            # 11.11Temperature alarm

            # Handle rectangular trajectory for left arm (y key)
            if left_key_state.get('triangle') and left_arm:
                print("[MAIN] Left arm rectangular trajectory triggered!")
                left_arm.execute_rectangular_trajectory(robot, fps=FPS)
                continue

            # Handle rectangular trajectory for right arm (Y key)  
            if right_key_state.get('triangle') and right_arm:
                print("[MAIN] Right arm rectangular trajectory triggered!")
                right_arm.execute_rectangular_trajectory(robot, fps=FPS)
                continue

            # Handle reset for left arm
            if left_key_state.get('reset') and left_arm:
                left_arm.move_to_zero_position(robot)
                continue  

            # Handle reset for right arm
            if right_key_state.get('reset') and right_arm:
                right_arm.move_to_zero_position(robot)
                continue

            # Handle reset for head motors with '?'
            if '?' in pressed_keys and head_control:
                head_control.move_to_zero_position(robot)
                continue

            left_action = {}
            right_action = {}
            head_action = {}

            if left_arm:
                left_arm.handle_keys(left_key_state)
                left_action = left_arm.p_control_action(robot)
            if right_arm:
                right_arm.handle_keys(right_key_state)
                right_action = right_arm.p_control_action(robot)
            if head_control:
                head_control.handle_keys(left_key_state)  # Head controlled by left arm keymap
                head_action = head_control.p_control_action(robot)

            # Base action
            keyboard_keys = np.array(list(pressed_keys))
            global telearms_robot
            base_action = base_control._from_keyboard_to_base_action(keyboard_keys, telearms_robot.get_speed_setting()) or {}

            action = {**left_action, **right_action, **head_action, **base_action}
            robot.send_action(action)

            obs = robot.get_observation()

            if ROBOT_TYPE == "lekiwi":
                report = {'obs': [float(obs[k]) for k in LEKIWI_DATA_ORDER],
                    'act': [float(action[k]) for k in LEKIWI_DATA_ORDER]}
            else:
                report = {'obs': [float(obs[k]) for k in XLEROBOT_DATA_ORDER],
                    'act': [float(action[k]) for k in XLEROBOT_DATA_ORDER]}

            # report
            socket.send_string(json.dumps(report))

            angle_values = []
            act_values = []
            for motor in all_motors:
                angle = obs.get(f"{motor}.pos", 0.0)
                angle_values.append(angle)
                act_val = action.get(f"{motor}.pos", angle) 
                act_values.append(act_val)
            
            current_time_str = datetime.now().strftime("%Y%m%d_%H%M%S.%f")
            angle_row = {
                "timestamp": current_time_str,
                "obs": json.dumps({"obs": angle_values, "act": act_values})
            }
            #angle_csv_writer.writerow(angle_row)

            if ROBOT_TYPE == "lekiwi":
                current_bus = robot.bus.sync_read("Present_Current")
                all_current = {**current_bus}

                temp_bus = robot.bus.sync_read("Present_Temperature")
                all_temp = {**temp_bus}
            else:
                current_bus1 = robot.bus1.sync_read("Present_Current")
                current_bus2 = robot.bus2.sync_read("Present_Current")
                all_current = {**current_bus1, **current_bus2}

                temp_bus1 = robot.bus1.sync_read("Present_Temperature")
                temp_bus2 = robot.bus2.sync_read("Present_Temperature")
                all_temp = {**temp_bus1, **temp_bus2}
            
            current_time_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            '''
            print(f"\n[{current_time_str}] motor current and temperature:")
            for motor in all_motors:
                print(f"  {motor}: current={all_current.get(motor, 0)}mA, temp={all_temp.get(motor, 0)}°C", end="; ")
            print()
            '''
            csv_row = {"timestamp": current_time_str}
            for motor in all_motors:
                csv_row[f"{motor}_current"] = all_current.get(motor, 0)
            for motor in all_motors:
                csv_row[f"{motor}_temp"] = all_temp.get(motor, 0)

            csv_writer.writerow(csv_row)
            #11.11

            #Electric Current Warning

            any_over_high_current = any(current > current_high_threshold for current in all_current.values())
            any_over_medium_current = any(current > current_medium_threshold for current in all_current.values())

            if any_over_high_current:
                print(f"[ALERT] Detected that the current exceeds{current_high_threshold}mA,Stop control!")
                robot.bus1.disable_torque()
                robot.bus2.disable_torque()
                time.sleep(10)

                robot.bus1.enable_torque()
                robot.bus2.enable_torque()
                time.sleep(5)
 
                left_arm.move_to_fixed_position(robot)
                right_arm.move_to_fixed_position(robot)
                print(f"[ALERT] Power-on is completed and reset")
            
                consecutive_over_current = 0
            elif any_over_medium_current:
                consecutive_over_current += 1
                print(f"[WARNING] Detected that the current exceeds{current_medium_threshold}mA,counting: {consecutive_over_current}")
                if consecutive_over_current >= 3:
                    print(f"[ALERT] Three consecutive times the current exceeds{current_medium_threshold}mA,Stop control!")

                    robot.bus1.disable_torque()
                    robot.bus2.disable_torque()
                    time.sleep(10)
                    
                    robot.bus1.enable_torque()
                    robot.bus2.enable_torque()
                    time.sleep(10)
                    
                    left_arm.move_to_fixed_position(robot)
                    right_arm.move_to_fixed_position(robot)
                    print(f"[ALERT] Power-on is completed and reset")
                
                    consecutive_over_current = 0
            else:
                consecutive_over_current = 0



            # 11.11Temperature alarm
            any_over_temp = any(temp > temp_threshold for temp in all_temp.values())
            
            if any_over_temp:
                consecutive_over_temp += 1
                print(f"[WARNING] temperature exceeds{temp_threshold}°C,counting: {consecutive_over_temp}")
                
                if consecutive_over_temp >= 3:
                    print(f"[ALERT] Three consecutive times the temperature exceeds{temp_threshold}°C,The robotic arm stops.")

                    if ROBOT_TYPE == "lekiwi":
                        robot.bus.disable_torque()
                        time.sleep(10)
                        robot.bus.enable_torque()
                        time.sleep(5)
                        right_arm.move_to_zero_position(robot)
                        time.sleep(10)
                        right_arm.move_to_fixed_position(robot)
                        print(f"[ALERT] The robotic arm resets and pauses for 10 seconds.")
                        cooldown_needed = True

                    else:
                        robot.bus1.disable_torque()
                        robot.bus2.disable_torque()
                        time.sleep(10)
                        robot.bus1.enable_torque()
                        robot.bus2.enable_torque()
                        time.sleep(5)
                        left_arm.move_to_zero_position(robot)
                        right_arm.move_to_zero_position(robot)
                        time.sleep(10)
                        left_arm.move_to_fixed_position(robot)
                        right_arm.move_to_fixed_position(robot)
                        head_control.move_to_zero_position(robot)
                        print(f"[ALERT] The robotic arm resets and pauses for 10 seconds.")
                        cooldown_needed = True
            else:

                consecutive_over_temp = 0


            if ROBOT_TYPE == "lekiwi":
                pass
            else:
                obs = robot.get_observation()
                head1_angle = obs.get("head_motor_1.pos", 0.0)
                head2_angle = obs.get("head_motor_2.pos", 0.0)
                left_gripper_angle = obs.get("left_arm_gripper.pos", 0.0)
                right_gripper_angle = obs.get("right_arm_gripper.pos", 0.0)
                print(f"head1: {head1_angle:.2f}°head2: {head2_angle:.2f}°")
                print(f"left_angle: {left_gripper_angle:.2f}°right_angl: {right_gripper_angle:.2f}")
            # print(f"[MAIN] Observation: {obs}")
            #log_rerun_data(obs, action)
            busy_wait(1.0 / FPS)
    finally:
        #11.11
        csv_file.close()
        #11.11
        #angle_csv_file.close()
        robot.disconnect()
        #keyboard.disconnect()
        global is_running
        is_running = False
        action_thread.join()
        print("Teleoperation ended.")

if __name__ == "__main__":
    main()
