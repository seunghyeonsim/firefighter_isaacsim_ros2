# -*- coding: utf-8 -*-
"""
====================================
@File Name ：firefighter_articulation.py
@Time ： 2025/08/04
@Create by Author ： Seunghyeon Sim
====================================
"""

import numpy as np
import os
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.prims import Articulation
from isaacsim.core.api.controllers.articulation_controller import ArticulationController
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController


class FirefighterArticulation(Articulation):
    """
    Firefighter robot articulation class for Isaac Sim
    """
    
    def __init__(self, prim_paths_expr, name="firefighter"):
        super().__init__(prim_paths_expr=prim_paths_expr, name=name)
        
        # Initialize joint indices after world reset
        self.left_wheel_joint_indices = []
        self.right_wheel_joint_indices = []
        
        # Initialize articulation controller
        self.articulation_controller = ArticulationController()
        
        # Initialize differential controller
        wheel_radius = 0.176  # 17.6cm
        wheel_base = 0.906  # 90.6cm
        self.differential_controller = DifferentialController("firefighter_controller", wheel_radius, wheel_base)
        
    @staticmethod
    def setup_scene():
        """Setup the simulation scene with firefighter robot"""
        # Create world
        my_world = World(stage_units_in_meters=1.0)
        my_world.scene.add_default_ground_plane()  # add ground plane
        
        # Load firefighter scene (contains environment + firefighter)
        # Try multiple possible paths for the scene file
        possible_paths = [
            # When running from source directory
            os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "usd", "firefighter_scene.usd"),
            # When running from install directory via ros2 run
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..", "usd", "firefighter_scene.usd"),
            # Alternative install path
            "/home/van/firefighter_ws/src/firefighter_description/usd/firefighter_scene.usd"
        ]
        
        scene_path = None
        for path in possible_paths:
            if os.path.exists(path):
                scene_path = path
                print(f"Found firefighter_scene.usd at: {scene_path}")
                break
        
        if scene_path is None:
            raise FileNotFoundError(f"Could not find firefighter_scene.usd in any of the expected locations: {possible_paths}")
        
        add_reference_to_stage(usd_path=scene_path, prim_path="/World")

        # Use fixed firefighter path - this is the actual path found in the scene
        firefighter_path = "/World/base_footprint"
        print(f"Using firefighter path: {firefighter_path}")

        # initialize the world first
        my_world.reset()

        # Control the firefighter in the scene (create after scene is loaded)
        firefighter = FirefighterArticulation(prim_paths_expr=firefighter_path, name="my_firefighter")

        # set the initial pose of the firefighter
        firefighter.set_world_poses(positions=np.array([[0.0, 0.0, 0.15]]) / get_stage_units())

        # Initialize joint indices after world reset
        firefighter.initialize_joints()

        # Wait a bit for articulation to be fully initialized and physics to stabilize
        print("Waiting for articulation to initialize and physics to stabilize...")
        for j in range(50):  # Increased from 10 to 50 steps
            my_world.step(render=True)
        
        # Initialize articulation controller
        print("Initializing articulation controller...")
        firefighter.articulation_controller.initialize(firefighter)
        print("Articulation controller initialized successfully!")
            
        return my_world, firefighter
        
    def initialize_joints(self):
        """Initialize joint indices after the world is reset"""
        self._find_joint_indices()
        
    def _find_joint_indices(self):
        """Find and categorize joint indices"""
        self.left_wheel_joint_indices = []
        self.right_wheel_joint_indices = []
        self.flipper_joint_indices = {}
        
        # Check if joint_names is available
        if not hasattr(self, 'joint_names') or self.joint_names is None:
            print("Warning: joint_names not available yet")
            return
            
        for i, joint_name in enumerate(self.joint_names):
            # Find wheel joints
            if 'wheel' in joint_name.lower():
                if 'FL' in joint_name or 'RL' in joint_name:
                    self.left_wheel_joint_indices.append(i)
                    print(f"Found left wheel joint: {joint_name} at index {i}")
                elif 'FR' in joint_name or 'RR' in joint_name:
                    self.right_wheel_joint_indices.append(i)
                    print(f"Found right wheel joint: {joint_name} at index {i}")
            
            # Find flipper joints
            elif 'flipper' in joint_name.lower():
                self.flipper_joint_indices[joint_name] = i
                print(f"Found flipper joint: {joint_name} at index {i}")
        
        print(f"Left wheel joint indices: {self.left_wheel_joint_indices}")
        print(f"Right wheel joint indices: {self.right_wheel_joint_indices}")
        print(f"Flipper joint indices: {self.flipper_joint_indices}")
    
    def set_flippers_velocity(self, fl_velocity=0.0, fr_velocity=0.0, rl_velocity=0.0, rr_velocity=0.0):
        """Set flipper velocities using ArticulationController"""
        if not self.flipper_joint_indices:
            print("Warning: Flipper joint indices not initialized")
            return
        
        # Create velocity array for all joints
        velocities = np.zeros(self.num_joints)
        
        # Set flipper velocities
        flipper_velocities = {
            "Flipper_FL": fl_velocity,
            "Flipper_FR": fr_velocity,
            "Flipper_RL": rl_velocity,
            "Flipper_RR": rr_velocity
        }
        
        for flipper_name, velocity in flipper_velocities.items():
            if flipper_name in self.flipper_joint_indices:
                idx = self.flipper_joint_indices[flipper_name]
                velocities[idx] = velocity
        
        # Apply using articulation controller
        self.apply_articulation_action(joint_velocities=velocities)
    
    def set_flipper_velocity(self, flipper_name, velocity=0.0):
        """Set individual flipper velocity using ArticulationController"""
        if not self.flipper_joint_indices or flipper_name not in self.flipper_joint_indices:
            print(f"Warning: Flipper {flipper_name} not found")
            return
        
        # Create velocity array for all joints
        velocities = np.zeros(self.num_joints)
        
        # Set specific flipper velocity
        idx = self.flipper_joint_indices[flipper_name]
        velocities[idx] = velocity
        
        # Apply using articulation controller
        self.apply_articulation_action(joint_velocities=velocities)
        print(f"Set {flipper_name} velocity to {velocity}")
    
    def set_all_flippers_velocity(self, velocity=0.0):
        """Set all flippers to the same velocity using ArticulationController"""
        self.set_flippers_velocity(velocity, velocity, velocity, velocity)
        print(f"Set all flippers velocity to {velocity}")
    
    def stop_flippers(self):
        """Stop all flippers using ArticulationController"""
        self.set_flippers_velocity(0.0, 0.0, 0.0, 0.0)
        print("Set all flippers velocity to 0.0")
    
    def set_front_left_flipper_velocity(self, velocity):
        """Set front left flipper velocity using ArticulationController"""
        self.set_flipper_velocity("Flipper_FL", velocity)
    
    def set_front_right_flipper_velocity(self, velocity):
        """Set front right flipper velocity using ArticulationController"""
        self.set_flipper_velocity("Flipper_FR", velocity)
    
    def set_rear_left_flipper_velocity(self, velocity):
        """Set rear left flipper velocity using ArticulationController"""
        self.set_flipper_velocity("Flipper_RL", velocity)
    
    def set_rear_right_flipper_velocity(self, velocity):
        """Set rear right flipper velocity using ArticulationController"""
        self.set_flipper_velocity("Flipper_RR", velocity)
    
    def get_all_flipper_positions(self):
        """Get positions of all flipper joints"""
        positions = self.get_joint_positions()
        flipper_positions = {}
        for flipper_name, idx in self.flipper_joint_indices.items():
            flipper_positions[flipper_name] = positions[0, idx]
        return flipper_positions
    
    def set_wheel_velocities(self, left_velocity=0.0, right_velocity=0.0):
        """Set wheel velocities using ArticulationController"""
        if not self.left_wheel_joint_indices or not self.right_wheel_joint_indices:
            print("Warning: Wheel joint indices not initialized")
            return
        
        # Create velocity array for all joints
        velocities = np.zeros(self.num_joints)
        
        # Set left wheel velocities
        for idx in self.left_wheel_joint_indices:
            velocities[idx] = left_velocity
        
        # Set right wheel velocities
        for idx in self.right_wheel_joint_indices:
            velocities[idx] = right_velocity
        
        # Apply using articulation controller
        self.apply_articulation_action(joint_velocities=velocities)

    def apply_articulation_action(self, joint_positions=None, joint_velocities=None, joint_efforts=None, joint_indices=None):
        """Apply articulation action using ArticulationController"""
        if not hasattr(self, 'articulation_controller') or self.articulation_controller is None:
            print("Warning: Articulation controller not initialized")
            return
        
        # Create ArticulationAction
        action = ArticulationAction(
            joint_positions=joint_positions,
            joint_velocities=joint_velocities,
            joint_efforts=joint_efforts,
            joint_indices=joint_indices
        )
        
        # Apply the action
        self.articulation_controller.apply_action(action)
        print(f"Applied articulation action - positions: {joint_positions}, velocities: {joint_velocities}, efforts: {joint_efforts}")
    
    def move_differential(self, linear_speed=0.0, angular_speed=0.0):
        """Move robot using differential controller"""
        if not hasattr(self, 'differential_controller'):
            print("Warning: Differential controller not initialized")
            return
        angular_constant = 10
        # Create command for differential controller
        command = [linear_speed, angular_speed*angular_constant]
        
        # Get wheel actions from differential controller
        actions = self.differential_controller.forward(command)
        
        # Print the actions for debugging
        print(f"Differential controller actions - linear: {linear_speed}, angular: {angular_speed}")
        print(f"Wheel actions: {actions}")
        
        # Apply actions to articulation controller
        if actions is not None and hasattr(actions, 'joint_velocities') and actions.joint_velocities is not None:
            velocities = actions.joint_velocities
            if len(velocities) >= 2:
                left_wheel_velocity = velocities[0]  # Left wheel velocity
                right_wheel_velocity = velocities[1]  # Right wheel velocity
                
                # Use existing set_wheel_velocities function
                self.set_wheel_velocities(left_wheel_velocity, right_wheel_velocity)
                print(f"Applied wheel velocities - Left: {left_wheel_velocity}, Right: {right_wheel_velocity}")
            else:
                print("Warning: Invalid velocities from differential controller")
        else:
            print("Warning: Invalid actions from differential controller")
    
    def move_forward_differential(self, speed=0.3):
        """Move robot forward using differential controller"""
        self.move_differential(linear_speed=speed, angular_speed=0.0)
    
    def move_backward_differential(self, speed=0.3):
        """Move robot backward using differential controller"""
        self.move_differential(linear_speed=-speed, angular_speed=0.0)
    
    def turn_left_differential(self, speed=1.0):
        """Turn robot left using differential controller"""
        self.move_differential(linear_speed=0.0, angular_speed=speed)
    
    def turn_right_differential(self, speed=1.0):
        """Turn robot right using differential controller"""
        self.move_differential(linear_speed=0.0, angular_speed=-speed)
    
    def stop_differential(self):
        """Stop robot using differential controller"""
        self.move_differential(linear_speed=0.0, angular_speed=0.0) 