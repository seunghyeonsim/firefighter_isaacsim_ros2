#!/home/van/isaacsim/python.sh

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import sys
import os

# scripts 디렉토리를 Python 경로에 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)

import carb
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path

from firefighter_articulation import FirefighterArticulation

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from firefighter_interfaces.msg import FlipperCommand

class FirefighterIsaacSim(Node):
    def __init__(self):
        super().__init__('firefighter_isaacsim')
        
        # Publishers
        self.status_pub = self.create_publisher(String, 'firefighter/status', 10)
        self.joint_state_pub = self.create_publisher(JointState, 'firefighter/joint_states', 10)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.flipper_cmd_sub = self.create_subscription(FlipperCommand, 'firefighter/flipper_command', self.flipper_cmd_callback, 10)
        
        # Timer for simulation stepping and status publishing
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100Hz
        
        # Isaac Sim objects
        self.my_world = None
        self.firefighter = None
        
        # Initialize Isaac Sim
        self.init_isaac_sim()
        
        self.get_logger().info('FirefighterIsaacSim node initialized!')
        self.get_logger().info('Ready to receive commands via ROS2 topics')

    def init_isaac_sim(self):
        """Initialize Isaac Sim simulation"""
        try:
            self.get_logger().info('Initializing Isaac Sim...')
            self.my_world, self.firefighter = FirefighterArticulation.setup_scene()
            self.get_logger().info('Isaac Sim initialization completed successfully!')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize Isaac Sim: {str(e)}')
            raise

    def cmd_vel_callback(self, msg):
        """Handle Twist commands for differential drive"""
        if self.firefighter is None:
            self.get_logger().warn('Firefighter not initialized yet')
            return
        
        try:
            # Extract linear and angular velocities
            linear_speed = msg.linear.x
            angular_speed = msg.angular.z
            
            # Apply differential drive command
            self.firefighter.move_differential(linear_speed, angular_speed)
            
            self.get_logger().info(f'Applied cmd_vel - linear: {linear_speed}, angular: {angular_speed}')
            
        except Exception as e:
            self.get_logger().error(f'Error in cmd_vel_callback: {str(e)}')

    def flipper_cmd_callback(self, msg):
        """Handle FlipperCommand messages for individual flipper control"""
        if self.firefighter is None:
            self.get_logger().warn('Firefighter not initialized yet')
            return
        
        try:
            command_type = msg.command_type.lower()
            
            if command_type == "stop":
                # Stop all flippers
                self.firefighter.stop_flippers()
                self.get_logger().info('Stopped all flippers')
                
            elif command_type == "all":
                # Set all flippers to the same velocity (use fl_velocity as common value)
                velocity = msg.fl_velocity
                self.firefighter.set_all_flippers_velocity(velocity)
                self.get_logger().info(f'Set all flippers velocity to {velocity}')
                
            elif command_type == "individual":
                # Set individual flipper velocities
                if msg.fl_velocity != 0.0:
                    self.firefighter.set_front_left_flipper_velocity(msg.fl_velocity)
                    self.get_logger().info(f'Set front left flipper velocity to {msg.fl_velocity}')
                    
                if msg.fr_velocity != 0.0:
                    self.firefighter.set_front_right_flipper_velocity(msg.fr_velocity)
                    self.get_logger().info(f'Set front right flipper velocity to {msg.fr_velocity}')
                    
                if msg.rl_velocity != 0.0:
                    self.firefighter.set_rear_left_flipper_velocity(msg.rl_velocity)
                    self.get_logger().info(f'Set rear left flipper velocity to {msg.rl_velocity}')
                    
                if msg.rr_velocity != 0.0:
                    self.firefighter.set_rear_right_flipper_velocity(msg.rr_velocity)
                    self.get_logger().info(f'Set rear right flipper velocity to {msg.rr_velocity}')
                    
            else:
                self.get_logger().warn(f'Unknown command type: {command_type}')
                
        except Exception as e:
            self.get_logger().error(f'Error in flipper_cmd_callback: {str(e)}')

    def timer_callback(self):
        """Timer callback for simulation stepping and status publishing"""
        try:
            # Step the simulation
            if self.my_world is not None:
                self.my_world.step(render=True)
            
            # Publish joint states
            if self.firefighter is not None:
                self.publish_joint_states()
                
            # Publish status
            self.publish_status()
            
        except Exception as e:
            self.get_logger().error(f'Error in timer_callback: {str(e)}')

    def publish_joint_states(self):
        """Publish current joint states"""
        try:
            if hasattr(self.firefighter, 'joint_names') and self.firefighter.joint_names:
                joint_state = JointState()
                joint_state.header.stamp = self.get_clock().now().to_msg()
                joint_state.name = self.firefighter.joint_names
                
                # Get joint positions and velocities
                positions = self.firefighter.get_joint_positions()
                velocities = self.firefighter.get_joint_velocities()
                
                if positions is not None:
                    joint_state.position = positions[0].tolist()
                if velocities is not None:
                    joint_state.velocity = velocities[0].tolist()
                
                self.joint_state_pub.publish(joint_state)
                
        except Exception as e:
            self.get_logger().error(f'Error publishing joint states: {str(e)}')

    def publish_status(self):
        """Publish current status"""
        try:
            status_msg = String()
            status_msg.data = "Simulation running"
            self.status_pub.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing status: {str(e)}')

def main():
    rclpy.init()
    try:
        node = FirefighterIsaacSim()
        print("FirefighterIsaacSim node created successfully!")
        
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            
    except KeyboardInterrupt:
        print("Exiting simulation...")
        simulation_app.close()
    except Exception as e:
        print(f"Error in main: {str(e)}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 