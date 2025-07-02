#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
import math
import numpy as np

class FourWheelSteeringController(Node):
    def __init__(self):
        super().__init__('four_wheel_steering_controller')
        
        # Publishers
        self.pub_pos = self.create_publisher(Float64MultiArray, '/steering_controller', 1)
        self.pub_vel = self.create_publisher(Float64MultiArray, '/wheel_motors_controller', 1)
        
        # Subscriber
        self.subscription = self.create_subscription(Joy, '/joy', self.listenerCallback, 1)
        
        # Vehicle parameters (adjust according to your robot)
        self.wheelbase = 0.900  # Distance between front and rear axles (meters)
        self.track_width = 1.082  # Distance between left and right wheels (meters)
        self.max_steering_angle = math.pi/3  # Maximum steering angle (60 degrees)
        self.max_wheel_speed = 2.0  # Maximum wheel speed (m/s)
        
        # Control parameters
        self.steering_mode = 0  # 0: In-phase, 1: Opposite-phase, 2: Pivot
        self.deadzone = 0.1  # Joystick deadzone
        
        # Current command values
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        self.get_logger().info('4-Wheel Independent Steering Controller initialized')
        self.get_logger().info('Steering modes: Button 0=In-phase, Button 1=Opposite-phase, Button 2=Pivot')

    def listenerCallback(self, msg: Joy):
        """
        Joystick callback function
        Expected joystick mapping:
        - Left stick X (axis 0): Linear velocity
        - Left stick Y (axis 1): Not used
        - Right stick X (axis 3): Angular velocity / Lateral movement
        - Button 0: In-phase mode
        - Button 1: Opposite-phase mode  
        - Button 2: Pivot mode
        """
        try:
            # Mode selection based on button presses
            if len(msg.buttons) >= 3:
                if msg.buttons[0] == 1:  # Button A/X
                    self.steering_mode = 0
                    self.get_logger().info('Switched to IN-PHASE mode')
                elif msg.buttons[1] == 1:  # Button B/Circle
                    self.steering_mode = 1
                    self.get_logger().info('Switched to OPPOSITE-PHASE mode')
                elif msg.buttons[2] == 1:  # Button X/Square
                    self.steering_mode = 2
                    self.get_logger().info('Switched to PIVOT mode')
            
            # Extract joystick values with deadzone
            if len(msg.axes) >= 4:
                linear_cmd = self.apply_deadzone(msg.axes[1])  # Forward/backward
                angular_cmd = self.apply_deadzone(msg.axes[0])  # Left/right or rotation
                
                # Calculate steering and velocity commands
                self.calculate_wheel_commands(linear_cmd, angular_cmd)
                
        except Exception as e:
            self.get_logger().error(f'Error in joystick callback: {str(e)}')

    def apply_deadzone(self, value):
        """Apply deadzone to joystick input"""
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def calculate_wheel_commands(self, linear_cmd, angular_cmd):
        """
        Calculate individual wheel steering angles and velocities based on the selected mode
        """
        # Initialize arrays for 4 wheels [FL, FR, RL, RR]
        steering_angles = [0.0, 0.0, 0.0, 0.0]
        wheel_velocities = [0.0, 0.0, 0.0, 0.0]
        
        if self.steering_mode == 0:
            # IN-PHASE MODE: All wheels steer in the same direction
            # Used for crab walking (lateral movement) and high-speed turns
            steering_angles, wheel_velocities = self.calculate_in_phase_mode(linear_cmd, angular_cmd)
            
        elif self.steering_mode == 1:
            # OPPOSITE-PHASE MODE: Front and rear wheels steer in opposite directions
            # Used for tight turns and low-speed maneuvering (Ackermann-like)
            steering_angles, wheel_velocities = self.calculate_opposite_phase_mode(linear_cmd, angular_cmd)
            
        elif self.steering_mode == 2:
            # PIVOT MODE: Vehicle rotates around its center
            # All wheels point towards/away from the vehicle center
            steering_angles, wheel_velocities = self.calculate_pivot_mode(angular_cmd)
        
        # Publish commands
        self.publish_commands(steering_angles, wheel_velocities)

    def calculate_in_phase_mode(self, linear_cmd, angular_cmd):
        """
        In-phase mode: All wheels steer in the same direction
        Enables crab-like lateral movement
        """
        steering_angles = [0.0, 0.0, 0.0, 0.0]
        wheel_velocities = [0.0, 0.0, 0.0, 0.0]
        
        if abs(angular_cmd) > 0.01:  # Lateral movement
            # All wheels point in the same direction for crab walking
            steer_angle = angular_cmd * self.max_steering_angle
            steering_angles = [steer_angle, steer_angle, steer_angle, steer_angle]
            
            # All wheels move at the same speed
            base_speed = linear_cmd * self.max_wheel_speed
            wheel_velocities = [base_speed, base_speed, base_speed, base_speed]
            
        else:  # Forward/backward movement
            # Straight line movement - all wheels straight
            steering_angles = [0.0, 0.0, 0.0, 0.0]
            base_speed = linear_cmd * self.max_wheel_speed
            wheel_velocities = [base_speed, base_speed, base_speed, base_speed]
        
        return steering_angles, wheel_velocities

    def calculate_opposite_phase_mode(self, linear_cmd, angular_cmd):
        """
        Opposite-phase mode: Front and rear wheels steer in opposite directions
        Implements Ackermann-like steering for tight turns
        """
        steering_angles = [0.0, 0.0, 0.0, 0.0]
        wheel_velocities = [0.0, 0.0, 0.0, 0.0]
        
        if abs(angular_cmd) > 0.01:  # Turning
            # Calculate turn radius based on angular command
            # Smaller angular_cmd = larger radius, larger angular_cmd = smaller radius
            if abs(angular_cmd) > 0.99:
                turn_radius = self.wheelbase / 2.0  # Minimum turn radius
            else:
                turn_radius = self.wheelbase / (2.0 * abs(angular_cmd))
            
            # Ackermann steering geometry
            # Front wheels
            if angular_cmd > 0:  # Left turn
                front_left_angle = math.atan(self.wheelbase / (turn_radius + self.track_width/2))
                front_right_angle = math.atan(self.wheelbase / (turn_radius - self.track_width/2))
                # Rear wheels steer opposite for tighter turning
                rear_left_angle = -math.atan(self.wheelbase / (turn_radius + self.track_width/2)) * 0.5
                rear_right_angle = -math.atan(self.wheelbase / (turn_radius - self.track_width/2)) * 0.5
            else:  # Right turn
                front_left_angle = -math.atan(self.wheelbase / (turn_radius - self.track_width/2))
                front_right_angle = -math.atan(self.wheelbase / (turn_radius + self.track_width/2))
                rear_left_angle = math.atan(self.wheelbase / (turn_radius - self.track_width/2)) * 0.5
                rear_right_angle = math.atan(self.wheelbase / (turn_radius + self.track_width/2)) * 0.5
            
            steering_angles = [front_left_angle, front_right_angle, rear_left_angle, rear_right_angle]
            
            # Calculate wheel velocities based on distance from turn center
            base_speed = linear_cmd * self.max_wheel_speed
            if abs(angular_cmd) > 0:
                # Different speeds for each wheel based on their distance from turn center
                wheel_velocities[0] = base_speed * (turn_radius + self.track_width/2) / turn_radius  # FL
                wheel_velocities[1] = base_speed * (turn_radius - self.track_width/2) / turn_radius  # FR
                wheel_velocities[2] = base_speed * (turn_radius + self.track_width/2) / turn_radius  # RL
                wheel_velocities[3] = base_speed * (turn_radius - self.track_width/2) / turn_radius  # RR
        else:
            # Straight movement
            steering_angles = [0.0, 0.0, 0.0, 0.0]
            base_speed = linear_cmd * self.max_wheel_speed
            wheel_velocities = [base_speed, base_speed, base_speed, base_speed]
        
        return steering_angles, wheel_velocities

    def calculate_pivot_mode(self, angular_cmd):
        """
        Pivot mode: Vehicle rotates around its center point
        All wheels point toward or away from the vehicle center
        """
        steering_angles = [0.0, 0.0, 0.0, 0.0]
        wheel_velocities = [0.0, 0.0, 0.0, 0.0]
        
        if abs(angular_cmd) > 0.01:
            # Calculate angles for each wheel to point toward center
            # Front left wheel
            fl_angle = math.atan2(self.track_width/2, self.wheelbase/2)
            # Front right wheel  
            fr_angle = -math.atan2(self.track_width/2, self.wheelbase/2)
            # Rear left wheel
            rl_angle = -math.atan2(self.track_width/2, self.wheelbase/2)
            # Rear right wheel
            rr_angle = math.atan2(self.track_width/2, self.wheelbase/2)
            
            steering_angles = [fl_angle, fr_angle, rl_angle, rr_angle]
            
            # Calculate velocities - all wheels rotate around center
            # Distance from center to each wheel
            radius_to_wheel = math.sqrt((self.wheelbase/2)**2 + (self.track_width/2)**2)
            angular_velocity = angular_cmd * 2.0  # Scale factor
            
            # All wheels have same speed magnitude but different directions
            wheel_speed = abs(angular_velocity) * radius_to_wheel
            wheel_speed = min(wheel_speed, self.max_wheel_speed)
            
            # Direction depends on rotation direction
            if angular_cmd > 0:  # Counter-clockwise
                wheel_velocities = [wheel_speed, wheel_speed, wheel_speed, wheel_speed]
            else:  # Clockwise
                wheel_velocities = [wheel_speed, wheel_speed, wheel_speed, wheel_speed]
        
        return steering_angles, wheel_velocities

    def publish_commands(self, steering_angles, wheel_velocities):
        """
        Publish steering angles and wheel velocities
        """
        # Limit steering angles
        steering_angles = [max(-self.max_steering_angle, min(self.max_steering_angle, angle)) 
                          for angle in steering_angles]
        
        # Limit wheel velocities
        wheel_velocities = [max(-self.max_wheel_speed, min(self.max_wheel_speed, vel)) 
                           for vel in wheel_velocities]
        
        # Create and publish steering command
        steering_msg = Float64MultiArray()
        steering_msg.data = steering_angles
        self.pub_pos.publish(steering_msg)
        
        # Create and publish velocity command
        velocity_msg = Float64MultiArray()
        velocity_msg.data = wheel_velocities
        self.pub_vel.publish(velocity_msg)
        
        # Debug logging
        mode_names = ["IN-PHASE", "OPPOSITE-PHASE", "PIVOT"]
        self.get_logger().debug(f'Mode: {mode_names[self.steering_mode]}')
        self.get_logger().debug(f'Steering: FL={steering_angles[0]:.3f}, FR={steering_angles[1]:.3f}, '
                               f'RL={steering_angles[2]:.3f}, RR={steering_angles[3]:.3f}')
        self.get_logger().debug(f'Velocities: FL={wheel_velocities[0]:.3f}, FR={wheel_velocities[1]:.3f}, '
                               f'RL={wheel_velocities[2]:.3f}, RR={wheel_velocities[3]:.3f}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = FourWheelSteeringController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        if 'controller' in locals():
            controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()