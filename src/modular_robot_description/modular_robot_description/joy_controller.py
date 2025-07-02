import math
import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

vel_msg = Twist()  # robot velosity
mode_selection = 0 # 1:opposite phase(ackerman mode), 2:in-phase(crap walking) , 3:pivot turn 4: none
extending_length = 0.0 # extending length in meters, used for periodic extension
elevating_length = 0.0 # elevating length in meters, used for periodic elevation
class Commander(Node):

    def __init__(self):
        global extending_length
        super().__init__('commander')
        timer_period = 0.02
        self.WHEEL_SEPERATION = 1.082 
        self.WHEEL_BASE = 0.900
        self.WHEEL_RADIUS = 0.1
        self.wheel_steering_y_offset = 0.0
        self.steering_track = self.WHEEL_SEPERATION - 2 * self.wheel_steering_y_offset
        self.new_steering_track = self.steering_track
        self.turn_radius = 0.0 
        self.MAX_STEERING_ANGLE = math.pi/4

        self.pos = np.array([0,0,0,0], float) #[rear_right_steering, rear_left_steering, front_right_steering, front_left_steering] in radians
        self.vel = np.array([0,0,0,0], float) #[rear_right_velocity, rear_left_velocity, front_right_velocity, front_left_velocity] in radians/sec
        self.pub_pos = self.create_publisher(Float64MultiArray, '/steering_controller', 1)
        self.pub_vel = self.create_publisher(Float64MultiArray, '/wheel_motors_controller', 1)
        self.extending = self.create_publisher(Float64, "extending_in_meter_controller", 1)
        self.elevating = self.create_publisher(Float64, "/elevating_in_meter_controller", 1)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        global vel_msg, mode_selection, extending_length, elevating_length
        self.new_steering_track = self.steering_track + extending_length
        # opposite phase
        if(mode_selection == 1):
            
            vel_steerring_offset = vel_msg.angular.z * self.wheel_steering_y_offset
            sign = np.sign(vel_msg.linear.x)

            if abs(vel_msg.angular.z) > 0.01:
                self.turn_radius = self.WHEEL_BASE / (2.0*abs(vel_msg.angular.z))

            self.vel[0] = sign*math.hypot(vel_msg.linear.x - vel_msg.angular.z*self.new_steering_track/2, vel_msg.angular.z*self.WHEEL_BASE/2) - vel_steerring_offset
            self.vel[1] = sign*math.hypot(vel_msg.linear.x + vel_msg.angular.z*self.new_steering_track/2, vel_msg.angular.z*self.WHEEL_BASE/2) + vel_steerring_offset
            self.vel[2] = sign*math.hypot(vel_msg.linear.x - vel_msg.angular.z*self.new_steering_track/2, vel_msg.angular.z*self.WHEEL_BASE/2) - vel_steerring_offset
            self.vel[3] = sign*math.hypot(vel_msg.linear.x + vel_msg.angular.z*self.new_steering_track/2, vel_msg.angular.z*self.WHEEL_BASE/2) + vel_steerring_offset
            if(vel_msg.angular.z != 0):
                if (vel_msg.angular.z<0):
                    self.pos[0] = np.clip(math.atan2(self.WHEEL_BASE,(self.turn_radius+self.new_steering_track/2)),-self.MAX_STEERING_ANGLE,self.MAX_STEERING_ANGLE)
                    self.pos[1] = np.clip(math.atan2(self.WHEEL_BASE,(self.turn_radius-self.new_steering_track/2)),-self.MAX_STEERING_ANGLE,self.MAX_STEERING_ANGLE)
                    self.pos[2] =  -self.pos[0] 
                    self.pos[3] =  -self.pos[1]
                else:
                    self.pos[0] = -np.clip(math.atan2(self.WHEEL_BASE,(self.turn_radius+self.new_steering_track/2)),-self.MAX_STEERING_ANGLE,self.MAX_STEERING_ANGLE)
                    self.pos[1] = -np.clip(math.atan2(self.WHEEL_BASE,(self.turn_radius-self.new_steering_track/2)),-self.MAX_STEERING_ANGLE,self.MAX_STEERING_ANGLE)
                    self.pos[2] = -self.pos[0]
                    self.pos[3] = -self.pos[1]       
            else:
                self.pos[0] = 0
                self.pos[1] = 0
                self.pos[2] = 0
                self.pos[3] = 0

        # in-phase
        elif(mode_selection == 2):

            V = math.hypot(vel_msg.linear.x, vel_msg.linear.y)
            sign = np.sign(vel_msg.linear.x)
            
            if(vel_msg.linear.x != 0):
                ang = - vel_msg.linear.y / vel_msg.linear.x
            else:
                ang = 0
            
            self.pos[0] = math.atan(ang)
            self.pos[1] = math.atan(ang)
            self.pos[2] = self.pos[0]
            self.pos[3] = self.pos[1]
            
            self.vel[:] = sign*V
            
        # pivot turn
        elif(mode_selection == 3):

            self.pos[0] = math.atan2(self.WHEEL_BASE,self.new_steering_track)
            self.pos[1] = -math.atan2(self.WHEEL_BASE,self.new_steering_track)
            self.pos[2] = -math.atan2(self.WHEEL_BASE,self.new_steering_track)
            self.pos[3] = math.atan2(self.WHEEL_BASE,self.new_steering_track)
            
            self.vel[0] = -vel_msg.angular.z
            self.vel[1] = vel_msg.angular.z
            self.vel[2] = self.vel[0]
            self.vel[3] = self.vel[1]

        else:

            self.pos[:] = 0
            self.vel[:] = 0

        pos_array = Float64MultiArray(data=self.pos) 
        vel_array = Float64MultiArray(data=self.vel) 
        self.pub_pos.publish(pos_array)
        self.pub_vel.publish(vel_array)
        self.extending.publish(Float64(data=extending_length))
        self.elevating.publish(Float64(data=elevating_length))
        self.pos[:] = 0
        self.vel[:] = 0

class Joy_subscriber(Node):

    def __init__(self):
        super().__init__('joy_subscriber')
        self.subscription = self.create_subscription(Joy,'/joy',self.listenerCallback,1)
        self.create_timer(0.1,self.ControlApply)
        self.joy_controller = Joy()

    def listenerCallback(self, data: Joy):
        self.joy_controller = data

    def ControlApply(self):
        global vel_msg, mode_selection, extending_length, elevating_length

        if(self.joy_controller.buttons[0] == 1):   # in-phase # A button of Xbox 360 controller
            mode_selection = 1
        elif(self.joy_controller.buttons[1] == 1): # opposite phase # LB button of Xbox 360 controller
            mode_selection = 2
        elif(self.joy_controller.buttons[2] == 1): # pivot turn # RB button of Xbox 360 controller
            mode_selection = 3
        else:
            mode_selection = mode_selection

        # Increment the extending and elevating lengths periodically
        if (self.joy_controller.buttons[4] == 1 and extending_length <= 0.5):
            extending_length +=0.01
        elif (self.joy_controller.buttons[6] == 1 and extending_length >= 0.0):
            extending_length -= 0.01
        
        if (self.joy_controller.buttons[5] == 1 and elevating_length <= 0.4):
            elevating_length +=0.01
        elif (self.joy_controller.buttons[7]==1 and elevating_length > 0.0):
            elevating_length -=0.01
        pass

        vel_msg.linear.x  = self.joy_controller.axes[1]*1.5
        vel_msg.linear.y  = self.joy_controller.axes[0]*1.5
        vel_msg.angular.z = self.joy_controller.axes[3]*3

        

if __name__ == '__main__':
    rclpy.init(args=None)
    
    commander = Commander()
    joy_subscriber = Joy_subscriber()

    executor = MultiThreadedExecutor()
    executor.add_node(commander)
    executor.add_node(joy_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    rate = commander.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()
    executor_thread.join()
