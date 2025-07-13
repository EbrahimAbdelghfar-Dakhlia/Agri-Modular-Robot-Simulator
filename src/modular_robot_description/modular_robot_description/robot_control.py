import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray
class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        # intialize the variable to store the joint states
        self.recieved_extending_length_meter = 0.0
        self.recieved_elevating_length_meter = 0.0
        self.recieved_rr_steering_angle_radian = 0.0 
        self.recieved_rl_steering_angle_radian = 0.0
        self.recieved_fr_steering_angle_radian = 0.0
        self.recieved_fl_steering_angle_radian = 0.0
        self.motor_velocity_rad_sec = [0.0,0.0,0.0,0.0]
        # Define QoS profile for the publisher
        self.__qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        #define the subscribers for the joint states
        self.create_subscription(Float64, 'extending_in_meter_controller',self.extendingCallback, 1)
        self.create_subscription(Float64, 'elevating_in_meter_controller',self.elevatingCallback, 1)
        self.create_subscription(Float64MultiArray, 'wheel_motors_controller', self.setMotorVelocity, 1)
        # Create a subscription to the joint states topic
        self.create_subscription(Float64MultiArray, 'steering_controller', self.jointStatesCallback,1)
        self.create_timer(0.01, self.timerCallback)
        # Create a publisher for extending joints in meters
        self.__rr_extending_publisher = self.create_publisher(Float64, 'rr_extending_pos_arm_joint',1)
        self.__rl_extending_publisher = self.create_publisher(Float64, 'rl_extending_pos_arm_joint',1)
        self.__fr_extending_publisher = self.create_publisher(Float64, 'fr_extending_pos_arm_joint',1)
        self.__fl_extending_publisher = self.create_publisher(Float64, 'fl_extending_pos_arm_joint',1)
        # Create a publisher for elevating position arm joints in meters
        self.__rr_elevating_publisher = self.create_publisher(Float64, 'rr_elevating_pos_arm_joint',1)
        self.__rl_elevating_publisher = self.create_publisher(Float64, 'rl_elevating_pos_arm_joint',1)
        self.__fr_elevating_publisher = self.create_publisher(Float64, 'fr_elevating_pos_arm_joint',1)
        self.__fl_elevating_publisher = self.create_publisher(Float64, 'fl_elevating_pos_arm_joint',1)
        # Create a publisher for steering position arm joints in radians
        self.__rr_steering_publisher = self.create_publisher(Float64, '/fwids/rear_right_steer_rad/command',1)
        self.__rl_steering_publisher = self.create_publisher(Float64, '/fwids/rear_left_steer_rad/command',1)
        self.__fr_steering_publisher = self.create_publisher(Float64, '/fwids/front_right_steer_rad/command',1)
        self.__fl_steering_publisher = self.create_publisher(Float64, '/fwids/front_left_steer_rad/command',1)
        # create a publisher for the motor velocity in rad/sec
        self.__rr_motor_velocity_publisher = self.create_publisher(Float64, '/fwids/rear_right_rotor_radpersec/command',1)
        self.__rl_motor_velocity_publisher = self.create_publisher(Float64, '/fwids/rear_left_rotor_radpersec/command',1)
        self.__fr_motor_velocity_publisher = self.create_publisher(Float64, '/fwids/front_right_rotor_radpersec/command',1)
        self.__fl_motor_velocity_publisher = self.create_publisher(Float64, '/fwids/front_left_rotor_radpersec/command',1)
        # Create a timer to publish

    def setMotorVelocity(self, velocity:Float64MultiArray):
        # Set the motor velocity in rad/sec
        wheel_radius = 0.1  # in meters
        self.motor_velocity_rad_sec = [
            velocity.data[0]/wheel_radius, #rear right motor
            velocity.data[1]/wheel_radius, #rear left motor
            velocity.data[2]/wheel_radius, #front right motor
            velocity.data[3]/wheel_radius  #front left motor
        ]

    def jointStatesCallback(self, msg:Float64MultiArray):
        # Update the steering angles based on the received message
        self.recieved_rr_steering_angle_radian = msg.data[0]
        self.recieved_rl_steering_angle_radian = msg.data[1]
        self.recieved_fr_steering_angle_radian = msg.data[2]
        self.recieved_fl_steering_angle_radian = msg.data[3]
    def extendingCallback(self, msg:Float64):
        # Update the extending length based on the received message
        self.recieved_extending_length_meter = msg.data

    def elevatingCallback(self, msg:Float64):
        # Update the elevating length based on the received message
        self.recieved_elevating_length_meter = msg.data

    def timerCallback(self):
        # Publish the command to each joint
        self.__rr_extending_publisher.publish(Float64(data=self.recieved_extending_length_meter))
        self.__rl_extending_publisher.publish(Float64(data=-1*self.recieved_extending_length_meter))
        self.__fr_extending_publisher.publish(Float64(data=self.recieved_extending_length_meter))
        self.__fl_extending_publisher.publish(Float64(data=-1*self.recieved_extending_length_meter))
        
        self.__rr_elevating_publisher.publish(Float64(data=-1*self.recieved_elevating_length_meter))
        self.__rl_elevating_publisher.publish(Float64(data=-1*self.recieved_elevating_length_meter))
        self.__fr_elevating_publisher.publish(Float64(data=-1*self.recieved_elevating_length_meter))
        self.__fl_elevating_publisher.publish(Float64(data=-1*self.recieved_elevating_length_meter))
        
        self.__rr_steering_publisher.publish(Float64(data=self.recieved_rr_steering_angle_radian))
        self.__rl_steering_publisher.publish(Float64(data=self.recieved_rl_steering_angle_radian))
        self.__fr_steering_publisher.publish(Float64(data=self.recieved_fr_steering_angle_radian))
        self.__fl_steering_publisher.publish(Float64(data=self.recieved_fl_steering_angle_radian))

        # Publish the motor velocity
        self.__rr_motor_velocity_publisher.publish(Float64(data=self.motor_velocity_rad_sec[0]))
        self.__rl_motor_velocity_publisher.publish(Float64(data=self.motor_velocity_rad_sec[1]))
        self.__fr_motor_velocity_publisher.publish(Float64(data=self.motor_velocity_rad_sec[2]))
        self.__fl_motor_velocity_publisher.publish(Float64(data=self.motor_velocity_rad_sec[3]))

def main(args=None):
    rclpy.init(args=args)
    robot_control_node = RobotControlNode()
    try:
        rclpy.spin(robot_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        robot_control_node.destroy_node()
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()