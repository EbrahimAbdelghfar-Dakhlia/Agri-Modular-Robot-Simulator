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
        #define the subscribers for the joint states
        self.create_subscription(Float64, 'extending_in_meter_controller',self.extendingCallback, 1)
        self.create_subscription(Float64, 'elevating_in_meter_controller',self.elevatingCallback, 1)
        self.create_timer(0.001, self.timerCallback)
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