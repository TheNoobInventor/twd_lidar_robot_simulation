#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import math

class Joystick(Node):
    
    #
    def __init__(self, name):

        super().__init__(name)
        self.get_logger().info(self.get_name() + " is initialized")

        self.speed = 0.0 #
        self.spin = 0.0 # 

        self.twist = Twist()

        self.__joy_subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            5
        )
        self.__cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    #
    def joy_callback(self, msg):
        '''
        This translates buttons on a generic game controller into speed and spin

        Using:

        Move right joystick left/right for corresponding left/right motion
        Move left joystick forward/backward for corresponding forward/backward motion 
        R2 for emergency stop

        Rstick left/right         axes[2]    +1 (left)    to -1 (right)
        Lstick forward/backward   axes[1]    +1 (forward) to -1 (backward)
        R2                        buttons[7]  1 pressed, 0 otherwise
        '''

        if abs(msg.axes[2]) > 0.10:
            self.spin = math.pi*msg.axes[2]
        else:
            self.spin = 0.0

        if abs(msg.axes[1]) > 0.10:
            self.speed = msg.axes[1]
        else:
            self.speed = 0.0

        if msg.buttons[7] == 1:
            self.speed = 0.0
            self.spin = 0.0

        # publish cmd vel values
        self.command_vel_pub()    

    def command_vel_pub(self):

        self.twist.linear.x = self.speed
        self.twist.angular.z = self.spin
        
        self.__cmd_vel_publisher.publish(self.twist)

def main(args=None):

    rclpy.init(args=args)

    #
    joystick = Joystick('joystick_node') 

    rclpy.spin(joystick)

    joystick.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# Comment cleanly

# change name of class, node and file to remove confusion