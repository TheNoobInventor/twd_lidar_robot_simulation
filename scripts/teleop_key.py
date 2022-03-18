#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput.keyboard import Key, Listener, KeyCode
# import getpass

#
class TeleopPublisher(Node):
    
    def __init__(self, name):
        #
        super().__init__(name)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1 # seconds
        #
        self.get_logger().info(self.get_name() + " is initialized")
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.msg = Twist()
        # 2D linear and angular movement
        self.msg.linear.x = 0.0
        self.msg.angular.z = 0.0

    #
    def timer_callback(self):
        # Collect events until released(?)

        with Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            self.get_logger().info("""
                Move the 2-wheeled drive car around:
                ------------------------------------
                        w
                    a       d
                        s
                
                Space key: Force stop

                Ctrl-C: to quit            
            """
            )

            listener.join()

    #
    def on_press(self, key):
        if key == KeyCode(char="w"):
            self.msg.linear.x = 0.5 
        elif key == KeyCode(char="s"):
            self.msg.linear.x = -0.5
        elif key == KeyCode(char="a"):
            self.msg.angular.z = 3.0
        elif key == KeyCode(char="d"):
            self.msg.angular.z = -3.0
        elif key == Key.space:
            self.msg.linear.x = 0.0
            self.msg.angular.z = 0.0

        self.publisher.publish(self.msg)
    #
    def on_release(self, key):
        self.msg.linear.x = 0.0
        self.msg.angular.z = 0.0

        self.publisher.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)

    teleop_publisher = TeleopPublisher('teleop_keyboard')

    rclpy.spin(teleop_publisher)

    #
    teleop_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# To-do:

# Add comments...cleanly

# Tune linear and angular better?