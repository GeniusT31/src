import sys
import ctypes
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pandas as pd

class Pos_Publisher(Node):

    def __init__(self):
        super().__init__("Cartesian_Position_Publisher")

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.total_rows = 200

        #0 indexing, using 1 to skip header
        self.current_row = 1
        self.df = pd.read_csv("Random_positions.csv")
        self.msg = Twist()

        self.msg.angular.x = 0
        self.msg.angular.y = 0
        self.msg.angular.z = 0

        self.cmd_vel_pub = self.create_publisher(Twist, 'desired_EE_pos', 10)
        self.time = self.create_timer(5, self.send_velocity_command)

        self.get_logger().info("Cartesian end effector velocity publisher has been started")
    
    def send_velocity_command(self):
        if(self. current_row == self.total_rows):
            self.get_logger().info("Experiment complete, shutting down...")
            self.file.close()
            rclpy.shutdown()

        #TODO define the position in x, y and z direction
        self.msg.linear.x = self.x
        self.msg.linear.y = self.y
        self.msg.linear.z = self.z

        self.msg.linear.x = self.df.iloc[self.current_row][0]
        self.msg.linear.y = self.df.iloc[self.current_row][1]
        self.msg.linear.z = self.df.iloc[self.current_row][2]
        self.current_row = self.current_row + 1

        self.x = self.msg.linear.x
        self.y = self.msg.linear.y
        self.z = self.msg.linear.z

        self.cmd_vel_pub.publish(self.msg)

    async def execute_callback():
        print("Hi")
    
def main(args = None):
    rclpy.init()
    node = Pos_Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


