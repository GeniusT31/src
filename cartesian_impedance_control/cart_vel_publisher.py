import sys
import ctypes
import sdl2
import sdl2.ext
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
from rclpy.action import ActionServer
from std_srvs.srv import SetBool
from control_msgs.action import GripperCommand

class Vel_Publisher(Node):

    def __init__(self):
        super().__init__("Cartesian_Velocity_Publisher")

        if sdl2.SDL_Init(sdl2.SDL_INIT_VIDEO | sdl2.SDL_INIT_GAMECONTROLLER) != 0:
            print("SDL_Init Error:", sdl2.SDL_GetError().decode())
            return
        controller = None
        num_joysticks = sdl2.SDL_NumJoysticks()
        for i in range(num_joysticks):
            if sdl2.SDL_IsGameController(i):
                controller = sdl2.SDL_GameControllerOpen(i)
                if controller:
                    name = sdl2.SDL_GameControllerName(controller)
                    print("Connected controller:", name.decode() if name else "Unknown")
                    break
        if not controller:
            print("No game controller found!")
            sdl2.SDL_Quit()
            sys.exit(1)
        
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.ang_z = 0.0
        self.cmd_vel_pub = self.create_publisher(Twist, 'desired_EE_vel', 10)
        self.time = self.create_timer(0.1, self.send_velocity_command)

        self.get_logger().info("Cartesian end effector velocity publisher has been started")
    
    def send_velocity_command(self):
        prev = time.time()
        event = sdl2.SDL_Event()
        msg = Twist()
        #TODO define the velocities in x, y and z direction
        msg.linear.x = self.x
        msg.linear.y = self.y
        msg.linear.z = self.z
        msg.angular.z = self.ang_z

        while((time.time() - prev < 0.0999)):
            somehow_needed = sdl2.SDL_PollEvent(ctypes.byref(event))
            if event.type == sdl2.SDL_CONTROLLERAXISMOTION:
                axis = event.caxis.axis
                value = event.caxis.value
                print("joystick")
                if(axis == 0):
                    msg.linear.y = round(value / 32767, 2)
                elif(axis == 1):
                    msg.linear.x = round(value / 32768, 2)
            elif event.type == sdl2.SDL_CONTROLLERBUTTONDOWN:
                button = event.cbutton.button
                print("button")
                if(button == 0):
                    msg.linear.z = -1.0
                elif(button == 1):
                    msg.linear.z = 1.0
                elif(button == 13):
                    msg.angular.z = -1.0
                    print("hee")
                elif(button == 14):
                    msg.angular.z = 1.0
                elif(button == 6):
                    sdl2.SDL_Quit()
                    sys.exit(1)
            elif event.type == sdl2.SDL_CONTROLLERBUTTONUP:
                button = event.cbutton.button
                if(button == 0 or button == 1):
                    msg.linear.z = 0.0
                elif(button == 13 or button == 14):
                    print("haa")
                    msg.angular.z = 0.0

        self.x = msg.linear.x
        self.y = msg.linear.y
        self.z = msg.linear.z
        self.ang_z = msg.angular.z

        self.cmd_vel_pub.publish(msg)

    async def execute_callback():
        print("Hi")
    
def main(args = None):
    rclpy.init()
    node = Vel_Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


