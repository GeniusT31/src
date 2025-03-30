import sys
import ctypes
import sdl2
import sdl2.ext
import rclpy
from rclpy.node import Node
from franka_msgs.msg import XBoxController
import time

class Vel_Publisher(Node):

    def __init__(self):
        super().__init__("X_Box_Controller_Publisher")

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
        
        self.buttonNo = 14
        self.axisNo = 6
        
        self.button_a0 = False
        self.button_b1 = False
        self.button_x2 = False
        self.button_y3 = False
        self.button_4 = False
        self.button_menu_6 = False
        self.button_l3_7 = False
        self.button_r3_8 = False
        self.button_l1_9 = False
        self.button_r1_10 = False
        self.button_up_11 = False
        self.button_down_12 = False
        self.button_left_13 = False
        self.button_right_14 = False

        self.joystick_l_vert = 0.0
        self.joystick_l_hori = 0.0
        self.joystick_r_vert = 0.0
        self.joystick_r_hori = 0.0
        self.l2_axis4 = 0.0
        self.r2_axis5 = 0.0
            
        self.cmd_vel_pub = self.create_publisher(XBoxController, 'XBoxButtons', 10)
        self.time = self.create_timer(0.1, self.send_velocity_command)

        self.get_logger().info("X Box buttons are being published")

    def send_velocity_command(self):
        prev = time.time()
        event = sdl2.SDL_Event()

        msg = XBoxController()

        msg.button_a0 = self.button_a0
        msg.button_b1 = self.button_b1
        msg.button_x2 = self.button_x2
        msg.button_y3 = self.button_y3
        msg.button_4 = self.button_4
        #button 5 does not show up nor work
        msg.button_menu_6 = self.button_menu_6
        msg.button_l3_7 = self.button_l3_7
        msg.button_r3_8 = self.button_r3_8
        msg.button_l1_9 = self.button_l1_9
        msg.button_r1_10 = self.button_r1_10
        msg.button_up_11 = self.button_up_11
        msg.button_down_12 = self.button_down_12
        msg.button_left_13 = self.button_left_13
        msg.button_right_14 = self.button_right_14

        msg.joystick_l_vert = self.joystick_l_vert
        msg.joystick_l_hori = self.joystick_l_hori
        msg.joystick_r_vert = self.joystick_r_vert
        msg.joystick_r_hori = self.joystick_r_hori
        msg.l2_axis4 = self.l2_axis4
        msg.r2_axis5 = self.r2_axis5



        while((time.time() - prev < 0.0999)):
            somehow_needed = sdl2.SDL_PollEvent(ctypes.byref(event))
            if event.type == sdl2.SDL_CONTROLLERAXISMOTION:
                axis = event.caxis.axis
                value = event.caxis.value
                if(axis == 0):
                    msg.joystick_l_hori = round(value / 32767, 2)
                elif(axis == 1):
                    msg.joystick_l_vert = -round(value / 32768, 2)
                elif(axis == 2):
                    msg.joystick_r_hori = round(value / 32767, 2)
                elif(axis == 3):
                    msg.joystick_r_vert = -round(value / 32768, 2)
                elif(axis == 4):
                    msg.l2_axis4 = round(value / 32768, 2)
                elif(axis == 5):
                    msg.r2_axis5 = round(value / 32768, 2)
            elif event.type == sdl2.SDL_CONTROLLERBUTTONDOWN:
                button = event.cbutton.button
                if(button == 0):
                    msg.button_a0 = True
                elif(button == 1):
                    msg.button_b1 = True
                elif(button == 2):
                    msg.button_x2 = True
                elif(button == 3):
                    msg.button_y3 = True
                elif(button == 4):
                    msg.button_4 = True
                elif(button == 5):
                    pass
                elif(button == 6):
                    msg.button_menu_6 = True
                elif(button == 7):
                    msg.button_l3_7 = True
                elif(button == 8):
                    msg.button_r3_8 = True
                elif(button == 9):
                    msg.button_l1_9 = True
                elif(button == 10):
                    msg.button_r1_10 = True
                elif(button == 11):
                    msg.button_up_11 = True
                elif(button == 12):
                    msg.button_down_12 = True
                elif(button == 13):
                    msg.button_left_13 = True
                elif(button == 14):
                    msg.button_right_14 = True
                print(f"Button {button} pressed")

                #if quitting desired
                '''    sdl2.SDL_Quit()
                    sys.exit(1)'''
                
            elif event.type == sdl2.SDL_CONTROLLERBUTTONUP:
                button = event.cbutton.button
                if(button == 0):
                    msg.button_a0 = False
                elif(button == 1):
                    msg.button_b1 = False
                elif(button == 2):
                    msg.button_x2 = False
                elif(button == 3):
                    msg.button_y3 = False
                elif(button == 4):
                    msg.button_4 = False
                elif(button == 5):
                    pass
                elif(button == 6):
                    msg.button_menu_6 = False
                elif(button == 7):
                    msg.button_l3_7 = False
                elif(button == 8):
                    msg.button_r3_8 = False
                elif(button == 9):
                    msg.button_l1_9 = False
                elif(button == 10):
                    msg.button_r1_10 = False
                elif(button == 11):
                    msg.button_up_11 = False
                elif(button == 12):
                    msg.button_down_12 = False
                elif(button == 13):
                    msg.button_left_13 = False
                elif(button == 14):
                    msg.button_right_14 = False
                print(f"Button {button} released")

        self.button_a0 = msg.button_a0
        self.button_b1 = msg.button_b1
        self.button_x2 = msg.button_x2
        self.button_y3 = msg.button_y3
        self.button_4 = msg.button_4
        #button 5 does not show up nor work
        msg.button_menu_6 = self.button_menu_6 = msg.button_menu_6
        self.button_l3_7 =msg.button_l3_7
        self.button_r3_8 = msg.button_r3_8
        self.button_l1_9 = msg.button_l1_9
        self.button_r1_10 = msg.button_r1_10
        self.button_up_11 = msg.button_up_11
        self.button_down_12 = msg.button_down_12
        self.button_left_13 = msg.button_left_13
        self.button_right_14 = msg.button_right_14

        self.joystick_l_vert = msg.joystick_l_vert
        self.joystick_l_hori = msg.joystick_l_hori
        self.joystick_r_vert = msg.joystick_r_vert
        self.joystick_r_hori = msg.joystick_r_hori
        self.l2_axis4 = msg.l2_axis4
        self.r2_axis5 = msg.r2_axis5

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


