import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from franka_msgs.action import Grasp
from rclpy.task import Future
from franka_msgs.msg import GraspEpsilon

import sys
import ctypes
import sdl2
import sdl2.ext

class GraspActionClient(Node):
    def __init__(self):
        super().__init__('grasp_action_client')
        self._action_client = ActionClient(self, Grasp, '/fr3_gripper/grasp')
        self.complete = False

    def send_goal(self, width, epsilon, speed, force):
        """Send a grasping goal to the action server."""
        goal_msg = Grasp.Goal()
        goal_msg.width = width
        goal_msg.epsilon = epsilon
        goal_msg.speed = speed
        goal_msg.force = force

        self.get_logger().info(f"Sending goal: width={width}, speed={speed}, force={force}")
        print("waiting for server")
        self._action_client.wait_for_server()
        print("got server")
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        #send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback = self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)
        return

    def goal_response_callback(self, future):
        """Handle goal response (accepted or rejected)."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected by the server.")
            return

        self.get_logger().info("Goal accepted. Waiting for result...")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
        return

    def feedback_callback(self, feedback_msg):
        """Handle feedback from the action server."""
        self.get_logger().info(f"Feedback: current_width={feedback_msg.current_width}")
        return

    def get_result_callback(self, future):
        """Handle the result of the action."""
        result = future.result().result
        if result.success:
            self.get_logger().info(f"Grasp successful!")
            self.complete = True
        else:
            self.get_logger().error(f"Grasp failed: {result.error}")
            self.complete = True
        print("yeeda yada")

def main(args=None):
    # Initialize SDL2 with video and game controller support.
    if sdl2.SDL_Init(sdl2.SDL_INIT_VIDEO | sdl2.SDL_INIT_GAMECONTROLLER) != 0:
        print("SDL_Init Error:", sdl2.SDL_GetError().decode())
        return

    # Open the first available game controller.
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

    event = sdl2.SDL_Event()
    running = True
    print("Listening for controller events. Press the close button on the window to exit.")

    rclpy.init(args=args)
    action_client = GraspActionClient()
    
    # Example goal values
    width = 0.08  # 8 cm
    epsilon = GraspEpsilon(inner = 0.1, outer = 0.1)  # Define the correct epsilon structure
    speed = 0.05  # 5 cm/s
    force = 20.0  # 20 N
    close = False
    open = False

    while running:
        # Poll events from the SDL event queue.
        while sdl2.SDL_PollEvent(ctypes.byref(event)):
            if event.type == sdl2.SDL_QUIT:
                running = False
            elif event.type == sdl2.SDL_CONTROLLERAXISMOTION:
                axis = event.caxis.axis
                value = event.caxis.value
            elif event.type == sdl2.SDL_CONTROLLERBUTTONDOWN:
                button = event.cbutton.button
                if(button == 9):
                    width = 0.0
                    speed = 0.1
                    action_client.send_goal(width, epsilon, speed, force)
                    close = True
                    action_client.complete = False
                elif(button == 10):
                    width = 0.075
                    speed = 0.1
                    action_client.send_goal(width, epsilon, speed, force)
                    open = True
                    action_client.complete = False
                elif(button == 6):
                    running = False
                print(f"Button {button} pressed")
            elif event.type == sdl2.SDL_CONTROLLERBUTTONUP:
                button = event.cbutton.button
                print(f"Button {button} released")
            
            while(close or open):
                rclpy.spin_once(action_client)
                if action_client.complete:
                    close = False
                    open = False
                

    print("bye bye")
    # Clean up before exiting.
    sdl2.SDL_GameControllerClose(controller)
    sdl2.SDL_Quit()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
