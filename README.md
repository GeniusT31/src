# Singularity and Oscillation Avoidance: Bachelor's Thesis Anthony Li
Entire src folder needed due to code changes in packages other than cartesian_impedance_control

## Installation
# Prerequisites:
* ROS2 humble <br />
* Libfranka 0.13.0 or newer <br />
* franka_ros2 v0.13.1 <br />


For further information, please refer to the [Franka ROS2 FCI documentation](https://support.franka.de/docs/franka_ros2.html)

Once you have everything set up, follow the steps below to get the controller running.

# If you do not yet have a franka_ros2_ws or it is empty:
Entire src folder needed due to code changes in packages other than cartesian_impedance_control  
Clone this repository in the src directory of your franka_ros2_ws : 

```bash
mkdir franka_ros2_ws/
cd franka_ros2_ws/
git clone https://github.com/GeniusT31/src.git
```

# If there are other files in your franka_ros2_ws/src directory
Steps to be done:
Add XBoxController.msg and add the new msg type to CMakeLists.txt
Replace franka_robot_model.hpp with my implementation: Found under franka_ros2_ws/src/franka_semantic_components/include/franka_semantic_components/


# Running the controller
Build the package or whole workspace: <br />
```bash
colcon build --packages-select cartesian_impedance_control --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release #Builds all the packages in your src folder
```

If not yet done, ensure your setup is always source by adding the following line to the end of your .bashrc file (to get access to it, you need to execute `nano .bashrc` in your home directory). : <br />
```bash
source /home/<user>/franka_ros2_ws/install/setup.sh 
```

Install the necessary library to read xBox controller messages: <br />
```bash
pip install PySDL2
```

# Launching the controller
Launch the controller: <br />
```bash
ros2 launch cartesian_impedance_control cartesian_impedance_controller.launch.py
```

Open a new terminal to launch the X-Box controller publisher<br />
```bash
python3 /home/<user>/franka_ros2_ws/src/cartesian_impedance_control/cart_vel_publisher.py
```

Open another new terminal to launch the gripper functionality, not needed if not using gripper<br />
```bash
python3 /home/<user>/franka_ros2_ws/src/cartesian_impedance_control/Action_client.py
```

