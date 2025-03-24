cmake_minimum_required(VERSION 3.8)
project(messages_fr3)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetPose.srv"
  "srv/SetParam.srv"
  "srv/SetMode.srv"
  "srv/PlannerService.srv"
  "srv/SetStiffness.srv"
  "srv/ControllerActivation.srv"
  "msg/JacobianEE.msg"
  "msg/JointEEState.msg"
  "msg/PoseDirection.msg"
  "msg/JointConfig.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
