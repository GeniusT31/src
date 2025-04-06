// Copyright (c) 2021 Franka Emika GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <array>
#include <cassert>
#include <cmath>
#include <cstring>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <unistd.h>
#include <thread>
#include <chrono>         

#include "cartesian_impedance_control/user_input_server.hpp"

#include <rclcpp/rclcpp.hpp>
#include "franka_msgs/msg/x_box_controller.hpp"
#include "rclcpp/subscription.hpp"

#include <Eigen/Dense>
#include <Eigen/Eigen>

#include <controller_interface/controller_interface.hpp>

#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include "franka_hardware/franka_hardware_interface.hpp"
#include <franka_hardware/model.hpp>

#include "franka_msgs/msg/franka_robot_state.hpp"
#include "franka_msgs/msg/errors.hpp"
#include "messages_fr3/srv/set_pose.hpp"

#include "franka_semantic_components/franka_robot_model.hpp"
#include "franka_semantic_components/franka_robot_state.hpp"

#include <fstream>

#define IDENTITY Eigen::MatrixXd::Identity(6, 6)

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using Vector7d = Eigen::Matrix<double, 7, 1>;

namespace cartesian_impedance_control {

class CartesianImpedanceController : public controller_interface::ControllerInterface {
public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;

  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

    void setPose(const std::shared_ptr<messages_fr3::srv::SetPose::Request> request, 
    std::shared_ptr<messages_fr3::srv::SetPose::Response> response);
      

 private:
    //Nodes
    rclcpp::Subscription<franka_msgs::msg::FrankaRobotState>::SharedPtr franka_state_subscriber = nullptr;
    rclcpp::Service<messages_fr3::srv::SetPose>::SharedPtr pose_srv_;

    //Functions
    void topic_callback(const std::shared_ptr<franka_msgs::msg::FrankaRobotState> msg);
    void updateJointStates();
    void update_stiffness_and_references();
    void arrayToMatrix(const std::array<double, 6>& inputArray, Eigen::Matrix<double, 6, 1>& resultMatrix);
    void arrayToMatrix(const std::array<double, 7>& inputArray, Eigen::Matrix<double, 7, 1>& resultMatrix);
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(const Eigen::Matrix<double, 7, 1>& tau_d_calculated, const Eigen::Matrix<double, 7, 1>& tau_J_d);  
    std::array<double, 6> convertToStdArray(const geometry_msgs::msg::WrenchStamped& wrench);
    //State vectors and matrices
    std::array<double, 7> q_subscribed;
    std::array<double, 7> tau_J_d = {0,0,0,0,0,0,0};
    std::array<double, 6> O_F_ext_hat_K = {0,0,0,0,0,0};
    std::array<double, 16> F_T_EE;
    std::array<double, 16> EE_T_K;
    Eigen::Matrix<double, 7, 1> q_subscribed_M;
    Eigen::Matrix<double, 7, 1> tau_J_d_M = Eigen::MatrixXd::Zero(7, 1);
    Eigen::Matrix<double, 6, 1> O_F_ext_hat_K_M = Eigen::MatrixXd::Zero(6,1);
    Eigen::Matrix<double, 7, 1> q_;
    std::array<double, 7> q_as_array;
    Eigen::Vector<double, 7> del_manipulability;
    Eigen::MatrixXd N;
    Eigen::MatrixXd N_pseud;
    Eigen::VectorXd dq_goal;
    Eigen::Matrix<double, 7, 1> dq_;
    Eigen::MatrixXd jacobian_transpose_pinv;  

    //Robot parameters
    const int num_joints = 7;
    const std::string state_interface_name_{"robot_state"};
    const std::string robot_name_{"fr3"};
    const std::string k_robot_state_interface_name{"robot_state"};
    const std::string k_robot_model_interface_name{"robot_model"};
    franka_hardware::FrankaHardwareInterface interfaceClass;
    std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
    const double delta_tau_max_{1.0};
    const double dt = 0.001;
    

    //Joint limit avoidance
    /*Eigen::Matrix<double, 7, 2> joint_limits{
      {-2.7437, 2.7437},
      {-1.7837, 1.7837},
      {-2.9007, 2.9007},
      {-3.0421,-0.1518},
      {-2.8065, 2.8065},
      { 0.5445, 4.5169},
      {-3.0159, 3.0159}
    };*/

    Eigen::Matrix<double, 7, 2> joint_limits{
      {-2.7437, 2.7437},
      {-1.7837, 1.7837},
      {-2.9007, 2.9007},
      {-3.0421,-0.1518},
      {-2.8065, 2.8065},
      { 0.5445, 4.5169},
      {-2.83469, 3.00255}
    };
  
    Eigen::Vector<double, 7> homing_goal{0.0, 0.3, 0.0, -M_PI_2, 0.0, 2.0, M_PI_4};
    double max_torque = 30.0;
    double danger_angle = 0.5;
    bool danger[7] = {};
    Eigen::Vector<double, 3> pos_goal{0.2, 0.0, 0.3};

    //joint space impedance
    Eigen::Matrix<double, 7, 7> kp_joint_space{
    {20,   0,   0,   0,   0,   0,  0},
    {0,   20,   0,   0,   0,   0,  0},
    {0,   0,   60,   0,   0,   0,  0},
    {0,   0,   0,   20,   0,   0,  0},
    {0,   0,   0,   0,   30,   0,  0},
    {0,   0,   0,   0,   0,   10,  0},
    {0,   0,   0,   0,   0,   0,  5}};

    bool recording = false;
    bool singularity_torques_on = true;
    bool joint_limit_torques_on = true;
    unsigned int noSingOff = 1;
    unsigned int noSingOn = 1;
    
    std::string csv_path = "/home/anthonyli/Desktop/Thesis_Data/det(JJT)/Tony/";
    std::string current_user = "Tony"; //only used for naming the .csv file created
    bool button_menu_6_prev = false;
    bool button_4_prev = false;
    Eigen::Vector<double, 3> prev_ee_vel{0.0, 0.0, 0.0};
    std::ofstream file;
    std::chrono::high_resolution_clock::time_point starting_time;
    std::chrono::high_resolution_clock::time_point current_time;
    std::chrono::duration<double> duration;

    //Impedance control variables              
    Eigen::Matrix<double, 6, 6> Lambda = IDENTITY;                                           // operational space mass matrix
    Eigen::Matrix<double, 6, 6> Sm = IDENTITY;                                               // task space selection matrix for positions and rotation
    Eigen::Matrix<double, 6, 6> Sf = Eigen::MatrixXd::Zero(6, 6);                            // task space selection matrix for forces
    Eigen::Matrix<double, 6, 6> K =  (Eigen::MatrixXd(6,6) << 250,   0,   0,   0,   0,   0,
                                                                0, 250,   0,   0,   0,   0,
                                                                0,   0, 250,   0,   0,   0,  // impedance stiffness term
                                                                0,   0,   0, 130,   0,   0,
                                                                0,   0,   0,   0, 130,   0,
                                                                0,   0,   0,   0,   0,  10).finished();

    Eigen::Matrix<double, 6, 6> D =  (Eigen::MatrixXd(6,6) <<  35,   0,   0,   0,   0,   0,
                                                                0,  35,   0,   0,   0,   0,
                                                                0,   0,  35,   0,   0,   0,  // impedance damping term
                                                                0,   0,   0,   25,   0,   0,
                                                                0,   0,   0,   0,   25,   0,
                                                                0,   0,   0,   0,   0,   6).finished();

    // Eigen::Matrix<double, 6, 6> K =  (Eigen::MatrixXd(6,6) << 250,   0,   0,   0,   0,   0,
    //                                                             0, 250,   0,   0,   0,   0,
    //                                                             0,   0, 250,   0,   0,   0,  // impedance stiffness term
    //                                                             0,   0,   0,  80,   0,   0,
    //                                                             0,   0,   0,   0,  80,   0,
    //                                                             0,   0,   0,   0,   0,  10).finished();

    // Eigen::Matrix<double, 6, 6> D =  (Eigen::MatrixXd(6,6) <<  30,   0,   0,   0,   0,   0,
    //                                                             0,  30,   0,   0,   0,   0,
    //                                                             0,   0,  30,   0,   0,   0,  // impedance damping term
    //                                                             0,   0,   0,  18,   0,   0,
    //                                                             0,   0,   0,   0,  18,   0,
    //                                                             0,   0,   0,   0,   0,   9).finished();
    Eigen::Matrix<double, 6, 6> Theta = IDENTITY;
    Eigen::Matrix<double, 6, 6> T = (Eigen::MatrixXd(6,6) <<       1,   0,   0,   0,   0,   0,
                                                                   0,   1,   0,   0,   0,   0,
                                                                   0,   0,   2.5,   0,   0,   0,  // Inertia term
                                                                   0,   0,   0,   1,   0,   0,
                                                                   0,   0,   0,   0,   1,   0,
                                                                   0,   0,   0,   0,   0,   2.5).finished();                                               // impedance inertia term

    Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;                                 // impedance damping term
    Eigen::Matrix<double, 6, 6> cartesian_damping_target_;                                   // impedance damping term
    Eigen::Matrix<double, 6, 6> cartesian_inertia_target_;                                   // impedance damping term
    Eigen::Vector3d position_d_target_ = {0.5, 0.0, 0.5};
    Eigen::Vector3d rotation_d_target_ = {M_PI, 0.0, 0.0};
    Eigen::Quaterniond orientation_d_target_;
    Eigen::Vector3d position_d_;
    Eigen::Quaterniond orientation_d_; 
    Eigen::Matrix<double, 6, 1> F_impedance;  
    Eigen::Matrix<double, 6, 1> F_contact_des = Eigen::MatrixXd::Zero(6, 1);                 // desired contact force
    Eigen::Matrix<double, 6, 1> F_contact_target = Eigen::MatrixXd::Zero(6, 1);              // desired contact force used for filtering
    Eigen::Matrix<double, 6, 1> F_ext = Eigen::MatrixXd::Zero(6, 1);                         // external forces
    Eigen::Matrix<double, 6, 1> F_cmd = Eigen::MatrixXd::Zero(6, 1);                         // commanded contact force
    Eigen::Matrix<double, 7, 1> q_d_nullspace_;
    Eigen::Matrix<double, 6, 1> error;                                                       // pose error (6d)
    double nullspace_stiffness_{0.001};
    double nullspace_stiffness_target_{0.001};

    //Logging
    unsigned int outcounter = 0;
    const int update_frequency = 2; //frequency for update outputs

    //Integrator
    Eigen::Matrix<double, 6, 1> I_error = Eigen::MatrixXd::Zero(6, 1);                      // pose error (6d)
    Eigen::Matrix<double, 6, 1> I_F_error = Eigen::MatrixXd::Zero(6, 1);                    // force error integral
    Eigen::Matrix<double, 6, 1> integrator_weights = 
      (Eigen::MatrixXd(6,1) << 75.0, 75.0, 75.0, 75.0, 75.0, 4.0).finished();
    Eigen::Matrix<double, 6, 1> max_I = 
      (Eigen::MatrixXd(6,1) << 30.0, 30.0, 30.0, 50.0, 50.0, 2.0).finished();

   
  
    std::mutex position_and_orientation_d_target_mutex_;

    //Flags
    bool config_control = false;           // sets if we want to control the configuration of the robot in nullspace
    bool do_logging = false;               // set if we do log values

    //Filter-parameters
    double filter_params_{0.001};
    int mode_ = 2;





    class CartesianPositionSubscriber : public rclcpp::Node {
      public:
      CartesianPositionSubscriber() : Node("cartesian_position_subscriber") {
          subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "desired_EE_pos", 10,
            std::bind(&CartesianPositionSubscriber::twist_callback, this, std::placeholders::_1));
        }
      
        franka_msgs::msg::XBoxController get_latest_twist() {
          return latest_twist_;
        }
      
      private:
        void topic_callback(const geometry_msgs::msg::Twist& msg) {
          latest_twist_ = msg;
        }
      
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
        geometry_msgs::msg::Twist latest_twist_;
      };
      
std::shared_ptr<CartesianPositionSubscriber> sub_node = std::make_shared<CartesianPositionSubscriber>();

}; //class CartesianImpedanceController : public controller_interface::ControllerInterface {
}  // namespace cartesian_impedance_control
