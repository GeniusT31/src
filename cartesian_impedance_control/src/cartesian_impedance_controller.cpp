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

#include <cartesian_impedance_control/cartesian_impedance_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>
#include "yaml-cpp/yaml.h"
#include <franka/gripper.h>
#include <franka/exception.h>
#include <unsupported/Eigen/MatrixFunctions> //for finding sqrt of matrix

//For writing to csv file
#include <iostream>
#include <fstream>
#include <chrono>
//For

#include <Eigen/Eigen>

namespace {

template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}
}

namespace cartesian_impedance_control {

void CartesianImpedanceController::update_stiffness_and_references(){
  //target by filtering
  /** at the moment we do not use dynamic reconfigure and control the robot via D, K and T **/
  //K = filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * K;
  //D = filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * D;
  nullspace_stiffness_ = filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  //std::lock_guard<std::mutex> position_d_target_mutex_lock(position_and_orientation_d_target_mutex_);
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
  F_contact_des = 0.05 * F_contact_target + 0.95 * F_contact_des;
}


void CartesianImpedanceController::arrayToMatrix(const std::array<double,7>& inputArray, Eigen::Matrix<double,7,1>& resultMatrix)
{
 for(long unsigned int i = 0; i < 7; ++i){
     resultMatrix(i,0) = inputArray[i];
   }
}

void CartesianImpedanceController::arrayToMatrix(const std::array<double,6>& inputArray, Eigen::Matrix<double,6,1>& resultMatrix)
{
 for(long unsigned int i = 0; i < 6; ++i){
     resultMatrix(i,0) = inputArray[i];
   }
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceController::saturateTorqueRate(
  const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
  const Eigen::Matrix<double, 7, 1>& tau_J_d_M) {  
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
  double difference = tau_d_calculated[i] - tau_J_d_M[i];
  tau_d_saturated[i] =
         tau_J_d_M[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}


inline void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped = true) {
  double lambda_ = damped ? 0.01 : 0.0;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);   
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
  Eigen::MatrixXd S_ = M_;  // copying the dimensions of M_, its content is not needed.
  S_.setZero();

  for (int i = 0; i < sing_vals_.size(); i++)
     S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

  M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
}

Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& M, bool damped = true){
  Eigen::MatrixXd inverse;
  pseudoInverse(M, inverse, damped);
  return inverse;
}


Eigen::MatrixXd critical_damping_matrix(const Eigen::MatrixXd& K, const Eigen::MatrixXd& M){
  Eigen::MatrixXd Intermediate = K * M;
  return 2.1 * Intermediate.sqrt();
}

Eigen::MatrixXd critical_damping_matrix(double k, const Eigen::MatrixXd& M){
  return 2.1 * std::sqrt(k) * M.sqrt();
}

Eigen::MatrixXd critical_damping_matrix(const Eigen::MatrixXd& K, const Eigen::MatrixXd& M, const Eigen::MatrixXd& jacobian){
  Eigen::MatrixXd Intermediate = pseudoInverse(jacobian * pseudoInverse(M) * jacobian.transpose());
  return critical_damping_matrix(K, Intermediate);
}

Eigen::MatrixXd critical_damping_matrix(double k, const Eigen::MatrixXd& M, const Eigen::MatrixXd& jacobian){
  Eigen::MatrixXd Intermediate = pseudoInverse(jacobian * pseudoInverse(M) * jacobian.transpose());
  return critical_damping_matrix(k, Intermediate);
}


controller_interface::InterfaceConfiguration
CartesianImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(robot_name_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}


controller_interface::InterfaceConfiguration CartesianImpedanceController::state_interface_configuration()
  const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    state_interfaces_config.names.push_back(robot_name_ + "_joint" + std::to_string(i) + "/position");
    state_interfaces_config.names.push_back(robot_name_ + "_joint" + std::to_string(i) + "/velocity");
  }

  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    state_interfaces_config.names.push_back(franka_robot_model_name);
    std::cout << franka_robot_model_name << std::endl;
  }

  const std::string full_interface_name = robot_name_ + "/" + state_interface_name_;

  return state_interfaces_config;
}


CallbackReturn CartesianImpedanceController::on_init() {
   UserInputServer input_server_obj(&position_d_target_, &rotation_d_target_, &K, &D, &T);
   std::thread input_thread(&UserInputServer::main, input_server_obj, 0, nullptr);
   input_thread.detach();
   return CallbackReturn::SUCCESS;
}


CallbackReturn CartesianImpedanceController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
  franka_semantic_components::FrankaRobotModel(robot_name_ + "/" + k_robot_model_interface_name,
                                               robot_name_ + "/" + k_robot_state_interface_name));
                                               
  try {
    rclcpp::QoS qos_profile(1); // Depth of the message queue
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    franka_state_subscriber = get_node()->create_subscription<franka_msgs::msg::FrankaRobotState>(
    "franka_robot_state_broadcaster/robot_state", qos_profile, 
    std::bind(&CartesianImpedanceController::topic_callback, this, std::placeholders::_1));
    std::cout << "Succesfully subscribed to robot_state_broadcaster" << std::endl;
  }

  catch (const std::exception& e) {
    fprintf(stderr,  "Exception thrown during publisher creation at configure stage with message : %s \n",e.what());
    return CallbackReturn::ERROR;
    }


  RCLCPP_DEBUG(get_node()->get_logger(), "configured successfully");
  return CallbackReturn::SUCCESS;
}


CallbackReturn CartesianImpedanceController::on_activate(
  const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

  std::array<double, 16> initial_pose = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_pose.data()));
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
  std::cout << "Completed Activation process" << std::endl;
  return CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn CartesianImpedanceController::on_deactivate(
  const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

std::array<double, 6> CartesianImpedanceController::convertToStdArray(const geometry_msgs::msg::WrenchStamped& wrench) {
    std::array<double, 6> result;
    result[0] = wrench.wrench.force.x;
    result[1] = wrench.wrench.force.y;
    result[2] = wrench.wrench.force.z;
    result[3] = wrench.wrench.torque.x;
    result[4] = wrench.wrench.torque.y;
    result[5] = wrench.wrench.torque.z;
    return result;
}

/*Eigen::Vector<double, 6> msg_to_6dvector(const geometry_msgs::msg::Twist msg){
  Eigen::Vector<double, 6> result;
  result << msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z;
  return result;
}*/

std::array<double, 16> PoseStampedConvertToStdArray(const geometry_msgs::msg::PoseStamped& Pose){
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

  Eigen::Vector3d translation(Pose.pose.position.x, Pose.pose.position.y, Pose.pose.position.z);
  Eigen::Quaterniond rotation(Pose.pose.orientation.w, Pose.pose.orientation.x, Pose.pose.orientation.y, Pose.pose.orientation.z);

  transform.block<3, 3>(0, 0) = rotation.toRotationMatrix();
  transform.block<3, 1>(0, 3) = translation;

  std::array<double, 16> result;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      result[i * 4 + j] = transform(i, j); // Row-major order
        }
    }
return result;
}

void CartesianImpedanceController::topic_callback(const std::shared_ptr<franka_msgs::msg::FrankaRobotState> msg) {
  O_F_ext_hat_K = convertToStdArray(msg->o_f_ext_hat_k);
  F_T_EE = PoseStampedConvertToStdArray(msg->f_t_ee);
  EE_T_K = PoseStampedConvertToStdArray(msg->ee_t_k);

  arrayToMatrix(O_F_ext_hat_K, O_F_ext_hat_K_M);
}

void CartesianImpedanceController::updateJointStates() {
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);
    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");
    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

double manipulability(const Eigen::MatrixXd& jacobian){
  Eigen::MatrixXd JJT = jacobian * jacobian.transpose();
  return JJT.determinant();
}

Eigen::VectorXd torques_limited(const Eigen::VectorXd torques, bool max = 7.0){
  if(torques.norm() > max){
    return max * torques / torques.norm();
  }
  return torques;
}

controller_interface::return_type CartesianImpedanceController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {  
  // if (outcounter == 0){
  // std::cout << "Enter 1 if you want to track a desired position or 2 if you want to use free floating with optionally shaped inertia" << std::endl;
  // std::cin >> mode_;
  // std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  // std::cout << "Mode selected" << std::endl;
  // while (mode_ != 1 && mode_ != 2){
  //   std::cout << "Invalid mode, try again" << std::endl;
  //   std::cin >> mode_;
  // }
  // }
  updateJointStates();
  std::array<double, 49> mass = franka_robot_model_->getMassMatrix();
  std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();
  std::array<double, 42> jacobian_array =  franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 16> pose = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 7>> M(mass.data());
  Eigen::Map<Eigen::Matrix<double, 4, 4>> Pose(pose.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(pose.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());
  orientation_d_target_ = Eigen::AngleAxisd(rotation_d_target_[0], Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(rotation_d_target_[1], Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(rotation_d_target_[2], Eigen::Vector3d::UnitZ());
  
  error.head(3) << position - position_d_;
  Eigen::MatrixXd cut_jacobian = jacobian.topRows(6);
  for(unsigned int i = 0; i < 7; i++){q_as_array[i] = q_(i);}

  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  error.tail(3) << transform.rotation() * error.tail(3);
  I_error += Sm * dt * integrator_weights.cwiseProduct(error);
  for (int i = 0; i < 6; i++){
    I_error(i,0) = std::min(std::max(-max_I(i,0),  I_error(i,0)), max_I(i,0)); 
  }

  Lambda = (jacobian * M.inverse() * jacobian.transpose()).inverse();
  // Theta = T*Lambda;
  // F_impedance = -1*(Lambda * Theta.inverse() - IDENTITY) * F_ext;
  //Inertia of the robot
  switch (mode_)
  {
  case 1:
    Theta = Lambda;
    F_impedance = -1 * (D * (jacobian * dq_) + K * error /*+ I_error*/);
    break;

  case 2:
    Theta = T*Lambda;
    F_impedance = -1*(Lambda * Theta.inverse() - IDENTITY) * F_ext;
    break;
  
  default:
    break;
  }

  F_ext = 0.9 * F_ext + 0.1 * O_F_ext_hat_K_M; //Filtering 
  I_F_error += dt * Sf* (F_contact_des - F_ext);
  F_cmd = Sf*(0.4 * (F_contact_des - F_ext) + 0.9 * I_F_error + 0.9 * F_contact_des);

  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7), tau_impedance(7);
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                    (nullspace_stiffness_ * config_control * (q_d_nullspace_ - q_) - //if config_control = true we control the whole robot configuration
                    (2.0 * sqrt(nullspace_stiffness_)) * dq_);  // if config control ) false we don't care about the joint position

  tau_impedance = jacobian.transpose() * Sm * (F_impedance /*+ F_repulsion + F_potential*/) + jacobian.transpose() * Sf * F_cmd;

  Eigen::VectorXd joint_limit_torques = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd task_torques = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd singularity_torques = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd damping_torques = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd tau_d_placeholder = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd stationary_torques = Eigen::VectorXd::Zero(7);
 
  for(unsigned int joint_num = 0; joint_num < 7; ++joint_num){
    if(q_(joint_num) < (joint_limits(joint_num, 0) + danger_angle)){
      joint_limit_torques(joint_num) = -(q_(joint_num) - (joint_limits(joint_num, 0) + danger_angle)) * max_torque / danger_angle + 0.5;
      danger[joint_num] = true;
    }
    else if(q_(joint_num) > (joint_limits(joint_num, 1) - danger_angle)){
      joint_limit_torques(joint_num) = -(q_(joint_num) - (joint_limits(joint_num, 1) - danger_angle)) * max_torque / danger_angle - 0.5;
      danger[joint_num] = true;
    }
    else danger[joint_num] = false;
  }
  

  double current_manipulability = manipulability(jacobian);
  double delta_q = 0.01;
  for(unsigned int i = 0; i < 7; ++i){
    q_as_array[i] = q_as_array[i] + delta_q;
    jacobian_array =  franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector, q_as_array, F_T_EE, EE_T_K);
    Eigen::Map<Eigen::Matrix<double, 6, 7>> j_plus_1(jacobian_array.data());
    del_manipulability(i) = (manipulability(j_plus_1) - current_manipulability) / delta_q;
    q_as_array[i] = q_as_array[i] - delta_q;
  }
  
  dq_goal = del_manipulability / 100;
  
  if(current_manipulability * 10000 > 30){
    Eigen::MatrixXd bad_N = Eigen::MatrixXd::Identity(7, 7) - pseudoInverse(cut_jacobian) * cut_jacobian;

    // Compute the SVD
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(bad_N, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd singularValues = svd.singularValues();
    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd V = svd.matrixV();

    // Construct rank-1 approximation: sigma_1 * u_1 * v_1^T
    N = singularValues(0) * U.col(0) * V.col(0).transpose();

    dq_goal = N * del_manipulability;
  }
  dq_goal *= 2000;
  double kd = 11.0;
  singularity_torques = kd * (dq_goal - dq_);
  //singularity_torques = torques_limited(singularity_torques, 30.0);
  stationary_torques = -critical_damping_matrix(kd, M) * dq_;


  //node will be used to receive goal position of end effector in cartesian space
  rclcpp::spin_some(sub_node);
  auto msg = sub_node->get_latest_twist();
  if(sub_node->is_topic_active()){
    pos_goal[0] = msg.linear.x;
    pos_goal[1] = msg.linear.y;
    pos_goal[2] = msg.linear.z;

    
    for(unsigned int i = 0; i < 3; ++i){
      if(pos_goal[i] - position[i] > 0.4){ //might be missing a std::abs()
        pos_goal[i] = 0.4 + position[i];
      }
      else if(pos_goal[i] - position[i] < -0.4){ //might be missing a std::abs()
        pos_goal[i] = -0.4 + position[i];
      }
    }
  }
  //error.topRows(3) = pos_goal - position;
  error.topRows(3) = 2.5 * (pos_goal - position);

  /*
  if(!button_menu_6_prev && msg.button_menu_6){
    singularity_torques_on = !singularity_torques_on;
    if(singularity_torques_on){
      std::cout << "Singularity torques now ON" << std::endl;
    }
    else std::cout << "Singularity torques now OFF" << std::endl;
  }

  if(!button_4_prev && msg.button_4){
    joint_limit_torques_on = !joint_limit_torques_on;
    if(joint_limit_torques_on) std::cout << "joint_limit_torques_on now ON" << std::endl;
    else std::cout << "joint_limit_torques_on now OFF" << std::endl;
  }*/


  if(sub_node->is_topic_active() && !file.is_open()){
    recording = true;
    if(singularity_torques_on) {file.open(csv_path + "singAvoidOn.csv"); }
    else if(!singularity_torques_on) {file.open(csv_path + "singAvoidOff.csv" );}
    std::cout << "Recoding has begun:\n";
    file << "Time,Manipulability,Error\n";
    starting_time = std::chrono::high_resolution_clock::now();
  }
  else if((msg.angular.x < 0.5) && recording){
    if(file.is_open()){
    file.close();
    recording = false;
    std::cout << "Recording has completed" << std::endl;}
  }

  if(recording && (outcounter % 100 == 0) && file.is_open()){
    //writing to file JJT
    current_time = std::chrono::high_resolution_clock::now();
    duration = current_time - starting_time;
    double error_norm = error.topRows(3).norm();
    file << duration.count() << "," << current_manipulability * 10000 << "," << error_norm << "\n";
    file.flush();
  }

  double kp = 30.0;
  task_torques = kp * jacobian.transpose() * (error);

  damping_torques = -jacobian.transpose() * critical_damping_matrix(kp, M, jacobian) * jacobian * dq_; 

  //double ks = 8.0;
  //stationary_torques = -ks * jacobian.transpose() * jacobian * dq_;

  tau_d_placeholder += task_torques;
  tau_d_placeholder += singularity_torques * singularity_torques_on;
  tau_d_placeholder += joint_limit_torques * joint_limit_torques_on;
  tau_d_placeholder += damping_torques;
  //tau_d_placeholder += stationary_torques;

  tau_d << tau_d_placeholder;
  tau_d << saturateTorqueRate(tau_d, tau_J_d_M);  // Saturate torque rate to avoid discontinuities
  tau_J_d_M = tau_d;

  for (size_t i = 0; i < 7; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
  }
  if (outcounter % 1000/update_frequency == 0){
    for(int i = 0; i < 7; i++){
      //std::cout << i+1 << "th joint angle: " << q_(i) << std::endl;
      if(danger[i]){
        std::cout << "Joint " << i + 1 << " is close to its joint limit." << std::endl;
        std::cout << "Reaction torque is " << tau_d(i) << std::endl;}
    }
    //std::cout << error << std::endl;
    //std::cout << "N*N+\n" << 1000 * N * N_pseud << std::endl;
    //std::cout << "Desired EE velocity: \n" << desired_ee_vel << std::endl;
    //std::cout << "Distance to singularity is: " << current_manipulability * 10000 << std::endl;
    //std::cout << "Task torques: " << task_torques.norm() << std::endl;
    //std::cout << "singularity_torques: " << singularity_torques.norm() << std::endl;
    //std::cout << "joint_limit_torques:" << joint_limit_torques.norm() << std::endl;
    //std::cout << "Damping torques: " << damping_torques.norm() << std::endl;
    //std::cout << "Mag of dq_: " << dq_.norm() << std::endl;
    //std::cout << "Joint velocities are:\n" << dq_ << std::endl;
    //std::cout << "All the torques" << tau_d_placeholder << std::endl;
    //std::cout << "position is: " << position << std::endl;
    //std::cout << "dq_goal is: \n" << dq_goal << std::endl;
    //std::cout << "N (Nullspace projection matrix) is: \n" << 100 * N << std::endl;
    //std::cout << std::endl << "task torques: \n" << task_torques << std::endl;
    //std::cout << "joint positions: \n" << q_ << std::endl;
    //std::cout << "dq goal has a norm of:" << norm << std::endl;
    //std::cout << "J * dq_goal is \n" << cut_jacobian * dq_goal << std::endl;
    //std::cout << "J * current q_dot \n" << cut_jacobian * dq_ << std::endl;
    std::cout << "Gloal position is: \n" << pos_goal << std::endl;
 
  }
  outcounter++;
  update_stiffness_and_references();
  return controller_interface::return_type::OK;
}
}

// namespace cartesian_impedance_control
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cartesian_impedance_control::CartesianImpedanceController,
                       controller_interface::ControllerInterface)