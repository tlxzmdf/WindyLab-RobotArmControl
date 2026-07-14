#pragma once

#include <array>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "BaseDisturbanceGenerator.hpp"
#include "PinocchioArmModel.hpp"
#include "VelocityWbcController.hpp"

class TeleopWbcNode : public rclcpp::Node {
 public:
  TeleopWbcNode();

 private:
  struct TaskMetrics {
    Eigen::Matrix<double, 6, 1> task_error{Eigen::Matrix<double, 6, 1>::Zero()};
    double position_error{0.0};
    double orientation_error{0.0};
  };

  void ControlLoop();
  void PublishJointStates(
      const BaseDisturbanceGenerator::MountPose& mount, const rclcpp::Time& stamp);
  void PublishReferenceJoints(
      const std::array<double, PinocchioArmModel::kDof>& q_plan,
      const std::array<double, PinocchioArmModel::kDof>& q_cmd,
      const rclcpp::Time& stamp);
  void PublishMarkers(
      const BaseDisturbanceGenerator::MountPose& mount, const rclcpp::Time& stamp);
  void PublishMetrics(
      const TaskMetrics& metrics,
      const BaseDisturbanceGenerator::MountPose& mount,
      const rclcpp::Time& stamp);
  void MasterJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void MountDisturbanceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  bool UpdateTeleopTarget(double dt);
  pinocchio::SE3 ComputeMasterEeInWorld() const;
  BaseDisturbanceGenerator::MountPose GetMountPose(double dt);
  pinocchio::SE3 ComputeTargetInBase(
      const BaseDisturbanceGenerator::MountPose& mount) const;
  pinocchio::SE3 ComputeEeInWorld(
      const BaseDisturbanceGenerator::MountPose& mount) const;
  pinocchio::SE3 MountToWorldBase(
      const BaseDisturbanceGenerator::MountPose& mount) const;
  pinocchio::SE3 MountToWorldDrone(
      const BaseDisturbanceGenerator::MountPose& mount) const;
  BaseDisturbanceGenerator::MountPose ApplyMountAnchor(
      const BaseDisturbanceGenerator::MountPose& mount) const;
  std::array<double, 6> MountToJointValues(
      const BaseDisturbanceGenerator::MountPose& mount) const;
  Eigen::Matrix<double, 6, 1> ComputeFeedforwardTaskVelocity(
      const BaseDisturbanceGenerator::MountPose& mount_world, double dt) const;
  TaskMetrics ComputeTaskMetrics(const pinocchio::SE3& T_des_base) const;
  std::array<double, PinocchioArmModel::kDof> SelectContinuousWristBranch(
      const pinocchio::SE3& target,
      const std::array<double, PinocchioArmModel::kDof>& q_candidate,
      const std::array<double, PinocchioArmModel::kDof>& q_ref) const;
  void RunJointMirrorMode(
      const pinocchio::SE3& T_des_base,
      std::array<double, PinocchioArmModel::kDof>* q_plan,
      std::array<double, PinocchioArmModel::kDof>* q_cmd,
      TaskMetrics* metrics);
  void RunEeWbcMode(
      const pinocchio::SE3& T_des_base,
      const Eigen::Matrix<double, 6, 1>& v_feedforward,
      double dt,
      std::array<double, PinocchioArmModel::kDof>* q_plan,
      std::array<double, PinocchioArmModel::kDof>* q_cmd,
      TaskMetrics* metrics);

  std::unique_ptr<PinocchioArmModel> arm_;
  std::unique_ptr<PinocchioArmModel> master_fk_;
  std::unique_ptr<BaseDisturbanceGenerator> disturbance_;
  VelocityWbcController wbc_;
  VelocityWbcController::Params wbc_params_;
  VelocityWbcController::State wbc_state_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr master_joint_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr mount_disturbance_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr error_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr reference_pub_;

  std::vector<std::string> mount_joint_names_;
  std::vector<std::string> arm_joint_names_;
  double control_rate_{500.0};
  bool master_ready_{false};
  bool ref_init_{false};
  bool teleop_vel_init_{false};
  double teleop_target_filter_{0.35};
  int wbc_substeps_{8};
  std::string teleop_control_mode_{"ee_wbc"};
  std::string master_joint_topic_{"/master/joint_states"};
  std::string mount_disturbance_topic_{"/mount_disturbance/pose"};
  std::string base_source_{"static"};
  double mount_base_offset_z_{0.02};
  std::string arm_root_name_{"机载端"};
  std::string arm_ee_name_{"末端"};

  std::array<double, PinocchioArmModel::kDof> q_{};
  std::array<double, PinocchioArmModel::kDof> q_des_filtered_{};
  std::array<double, PinocchioArmModel::kDof> v_{};
  std::array<double, PinocchioArmModel::kDof> master_q_{};
  BaseDisturbanceGenerator::MountPose external_mount_{};
  BaseDisturbanceGenerator::MountPose prev_external_mount_{};
  bool external_mount_ready_{false};
  Eigen::Vector3d mount_anchor_pos_{Eigen::Vector3d::Zero()};
  pinocchio::SE3 ee_target_world_{pinocchio::SE3::Identity()};
  pinocchio::SE3 prev_T_des_base_{pinocchio::SE3::Identity()};
  rclcpp::Time last_joint_stamp_{0, 0, RCL_ROS_TIME};
  uint64_t control_step_{0};
  double last_solve_time_us_{0.0};
  std::deque<Eigen::Vector3d> base_trail_;
  static constexpr size_t kTrailLength = 400;
};
