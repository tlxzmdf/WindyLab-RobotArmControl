#include "EeStabilizationNode.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>

#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace {
std::array<double, PinocchioDynamicsModel::kDof> DefaultHome() {
  return {0.0, 0.35, -0.55, 0.0, 0.45, 0.0};
}

Eigen::Vector3d QuatToRpy(const Eigen::Quaterniond& q) {
  const tf2::Quaternion tq(q.x(), q.y(), q.z(), q.w());
  tf2::Matrix3x3 m(tq);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  m.getRPY(roll, pitch, yaw);
  return Eigen::Vector3d(roll, pitch, yaw);
}

/// Propagate mount pose one step using analytic translational / rotational rates.
BaseDisturbanceGenerator::MountPose PropagateMountPose(
    const BaseDisturbanceGenerator::MountPose& mount,
    double dt,
    const Eigen::Vector3d& omega_world) {
  BaseDisturbanceGenerator::MountPose next = mount;
  next.position = mount.position + mount.linear_velocity * dt;
  const Eigen::Vector3d wdt = omega_world * dt;
  if (wdt.norm() > 1e-12) {
    const Eigen::Quaterniond dq(
        Eigen::AngleAxisd(wdt.norm(), wdt.normalized()));
    next.orientation = (dq * mount.orientation).normalized();
  }
  next.rpy = QuatToRpy(next.orientation);
  next.linear_velocity = mount.linear_velocity;
  next.angular_velocity = mount.angular_velocity;
  return next;
}

double Clamp(double value, double min_value, double max_value) {
  return std::max(min_value, std::min(max_value, value));
}

double WristPoseCost(
    const PinocchioDynamicsModel& dynamics,
    const pinocchio::SE3& target,
    const std::array<double, PinocchioDynamicsModel::kDof>& q) {
  const pinocchio::SE3 pose =
      dynamics.ComputeEePoseInBase(dynamics.PackQ(q));
  const double pos_err = (pose.translation() - target.translation()).norm();
  const double orient_err =
      pinocchio::log3(pose.rotation().transpose() * target.rotation()).norm();
  return pos_err + orient_err;
}

double JointDistanceToRef(
    const std::array<double, PinocchioDynamicsModel::kDof>& q,
    const std::array<double, PinocchioDynamicsModel::kDof>& q_ref) {
  double dist = 0.0;
  for (size_t i = 0; i < PinocchioDynamicsModel::kDof; ++i) {
    const double delta = PinocchioIkSolver::UnwrapNear(q[i], q_ref[i]) - q_ref[i];
    dist += delta * delta;
  }
  return dist;
}

/// Rx(4)-Rz(5)-Rx(6) 腕部存在 (q4,q6) 与 (q4±π,q6∓π) 等价解，选最接近上一帧的支路。
std::array<double, PinocchioDynamicsModel::kDof> SelectContinuousWristBranch(
    const PinocchioDynamicsModel& dynamics,
    const pinocchio::SE3& target,
    const std::array<double, PinocchioDynamicsModel::kDof>& q_candidate,
    const std::array<double, PinocchioDynamicsModel::kDof>& q_ref) {
  constexpr size_t kJoint4 = 3;
  constexpr size_t kJoint6 = 5;
  const double ref_cost = WristPoseCost(dynamics, target, q_candidate);

  std::array<double, PinocchioDynamicsModel::kDof> best = q_candidate;
  double best_dist = JointDistanceToRef(q_candidate, q_ref);

  for (const double sign : {1.0, -1.0}) {
    std::array<double, PinocchioDynamicsModel::kDof> alt = q_candidate;
    alt[kJoint4] = PinocchioIkSolver::UnwrapNear(
        alt[kJoint4] + sign * M_PI, q_ref[kJoint4]);
    alt[kJoint6] = PinocchioIkSolver::UnwrapNear(
        alt[kJoint6] - sign * M_PI, q_ref[kJoint6]);
    const double alt_cost = WristPoseCost(dynamics, target, alt);
    if (alt_cost > ref_cost + 0.025) {
      continue;
    }
    const double alt_dist = JointDistanceToRef(alt, q_ref);
    if (alt_dist + 1e-6 < best_dist) {
      best = alt;
      best_dist = alt_dist;
    }
  }

  return best;
}

geometry_msgs::msg::Pose Se3ToPose(const pinocchio::SE3& se3) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = se3.translation().x();
  pose.position.y = se3.translation().y();
  pose.position.z = se3.translation().z();
  const Eigen::Quaterniond quat(se3.rotation());
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  pose.orientation.w = quat.w();
  return pose;
}

visualization_msgs::msg::Marker MakeTextLabel(
    int id,
    const std::string& text,
    const geometry_msgs::msg::Pose& anchor,
    float r,
    float g,
    float b,
    const rclcpp::Time& stamp,
    const rclcpp::Duration& lifetime,
    double z_offset = 0.06) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = stamp;
  marker.ns = "stabilization";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = anchor;
  marker.pose.position.z += z_offset;
  marker.pose.orientation.w = 1.0;
  marker.scale.z = 0.045;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0f;
  marker.text = text;
  marker.lifetime = lifetime;
  return marker;
}
}  // namespace

EeStabilizationNode::EeStabilizationNode() : Node("ee_stabilization") {
  mount_joint_names_ = {
      "mount_tx", "mount_ty", "mount_tz", "mount_rx", "mount_ry", "mount_rz"};
  arm_joint_names_ = {
      "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

  declare_parameter<std::string>("urdf_path", "");
  declare_parameter<std::string>("base_frame", "base_link");
  declare_parameter<std::string>("ee_frame", "link6");
  declare_parameter<double>("control_rate", 500.0);
  declare_parameter<double>("torque_limit", 45.0);
  declare_parameter<double>("osc_lambda", 0.06);
  declare_parameter<double>("disturbance_radius", 0.12);
  declare_parameter<double>("disturbance_orient_amp", 0.18);
  declare_parameter<double>("disturbance_time_constant", 1.0);
  declare_parameter<double>("disturbance_amplitude_scale", 0.92);
  declare_parameter<std::vector<double>>("sphere_center", {0.0, 0.0, 0.0});
  declare_parameter<std::vector<double>>(
      "kp_task", {600.0, 600.0, 600.0, 300.0, 300.0, 300.0});
  declare_parameter<std::vector<double>>(
      "kd_task", {60.0, 60.0, 60.0, 30.0, 30.0, 30.0});
  declare_parameter<std::vector<double>>(
      "joint_kp", {320.0, 320.0, 320.0, 240.0, 240.0, 240.0});
  declare_parameter<std::vector<double>>(
      "joint_kd", {42.0, 42.0, 42.0, 32.0, 32.0, 32.0});
  declare_parameter<double>("q_des_filter", 0.93);
  declare_parameter<bool>("use_ik_joint_control", true);
  declare_parameter<bool>("kinematic_stabilization", true);
  declare_parameter<std::string>("stabilization_mode", "");
  declare_parameter<double>("mode_d_alpha", 1.0);
  declare_parameter<double>("mode_d_kd_sat", 2.5);
  declare_parameter<double>("mode_d_sat_a", 10.0);
  declare_parameter<double>("mode_d_sat_epsilon", 0.08);
  declare_parameter<int>("mode_d_potential_N", 2);
  declare_parameter<std::vector<double>>(
      "mode_d_sigma_sq", {1.0e-6, 1.0e-6, 1.0e-6, 2.5e-4, 2.5e-4, 2.5e-4});
  declare_parameter<std::vector<double>>(
      "mode_d_potential_kp", std::vector<double>{});
  declare_parameter<double>("mode_d_clik_damping", 0.05);
  declare_parameter<std::vector<double>>(
      "mode_d_sat_vdes_gain", {0.06, 0.06, 0.06, 0.045, 0.045, 0.045});
  declare_parameter<std::vector<double>>(
      "mode_d_sat_task_ff", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  declare_parameter<double>("mode_d_ndo_gain", 5.0);
  declare_parameter<double>("mode_d_ndo_torque_limit", 10.0);
  declare_parameter<int>("ik_cycle_iters", 14);
  declare_parameter<int>("ik_refine_iters", 22);
  declare_parameter<int>("ik_recovery_iters", 36);
  declare_parameter<int>("ik_validate_iters", 12);
  declare_parameter<double>("ik_reach_pos_tol", 0.015);
  declare_parameter<double>("ik_reach_orient_tol", 0.10);
  declare_parameter<std::vector<double>>("mount_anchor", {0.0, 0.0, 0.0});
  declare_parameter<std::vector<double>>("q_home", std::vector<double>{});
  declare_parameter<double>("mount_base_offset_z", 0.02);
  declare_parameter<std::string>("arm_root_name", "机载端");
  declare_parameter<std::string>("arm_ee_name", "末端");
  declare_parameter<bool>("hardware_mode", false);
  declare_parameter<std::string>("base_source", "simulated");
  declare_parameter<std::string>("mount_disturbance_topic", "/mount_disturbance/pose");
  declare_parameter<std::string>("world_frame", "world");
  declare_parameter<int>("hardware_dof", 7);
  declare_parameter<double>("joint7_value", 0.0);
  declare_parameter<double>("hw_torque_limit", 9.0);
  declare_parameter<std::string>("joint_command_topic", "/student/joint_command");
  declare_parameter<std::string>("joint_feedback_topic", "/joint_states");
  declare_parameter<std::vector<double>>(
      "clik_kp", {8.0, 8.0, 8.0, 6.0, 6.0, 6.0});
  declare_parameter<double>("clik_damping", 0.05);
  declare_parameter<double>("max_joint_velocity", 3.0);
  declare_parameter<double>("ctc_vd_scale", 0.95);
  declare_parameter<double>("tf_velocity_filter_alpha", 0.85);

  declare_parameter<bool>("teleop_mode", false);
  declare_parameter<std::string>("master_joint_topic", "/master/joint_states");
  declare_parameter<std::string>("master_urdf_path", "");
  declare_parameter<double>("teleop_target_filter", 0.35);
  declare_parameter<double>("teleop_ik_nullspace_gain", 0.35);
  declare_parameter<double>("teleop_wrist_singularity_damping_scale", 0.08);
  declare_parameter<int>("teleop_clik_substeps", 8);
  declare_parameter<std::vector<double>>(
      "teleop_clik_kp", std::vector<double>{});
  declare_parameter<std::string>("teleop_control_mode", "ee_stabilization");
  declare_parameter<double>("master_feedback_timeout_sec", 1.0);

  const auto urdf_path = get_parameter("urdf_path").as_string();
  const auto base_frame = get_parameter("base_frame").as_string();
  const auto ee_frame = get_parameter("ee_frame").as_string();
  control_rate_ = get_parameter("control_rate").as_double();
  torque_limit_ = get_parameter("torque_limit").as_double();
  osc_gains_.lambda = get_parameter("osc_lambda").as_double();
  mount_base_offset_z_ = get_parameter("mount_base_offset_z").as_double();
  arm_root_name_ = get_parameter("arm_root_name").as_string();
  arm_ee_name_ = get_parameter("arm_ee_name").as_string();
  hardware_mode_ = get_parameter("hardware_mode").as_bool();
  base_source_ = get_parameter("base_source").as_string();
  world_frame_ = get_parameter("world_frame").as_string();
  base_frame_name_ = base_frame;
  hardware_dof_ = static_cast<size_t>(get_parameter("hardware_dof").as_int());
  joint7_value_ = get_parameter("joint7_value").as_double();
  hw_torque_limit_ = get_parameter("hw_torque_limit").as_double();
  if (hardware_mode_ && control_rate_ > 150.0) {
    control_rate_ = 100.0;
  }

  const auto kp_vec = get_parameter("kp_task").as_double_array();
  const auto kd_vec = get_parameter("kd_task").as_double_array();
  for (size_t i = 0; i < 6; ++i) {
    osc_gains_.kp(static_cast<Eigen::Index>(i)) = kp_vec[i];
    osc_gains_.kd(static_cast<Eigen::Index>(i)) = kd_vec[i];
  }

  std::array<double, PinocchioDynamicsModel::kDof> q_home = DefaultHome();
  const auto q_home_vec = get_parameter("q_home").as_double_array();
  if (q_home_vec.size() == PinocchioDynamicsModel::kDof) {
    for (size_t i = 0; i < PinocchioDynamicsModel::kDof; ++i) {
      q_home[i] = q_home_vec[i];
    }
  }

  q_ = q_home;
  q_des_filtered_ = q_home;
  v_.fill(0.0);

  dynamics_ = std::make_unique<PinocchioDynamicsModel>(
      urdf_path, arm_joint_names_, base_frame, ee_frame);

  PinocchioIkSolver::Params ik_params;
  ik_params.max_iters = 14;
  ik_params.position_tolerance = 5e-4;
  ik_params.orientation_tolerance = 8e-3;
  ik_params.partial_position_tolerance = 0.004;
  ik_params.partial_orientation_tolerance = 0.05;
  ik_params.orientation_weight = 1.0;
  ik_params.damping = 0.012;
  ik_params.step_scale = 0.9;
  ik_params.max_step = 0.2;
  ik_solver_ = std::make_unique<PinocchioIkSolver>(
      urdf_path, arm_joint_names_, base_frame, ee_frame, ik_params);

  q_des_filter_alpha_ = get_parameter("q_des_filter").as_double();
  use_ik_joint_control_ = get_parameter("use_ik_joint_control").as_bool();
  kinematic_stabilization_ = get_parameter("kinematic_stabilization").as_bool();
  const std::string stabilization_mode =
      get_parameter("stabilization_mode").as_string();
  if (stabilization_mode == "D" || stabilization_mode == "d") {
    use_mode_d_ = true;
    use_ik_joint_control_ = false;
    kinematic_stabilization_ = false;
  } else if (stabilization_mode == "A" || stabilization_mode == "a") {
    use_mode_d_ = false;
    use_ik_joint_control_ = true;
    kinematic_stabilization_ = true;
  } else if (stabilization_mode == "B" || stabilization_mode == "b") {
    use_mode_d_ = false;
    use_ik_joint_control_ = true;
    kinematic_stabilization_ = false;
  } else if (stabilization_mode == "C" || stabilization_mode == "c") {
    use_mode_d_ = false;
    use_ik_joint_control_ = false;
    kinematic_stabilization_ = false;
  }
  use_torque_feedforward_ =
      hardware_mode_ && (!kinematic_stabilization_ || use_mode_d_);
  ik_cycle_iters_ = get_parameter("ik_cycle_iters").as_int();
  ik_refine_iters_ = get_parameter("ik_refine_iters").as_int();
  ik_recovery_iters_ = get_parameter("ik_recovery_iters").as_int();
  ik_validate_iters_ = get_parameter("ik_validate_iters").as_int();
  ik_reach_pos_tol_ = get_parameter("ik_reach_pos_tol").as_double();
  ik_reach_orient_tol_ = get_parameter("ik_reach_orient_tol").as_double();
  const auto anchor_vec = get_parameter("mount_anchor").as_double_array();
  if (anchor_vec.size() == 3) {
    mount_anchor_pos_ =
        Eigen::Vector3d(anchor_vec[0], anchor_vec[1], anchor_vec[2]);
  }
  mount_anchor_set_ = true;
  const auto kp_joint_vec = get_parameter("joint_kp").as_double_array();
  const auto kd_joint_vec = get_parameter("joint_kd").as_double_array();
  kp_joint_.resize(6);
  kd_joint_.resize(6);
  for (size_t i = 0; i < 6; ++i) {
    kp_joint_(static_cast<Eigen::Index>(i)) = kp_joint_vec[i];
    kd_joint_(static_cast<Eigen::Index>(i)) = kd_joint_vec[i];
  }
  const auto clik_kp_vec = get_parameter("clik_kp").as_double_array();
  for (size_t i = 0; i < 6; ++i) {
    clik_kp_(static_cast<Eigen::Index>(i)) = clik_kp_vec[i];
  }
  clik_damping_ = get_parameter("clik_damping").as_double();
  max_joint_velocity_ = get_parameter("max_joint_velocity").as_double();
  tf_velocity_filter_alpha_ = get_parameter("tf_velocity_filter_alpha").as_double();
  ctc_vd_scale_ = get_parameter("ctc_vd_scale").as_double();

  if (use_mode_d_) {
    StabilizationModeD::Params mode_d_params;
    mode_d_params.alpha = get_parameter("mode_d_alpha").as_double();
    mode_d_params.kd_sat = get_parameter("mode_d_kd_sat").as_double();
    mode_d_params.sat_a = get_parameter("mode_d_sat_a").as_double();
    mode_d_params.sat_epsilon = get_parameter("mode_d_sat_epsilon").as_double();
    mode_d_params.potential_N =
        std::max(1, static_cast<int>(get_parameter("mode_d_potential_N").as_int()));
    mode_d_params.clik_damping = get_parameter("mode_d_clik_damping").as_double();
    mode_d_params.max_joint_velocity = max_joint_velocity_;
    const auto sat_ff_vec = get_parameter("mode_d_sat_task_ff").as_double_array();
    const auto sat_vdes_vec = get_parameter("mode_d_sat_vdes_gain").as_double_array();
    for (size_t i = 0; i < 6; ++i) {
      if (sat_ff_vec.size() == 6) {
        mode_d_params.sat_task_ff(static_cast<Eigen::Index>(i)) = sat_ff_vec[i];
      }
      if (sat_vdes_vec.size() == 6) {
        mode_d_params.sat_vdes_gain(static_cast<Eigen::Index>(i)) = sat_vdes_vec[i];
      }
    }
    mode_d_params.ndo_gain = get_parameter("mode_d_ndo_gain").as_double();
    mode_d_params.ndo_torque_limit =
        get_parameter("mode_d_ndo_torque_limit").as_double();
    const auto sigma_vec = get_parameter("mode_d_sigma_sq").as_double_array();
    const auto pot_kp_vec = get_parameter("mode_d_potential_kp").as_double_array();
    for (size_t i = 0; i < 6; ++i) {
      if (sigma_vec.size() == 6) {
        mode_d_params.sigma_sq(static_cast<Eigen::Index>(i)) = sigma_vec[i];
      }
      if (pot_kp_vec.size() == 6) {
        mode_d_params.potential_kp(static_cast<Eigen::Index>(i)) = pot_kp_vec[i];
      } else {
        mode_d_params.potential_kp(static_cast<Eigen::Index>(i)) =
            osc_gains_.kp(static_cast<Eigen::Index>(i));
      }
    }
    mode_d_ = std::make_unique<StabilizationModeD>(mode_d_params);
    mode_d_->Reset(q_home);
  }

  teleop_mode_ = get_parameter("teleop_mode").as_bool();
  master_joint_topic_ = get_parameter("master_joint_topic").as_string();
  mount_disturbance_topic_ = get_parameter("mount_disturbance_topic").as_string();
  teleop_target_filter_ = get_parameter("teleop_target_filter").as_double();
  teleop_ik_nullspace_gain_ = get_parameter("teleop_ik_nullspace_gain").as_double();
  teleop_wrist_singularity_damping_scale_ =
      get_parameter("teleop_wrist_singularity_damping_scale").as_double();
  teleop_clik_substeps_ = std::max(
      1, static_cast<int>(get_parameter("teleop_clik_substeps").as_int()));
  const auto teleop_clik_kp_vec = get_parameter("teleop_clik_kp").as_double_array();
  teleop_clik_kp_ = clik_kp_;
  if (teleop_clik_kp_vec.size() == 6) {
    for (size_t i = 0; i < 6; ++i) {
      teleop_clik_kp_(static_cast<Eigen::Index>(i)) = teleop_clik_kp_vec[i];
    }
  }
  teleop_control_mode_ = get_parameter("teleop_control_mode").as_string();
  const double master_feedback_timeout =
      get_parameter("master_feedback_timeout_sec").as_double();

  const auto center_vec = get_parameter("sphere_center").as_double_array();
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  if (center_vec.size() == 3) {
    center = Eigen::Vector3d(center_vec[0], center_vec[1], center_vec[2]);
  }
  disturbance_ = std::make_unique<BaseDisturbanceGenerator>(
      center,
      get_parameter("disturbance_radius").as_double(),
      get_parameter("disturbance_orient_amp").as_double(),
      get_parameter("disturbance_time_constant").as_double(),
      get_parameter("disturbance_amplitude_scale").as_double());

  SetupDisturbanceValidator();

  hardware_joint_names_.reserve(hardware_dof_);
  for (size_t i = 1; i <= std::min(hardware_dof_, static_cast<size_t>(6)); ++i) {
    hardware_joint_names_.push_back("joint" + std::to_string(i));
  }
  if (hardware_dof_ >= 7) {
    hardware_joint_names_.push_back("joint7");
  }

  if (hardware_mode_) {
    const auto feedback_topic = get_parameter("joint_feedback_topic").as_string();
    joint_feedback_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        feedback_topic, rclcpp::SensorDataQoS(),
        std::bind(&EeStabilizationNode::JointFeedbackCallback, this, std::placeholders::_1));
    const auto cmd_topic = get_parameter("joint_command_topic").as_string();
    student_cmd_pub_ = create_publisher<sensor_msgs::msg::JointState>(cmd_topic, 10);
    if (base_source_ == "tf") {
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }
  } else {
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    if (base_source_ == "external") {
      mount_disturbance_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
          mount_disturbance_topic_, rclcpp::SensorDataQoS(),
          std::bind(
              &EeStabilizationNode::MountDisturbanceCallback, this, std::placeholders::_1));
    }
  }

  if (teleop_mode_) {
    const auto master_urdf = get_parameter("master_urdf_path").as_string();
    const std::string master_model_path =
        master_urdf.empty() ? urdf_path : master_urdf;
    master_fk_ = std::make_unique<PinocchioDynamicsModel>(
        master_model_path, arm_joint_names_, base_frame, ee_frame);
    master_joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        master_joint_topic_, rclcpp::SensorDataQoS(),
        std::bind(&EeStabilizationNode::MasterJointCallback, this, std::placeholders::_1));
    (void)master_feedback_timeout;
  }
  marker_array_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/stabilization_markers", rclcpp::QoS(1).transient_local());
  error_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/stabilization_error", 10);
  reference_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "/stabilization_reference", 10);

  const auto period = std::chrono::duration<double>(1.0 / control_rate_);
  control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&EeStabilizationNode::ControlLoop, this));

  RCLCPP_INFO(
      get_logger(),
      "Control: %s%s | output=%s | base=%s | rate=%.0f Hz%s%s",
      use_mode_d_ ? "Mode-D sat+OSC+ESO"
                  : (use_ik_joint_control_ ? "IK+joint hold" : "OSC"),
      (kinematic_stabilization_ && !use_mode_d_) ? " (position MIT)"
                                                 : " (torque MIT ff)",
      hardware_mode_ ? "hardware" : "simulation",
      base_source_.c_str(),
      control_rate_,
      teleop_mode_ ? " | teleop=ON" : "",
      teleop_mode_ ? (" | teleop_mode=" + teleop_control_mode_).c_str() : "");
  RCLCPP_INFO(
      get_logger(),
      "Arm endpoints: [%s]=%s, [%s]=%s",
      arm_root_name_.c_str(),
      base_frame.c_str(),
      arm_ee_name_.c_str(),
      ee_frame.c_str());
}

pinocchio::SE3 EeStabilizationNode::MountToWorldDrone(
    const BaseDisturbanceGenerator::MountPose& mount) const {
  const pinocchio::SE3 T_trans(Eigen::Matrix3d::Identity(), mount.position);
  const pinocchio::SE3 T_rx(
      Eigen::AngleAxisd(mount.rpy.x(), Eigen::Vector3d::UnitX()).toRotationMatrix(),
      Eigen::Vector3d::Zero());
  const pinocchio::SE3 T_ry(
      Eigen::AngleAxisd(mount.rpy.y(), Eigen::Vector3d::UnitY()).toRotationMatrix(),
      Eigen::Vector3d::Zero());
  const pinocchio::SE3 T_rz(
      Eigen::AngleAxisd(mount.rpy.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix(),
      Eigen::Vector3d::Zero());
  return T_trans * T_rx * T_ry * T_rz;
}

pinocchio::SE3 EeStabilizationNode::MountToWorldBase(
    const BaseDisturbanceGenerator::MountPose& mount) const {
  const pinocchio::SE3 T_mount_to_base(
      Eigen::Matrix3d::Identity(),
      Eigen::Vector3d(0.0, 0.0, mount_base_offset_z_));
  return MountToWorldDrone(mount) * T_mount_to_base;
}

pinocchio::SE3 EeStabilizationNode::ComputeTargetInBase(
    const BaseDisturbanceGenerator::MountPose& mount) const {
  return MountToWorldBase(mount).actInv(ee_target_world_);
}

Eigen::Matrix<double, 6, 1> EeStabilizationNode::ComputeDesiredTaskVelocityAnalytic(
    const BaseDisturbanceGenerator::MountPose& mount_world, double dt) const {
  const pinocchio::SE3 T_wb = MountToWorldBase(mount_world);
  const pinocchio::SE3 T_drone = MountToWorldDrone(mount_world);
  const Eigen::Matrix3d R_drone = T_drone.rotation();
  const Eigen::Vector3d offset_z(0.0, 0.0, mount_base_offset_z_);
  const Eigen::Matrix3d Rx =
      Eigen::AngleAxisd(mount_world.rpy.x(), Eigen::Vector3d::UnitX()).toRotationMatrix();
  const Eigen::Matrix3d Ry =
      Eigen::AngleAxisd(mount_world.rpy.y(), Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Eigen::Vector3d omega_world =
      Eigen::Vector3d::UnitX() * mount_world.angular_velocity.x()
      + Rx * Eigen::Vector3d::UnitY() * mount_world.angular_velocity.y()
      + Rx * Ry * Eigen::Vector3d::UnitZ() * mount_world.angular_velocity.z();

  const Eigen::Vector3d v_base_w =
      mount_world.linear_velocity + omega_world.cross(R_drone * offset_z);
  const Eigen::Vector3d p_rel_w =
      ee_target_world_.translation() - T_wb.translation();
  const Eigen::Matrix3d R_wb = T_wb.rotation();

  Eigen::Matrix<double, 6, 1> v_des;
  v_des.head<3>() = -R_wb.transpose() * (v_base_w + omega_world.cross(p_rel_w));

  const pinocchio::SE3 T_now = ComputeTargetInBase(mount_world);
  const auto mount_next = PropagateMountPose(mount_world, dt, omega_world);
  const pinocchio::SE3 T_next = ComputeTargetInBase(mount_next);
  v_des.tail<3>() = pinocchio::log3(
                        T_now.rotation().transpose() * T_next.rotation()) /
                    std::max(dt, 1e-6);

  for (int i = 0; i < 6; ++i) {
    v_des[i] = Clamp(v_des[i], -15.0, 15.0);
  }
  return v_des;
}

std::array<double, PinocchioDynamicsModel::kDof>
EeStabilizationNode::ComputeHardwareJointVelocity(
    const pinocchio::SE3& T_des_base,
    const Eigen::Matrix<double, 6, 1>& v_task_des) const {
  const Eigen::VectorXd q = dynamics_->PackQ(q_);
  return dynamics_->ComputeClikJointVelocity(
      q, T_des_base, v_task_des, clik_kp_, clik_damping_, max_joint_velocity_);
}

BaseDisturbanceGenerator::MountPose EeStabilizationNode::ApplyMountAnchor(
    const BaseDisturbanceGenerator::MountPose& mount) const {
  BaseDisturbanceGenerator::MountPose world = mount;
  world.position = mount_anchor_pos_ + mount.position;
  world.rpy = mount_anchor_rpy_ + mount.rpy;
  const Eigen::AngleAxisd roll(world.rpy.x(), Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch(world.rpy.y(), Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw(world.rpy.z(), Eigen::Vector3d::UnitZ());
  world.orientation = (yaw * pitch * roll).normalized();
  return world;
}

pinocchio::SE3 EeStabilizationNode::ComputeEeInWorld(
    const BaseDisturbanceGenerator::MountPose& mount) const {
  const pinocchio::SE3 T_base_ee = dynamics_->ComputeEePoseInBase(dynamics_->PackQ(q_));
  return MountToWorldBase(mount) * T_base_ee;
}

std::array<double, 6> EeStabilizationNode::MountToJointValues(
    const BaseDisturbanceGenerator::MountPose& mount) const {
  return {
      mount.position.x(),
      mount.position.y(),
      mount.position.z(),
      mount.rpy.x(),
      mount.rpy.y(),
      mount.rpy.z()};
}

void EeStabilizationNode::SetupDisturbanceValidator() {
  disturbance_->SetGoalValidator([this](const BaseDisturbanceGenerator::MountPose& offset) {
    if (!ref_init_) {
      return true;
    }
    return IsMountOffsetReachable(offset);
  });
}

bool EeStabilizationNode::IsMountOffsetReachable(
    const BaseDisturbanceGenerator::MountPose& offset_pose) {
  const auto mount_world = ApplyMountAnchor(offset_pose);
  const pinocchio::SE3 T_des_base = ComputeTargetInBase(mount_world);

  const auto q_backup = ik_solver_->current_q();
  ik_solver_->set_current_q(q_);
  const auto result = ik_solver_->SolveSe3(T_des_base, ik_validate_iters_);
  ik_solver_->set_current_q(q_backup);

  return result.position_error < ik_reach_pos_tol_ &&
         result.orientation_error < ik_reach_orient_tol_;
}

void EeStabilizationNode::ControlLoop() {
  if (hardware_mode_ && !feedback_ready_) {
    if (control_step_ % static_cast<uint64_t>(control_rate_) == 0) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Waiting for joint feedback on /joint_states ...");
    }
    return;
  }

  if (teleop_mode_ && !hardware_mode_ && !master_ready_) {
    if (control_step_ % static_cast<uint64_t>(control_rate_) == 0) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Waiting for master feedback on %s ...", master_joint_topic_.c_str());
    }
    return;
  }

  auto stamp = now();
  if (stamp.nanoseconds() <= last_joint_stamp_.nanoseconds()) {
    stamp = last_joint_stamp_ + rclcpp::Duration::from_nanoseconds(2000000);
  }
  last_joint_stamp_ = stamp;
  const double dt = 1.0 / control_rate_;
  const auto mount = GetMountPose(dt);
  const auto mount_world = ApplyMountAnchor(mount);

  const Eigen::VectorXd q = dynamics_->PackQ(q_);
  const Eigen::VectorXd v = dynamics_->PackV(v_);

  if (teleop_mode_) {
    UpdateTeleopTarget(dt);
  } else if (!ref_init_) {
    ee_target_world_ =
        MountToWorldBase(mount_world) * dynamics_->ComputeEePoseInBase(q);
    ref_init_ = true;
    if (mode_d_) {
      mode_d_->Reset(q_);
    }
    RCLCPP_INFO(
        get_logger(),
        "Locked world EE target: [%.3f, %.3f, %.3f]",
        ee_target_world_.translation().x(),
        ee_target_world_.translation().y(),
        ee_target_world_.translation().z());
  }

  const pinocchio::SE3 T_des_base = ComputeTargetInBase(mount_world);
  Eigen::Matrix<double, 6, 1> v_des;
  if (teleop_mode_ && teleop_vel_init_) {
    const pinocchio::SE3 delta = T_des_base * prev_T_des_base_.inverse();
    v_des = pinocchio::log6(delta) / std::max(dt, 1e-6);
    for (int i = 0; i < 6; ++i) {
      v_des[i] = Clamp(v_des[i], -15.0, 15.0);
    }
  } else {
    v_des = ComputeDesiredTaskVelocityAnalytic(mount_world, dt);
  }
  prev_T_des_base_ = T_des_base;
  teleop_vel_init_ = true;

  Eigen::VectorXd tau = Eigen::VectorXd::Zero(dynamics_->model().nv);
  PinocchioDynamicsModel::OperationalResult osc_result;
  std::array<double, PinocchioDynamicsModel::kDof> q_plan{};
  std::array<double, PinocchioDynamicsModel::kDof> q_cmd{};

  const bool joint_mirror =
      teleop_mode_ && master_ready_ && teleop_control_mode_ == "joint_mirror";

  if (joint_mirror) {
    std::array<double, PinocchioDynamicsModel::kDof> q_des{};
    for (size_t i = 0; i < PinocchioDynamicsModel::kDof; ++i) {
      q_des[i] = PinocchioIkSolver::UnwrapNear(master_q_[i], q_[i]);
    }
    const double alpha = kinematic_stabilization_ ? 0.0 : q_des_filter_alpha_;
    const double beta = 1.0 - alpha;
    for (size_t i = 0; i < PinocchioDynamicsModel::kDof; ++i) {
      q_des_filtered_[i] = alpha * q_des_filtered_[i] + beta * q_des[i];
    }
    if (!hardware_mode_) {
      q_ = q_des_filtered_;
      v_.fill(0.0);
    }
    const pinocchio::SE3 T_ee = dynamics_->ComputeEePoseInBase(q);
    osc_result.task_error.head<3>() =
        T_des_base.translation() - T_ee.translation();
    osc_result.task_error.tail<3>() = pinocchio::log3(
        T_ee.rotation().transpose() * T_des_base.rotation());
    osc_result.position_error = osc_result.task_error.head<3>().norm();
    osc_result.orientation_error = osc_result.task_error.tail<3>().norm();
    q_plan = q_des;
    q_cmd = q_des_filtered_;
  } else if (use_mode_d_) {
    const StabilizationModeD::Output mode_d_out = mode_d_->Step(
        *dynamics_, q_, v_, T_des_base, v_des, dt, osc_gains_, ik_solver_.get(),
        ik_cycle_iters_);
    tau = mode_d_out.tau;
    osc_result = mode_d_out.metrics;
    q_plan = mode_d_out.q_plan;
    q_cmd = mode_d_out.q_plan;
    mode_d_dq_ref_ = mode_d_out.dq_ref;
  } else if (use_ik_joint_control_) {
    const bool teleop_ee =
        teleop_mode_ && master_ready_ && teleop_control_mode_ == "ee_stabilization";
    std::array<double, PinocchioDynamicsModel::kDof> q_des{};

    if (teleop_ee) {
      const auto solve_t0 = std::chrono::steady_clock::now();
      const double q5 = q_[4];
      const double singularity_scale =
          1.0 + teleop_wrist_singularity_damping_scale_ / (std::abs(q5) + 0.12);
      const double clik_damp = clik_damping_ * singularity_scale;
      const double sub_dt = dt / static_cast<double>(teleop_clik_substeps_);
      q_des = q_;
      for (int sub = 0; sub < teleop_clik_substeps_; ++sub) {
        const Eigen::VectorXd q_sub = dynamics_->PackQ(q_des);
        const auto dq_arr = dynamics_->ComputeClikJointVelocityWithNullSpace(
            q_sub, T_des_base, v_des, q_, teleop_clik_kp_, clik_damp,
            teleop_ik_nullspace_gain_, max_joint_velocity_);
        for (size_t i = 0; i < PinocchioDynamicsModel::kDof; ++i) {
          q_des[i] = PinocchioIkSolver::UnwrapNear(
              q_des[i] + dq_arr[i] * sub_dt, q_[i]);
        }
      }
      q_des = SelectContinuousWristBranch(*dynamics_, T_des_base, q_des, q_);
      ik_solver_->set_current_q(q_des);
      last_solve_time_us_ = static_cast<double>(
          std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::steady_clock::now() - solve_t0)
              .count());
    } else {
      ik_solver_->set_current_q(q_);
      if (teleop_mode_ && master_ready_) {
        std::array<double, PinocchioIkSolver::kDof> ref{};
        for (size_t i = 0; i < PinocchioDynamicsModel::kDof; ++i) {
          ref[i] = PinocchioIkSolver::UnwrapNear(master_q_[i], q_[i]);
        }
        ik_solver_->set_reference_q(ref, teleop_ik_nullspace_gain_);
      } else {
        ik_solver_->clear_reference_q();
      }
      auto ik_result = ik_solver_->SolveSe3(T_des_base, ik_cycle_iters_);
      if (!ik_result.acceptable) {
        ik_result = ik_solver_->RefineSe3(T_des_base, ik_refine_iters_);
      }
      if (!ik_result.acceptable && ik_recovery_iters_ > 0) {
        ik_result = ik_solver_->RefineSe3(T_des_base, ik_recovery_iters_);
      }
      q_des = ik_solver_->current_q();
      if (teleop_mode_ && master_ready_) {
        for (size_t i = 0; i < PinocchioDynamicsModel::kDof; ++i) {
          q_des[i] = PinocchioIkSolver::UnwrapNear(q_des[i], q_[i]);
        }
      }
    }

    const double alpha = kinematic_stabilization_ ? 0.0 : q_des_filter_alpha_;
    const double beta = 1.0 - alpha;
    for (size_t i = 0; i < PinocchioDynamicsModel::kDof; ++i) {
      q_des_filtered_[i] = alpha * q_des_filtered_[i] + beta * q_des[i];
    }

    if (kinematic_stabilization_) {
      if (!hardware_mode_) {
        q_ = q_des_filtered_;
        v_.fill(0.0);
      }
    } else {
      const Eigen::VectorXd q_d = dynamics_->PackQ(q_des_filtered_);
      Eigen::VectorXd v_d = Eigen::VectorXd::Zero(dynamics_->model().nv);
      const std::array<double, PinocchioDynamicsModel::kDof> v_d_arr =
          ComputeHardwareJointVelocity(T_des_base, v_des);
      const double vd_scale = hardware_mode_ ? 1.0 : ctc_vd_scale_;
      v_d = vd_scale * dynamics_->PackV(v_d_arr);
      const Eigen::VectorXd a_d = Eigen::VectorXd::Zero(dynamics_->model().nv);
      tau = dynamics_->ComputeTrackingTorque(
          q, v, q_d, v_d, a_d, kp_joint_, kd_joint_);
    }

    const pinocchio::SE3 T_ee = dynamics_->ComputeEePoseInBase(q);
    osc_result.task_error.head<3>() =
        T_des_base.translation() - T_ee.translation();
    osc_result.task_error.tail<3>() = pinocchio::log3(
        T_ee.rotation().transpose() * T_des_base.rotation());
    osc_result.position_error = osc_result.task_error.head<3>().norm();
    osc_result.orientation_error = osc_result.task_error.tail<3>().norm();
    q_plan = q_des;
    q_cmd = q_des_filtered_;
  } else {
    tau = dynamics_->ComputeOperationalTorque(
        q, v, T_des_base, v_des, osc_gains_, &osc_result);
    ik_solver_->set_current_q(q_);
    auto ik_ref = ik_solver_->SolveSe3(T_des_base, ik_cycle_iters_);
    if (!ik_ref.acceptable) {
      ik_ref = ik_solver_->RefineSe3(T_des_base, ik_refine_iters_);
    }
    q_plan = ik_solver_->current_q();
    q_cmd = q_plan;
  }

  if (!kinematic_stabilization_ && !hardware_mode_) {
    for (int i = 0; i < tau.size(); ++i) {
      tau[i] = Clamp(tau[i], -torque_limit_, torque_limit_);
    }

    const Eigen::VectorXd qdd = dynamics_->ForwardDynamics(q, v, tau);
    std::array<double, PinocchioDynamicsModel::kDof> qdd_arr{};
    dynamics_->UnpackV(qdd, qdd_arr);
    for (size_t i = 0; i < PinocchioDynamicsModel::kDof; ++i) {
      v_[i] += qdd_arr[i] * dt;
      v_[i] = Clamp(v_[i], -10.0, 10.0);
      q_[i] += v_[i] * dt;
      q_[i] = Clamp(q_[i], -3.0, 3.0);
    }
  }

  ++control_step_;

  if (hardware_mode_) {
    std::array<double, PinocchioDynamicsModel::kDof> q_cmd_hw = q_des_filtered_;
    if (!use_ik_joint_control_ || use_mode_d_) {
      q_cmd_hw = q_;
    }
    std::array<double, PinocchioDynamicsModel::kDof> dq_cmd =
        use_mode_d_ ? mode_d_dq_ref_
                    : ComputeHardwareJointVelocity(T_des_base, v_des);

    Eigen::VectorXd tau_hw = Eigen::VectorXd::Zero(PinocchioDynamicsModel::kDof);
    if (use_torque_feedforward_) {
      for (int i = 0; i < tau.size() && i < static_cast<int>(PinocchioDynamicsModel::kDof); ++i) {
        tau_hw[i] = Clamp(tau[i], -hw_torque_limit_, hw_torque_limit_);
      }
    }
    PublishHardwareCommand(q_cmd_hw, dq_cmd, tau_hw, stamp);
  } else {
    PublishJointStates(mount_world, stamp);
    PublishReferenceJoints(q_plan, q_cmd, stamp);
  }
  PublishMarkers(mount_world, stamp);
  PublishMetrics(osc_result, mount_world, stamp);
}

void EeStabilizationNode::PublishJointStates(
    const BaseDisturbanceGenerator::MountPose& mount,
    const rclcpp::Time& stamp) {
  const auto mount_vals = MountToJointValues(mount);

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = stamp;
  msg.name.insert(msg.name.end(), mount_joint_names_.begin(), mount_joint_names_.end());
  msg.name.insert(msg.name.end(), arm_joint_names_.begin(), arm_joint_names_.end());

  for (double val : mount_vals) {
    msg.position.push_back(val);
  }
  msg.position.insert(msg.position.end(), q_.begin(), q_.end());
  msg.velocity.assign(msg.name.size(), 0.0);
  msg.velocity[0] = mount.linear_velocity.x();
  msg.velocity[1] = mount.linear_velocity.y();
  msg.velocity[2] = mount.linear_velocity.z();
  msg.velocity[3] = mount.angular_velocity.x();
  msg.velocity[4] = mount.angular_velocity.y();
  msg.velocity[5] = mount.angular_velocity.z();
  for (size_t i = mount_joint_names_.size(); i < msg.name.size(); ++i) {
    msg.velocity[i] = v_[i - mount_joint_names_.size()];
  }

  joint_state_pub_->publish(msg);
}

void EeStabilizationNode::PublishReferenceJoints(
    const std::array<double, PinocchioDynamicsModel::kDof>& q_plan,
    const std::array<double, PinocchioDynamicsModel::kDof>& q_cmd,
    const rclcpp::Time& stamp) {
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = stamp;
  msg.name.assign(arm_joint_names_.begin(), arm_joint_names_.end());
  msg.position.assign(q_plan.begin(), q_plan.end());
  msg.velocity.assign(q_cmd.begin(), q_cmd.end());
  reference_pub_->publish(msg);
}

void EeStabilizationNode::PublishMarkers(
    const BaseDisturbanceGenerator::MountPose& mount,
    const rclcpp::Time& stamp) {
  const auto lifetime = rclcpp::Duration::from_seconds(0.5);
  const auto actual_ee_pose = Se3ToPose(ComputeEeInWorld(mount));
  const auto target_pose = Se3ToPose(ee_target_world_);

  visualization_msgs::msg::Marker sphere_boundary;
  sphere_boundary.header.frame_id = "world";
  sphere_boundary.header.stamp = stamp;
  sphere_boundary.ns = "stabilization";
  sphere_boundary.id = 0;
  sphere_boundary.type = visualization_msgs::msg::Marker::SPHERE;
  sphere_boundary.action = visualization_msgs::msg::Marker::ADD;
  sphere_boundary.pose.position.x = disturbance_->sphere_center().x();
  sphere_boundary.pose.position.y = disturbance_->sphere_center().y();
  sphere_boundary.pose.position.z = disturbance_->sphere_center().z();
  sphere_boundary.pose.orientation.w = 1.0;
  const double d = 2.0 * disturbance_->sphere_radius();
  sphere_boundary.scale.x = d;
  sphere_boundary.scale.y = d;
  sphere_boundary.scale.z = d;
  sphere_boundary.color.r = 0.9f;
  sphere_boundary.color.g = 0.9f;
  sphere_boundary.color.b = 0.2f;
  sphere_boundary.color.a = 0.12f;
  sphere_boundary.lifetime = lifetime;

  visualization_msgs::msg::Marker target_ee;
  target_ee.header.frame_id = "world";
  target_ee.header.stamp = stamp;
  target_ee.ns = "stabilization";
  target_ee.id = 1;
  target_ee.type = visualization_msgs::msg::Marker::SPHERE;
  target_ee.action = visualization_msgs::msg::Marker::ADD;
  target_ee.pose = target_pose;
  target_ee.scale.x = 0.02;
  target_ee.scale.y = 0.02;
  target_ee.scale.z = 0.02;
  target_ee.color.r = 0.1f;
  target_ee.color.g = 0.95f;
  target_ee.color.b = 0.25f;
  target_ee.color.a = 0.55f;
  target_ee.lifetime = lifetime;

  visualization_msgs::msg::Marker fixed_axes;
  fixed_axes.header.frame_id = "world";
  fixed_axes.header.stamp = stamp;
  fixed_axes.ns = "stabilization";
  fixed_axes.id = 8;
  fixed_axes.type = visualization_msgs::msg::Marker::LINE_LIST;
  fixed_axes.action = visualization_msgs::msg::Marker::ADD;
  fixed_axes.pose = target_pose;
  fixed_axes.scale.x = 0.003;
  fixed_axes.color.r = 0.15f;
  fixed_axes.color.g = 0.95f;
  fixed_axes.color.b = 0.35f;
  fixed_axes.color.a = 0.95f;
  fixed_axes.lifetime = lifetime;
  const Eigen::Vector3d origin = ee_target_world_.translation();
  const Eigen::Matrix3d Rfix = ee_target_world_.rotation();
  const double axis_len = 0.06;
  const std::array<Eigen::Vector3d, 3> dirs = {
      Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ()};
  for (const auto& dir : dirs) {
    geometry_msgs::msg::Point a;
    a.x = origin.x();
    a.y = origin.y();
    a.z = origin.z();
    const Eigen::Vector3d tip = origin + Rfix * (dir * axis_len);
    geometry_msgs::msg::Point bpt;
    bpt.x = tip.x();
    bpt.y = tip.y();
    bpt.z = tip.z();
    fixed_axes.points.push_back(a);
    fixed_axes.points.push_back(bpt);
  }

  visualization_msgs::msg::Marker actual_marker;
  actual_marker.header.frame_id = "world";
  actual_marker.header.stamp = stamp;
  actual_marker.ns = "stabilization";
  actual_marker.id = 2;
  actual_marker.type = visualization_msgs::msg::Marker::SPHERE;
  actual_marker.action = visualization_msgs::msg::Marker::ADD;
  actual_marker.pose = actual_ee_pose;
  actual_marker.scale.x = 0.015;
  actual_marker.scale.y = 0.015;
  actual_marker.scale.z = 0.015;
  actual_marker.color.r = 1.0f;
  actual_marker.color.g = 0.45f;
  actual_marker.color.b = 0.05f;
  actual_marker.color.a = 1.0f;
  actual_marker.lifetime = lifetime;

  visualization_msgs::msg::Marker mount_pt;
  mount_pt.header.frame_id = "world";
  mount_pt.header.stamp = stamp;
  mount_pt.ns = "stabilization";
  mount_pt.id = 3;
  mount_pt.type = visualization_msgs::msg::Marker::SPHERE;
  mount_pt.action = visualization_msgs::msg::Marker::ADD;
  const auto base_pose = Se3ToPose(MountToWorldBase(mount));
  mount_pt.pose = base_pose;
  mount_pt.scale.x = 0.09;
  mount_pt.scale.y = 0.09;
  mount_pt.scale.z = 0.09;
  mount_pt.color.r = 0.95f;
  mount_pt.color.g = 0.35f;
  mount_pt.color.b = 0.1f;
  mount_pt.color.a = 0.9f;
  mount_pt.lifetime = lifetime;

  visualization_msgs::msg::Marker err_line;
  err_line.header.frame_id = "world";
  err_line.header.stamp = stamp;
  err_line.ns = "stabilization";
  err_line.id = 4;
  err_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  err_line.action = visualization_msgs::msg::Marker::ADD;
  err_line.pose.orientation.w = 1.0;
  err_line.scale.x = 0.004;
  err_line.color.r = 1.0f;
  err_line.color.g = 0.2f;
  err_line.color.b = 0.2f;
  err_line.color.a = 0.8f;
  err_line.lifetime = lifetime;
  geometry_msgs::msg::Point p0;
  p0.x = target_pose.position.x;
  p0.y = target_pose.position.y;
  p0.z = target_pose.position.z;
  geometry_msgs::msg::Point p1;
  p1.x = actual_ee_pose.position.x;
  p1.y = actual_ee_pose.position.y;
  p1.z = actual_ee_pose.position.z;
  err_line.points = {p0, p1};

  base_trail_.push_back(Eigen::Vector3d(
      base_pose.position.x, base_pose.position.y, base_pose.position.z));
  while (base_trail_.size() > kTrailLength) {
    base_trail_.pop_front();
  }

  visualization_msgs::msg::Marker base_trail;
  base_trail.header.frame_id = "world";
  base_trail.header.stamp = stamp;
  base_trail.ns = "stabilization";
  base_trail.id = 7;
  base_trail.type = visualization_msgs::msg::Marker::LINE_STRIP;
  base_trail.action = visualization_msgs::msg::Marker::ADD;
  base_trail.pose.orientation.w = 1.0;
  base_trail.scale.x = 0.006;
  base_trail.color.r = 0.95f;
  base_trail.color.g = 0.55f;
  base_trail.color.b = 0.1f;
  base_trail.color.a = 0.75f;
  base_trail.lifetime = lifetime;
  for (const auto& pt : base_trail_) {
    geometry_msgs::msg::Point p;
    p.x = pt.x();
    p.y = pt.y();
    p.z = pt.z();
    base_trail.points.push_back(p);
  }

  const auto label_root = MakeTextLabel(
      5, arm_root_name_ + "·动", base_pose, 0.95f, 0.55f, 0.15f, stamp, lifetime, 0.07);
  const auto label_ee = MakeTextLabel(
      6, arm_ee_name_ + "·固定", target_pose, 0.15f, 0.95f, 0.35f, stamp, lifetime, 0.07);

  visualization_msgs::msg::MarkerArray array;
  array.markers = {
      sphere_boundary, target_ee, fixed_axes, actual_marker, mount_pt, base_trail,
      err_line, label_root, label_ee};
  marker_array_pub_->publish(array);
}

void EeStabilizationNode::PublishMetrics(
    const PinocchioDynamicsModel::OperationalResult& result,
    const BaseDisturbanceGenerator::MountPose& mount,
    const rclcpp::Time& stamp) {
  (void)stamp;
  const pinocchio::SE3 T_ee_world = ComputeEeInWorld(mount);
  const Eigen::Vector3d world_pos_err =
      ee_target_world_.translation() - T_ee_world.translation();
  const double world_pos_err_norm = world_pos_err.norm();
  const double world_orient_err = pinocchio::log3(
      T_ee_world.rotation().transpose() * ee_target_world_.rotation()).norm();

  std_msgs::msg::Float64MultiArray msg;
  msg.data = {
      world_pos_err_norm,
      world_orient_err,
      result.position_error,
      result.orientation_error,
      last_solve_time_us_,
  };
  error_pub_->publish(msg);

  if (control_step_ % static_cast<uint64_t>(control_rate_) == 0) {
    RCLCPP_INFO(
        get_logger(),
        "World EE err pos=%.4f m orient=%.4f rad | mount=[%.2f, %.2f, %.2f]",
        world_pos_err_norm,
        world_orient_err,
        mount.position.x(),
        mount.position.y(),
        mount.position.z());
  }
}

void EeStabilizationNode::MasterJointCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  for (size_t i = 0; i < PinocchioDynamicsModel::kDof; ++i) {
    const std::string& name = arm_joint_names_[i];
    auto it = std::find(msg->name.begin(), msg->name.end(), name);
    if (it == msg->name.end()) {
      continue;
    }
    const size_t idx = static_cast<size_t>(std::distance(msg->name.begin(), it));
    if (idx < msg->position.size()) {
      master_q_[i] = msg->position[idx];
    }
  }
  master_ready_ = true;
}

pinocchio::SE3 EeStabilizationNode::ComputeMasterEeInWorld() const {
  const pinocchio::SE3 T_base_ee =
      master_fk_->ComputeEePoseInBase(master_fk_->PackQ(master_q_));
  return T_base_ee;
}

bool EeStabilizationNode::UpdateTeleopTarget(double dt) {
  if (!master_ready_ || !master_fk_) {
    return false;
  }

  const pinocchio::SE3 target_raw = ComputeMasterEeInWorld();
  if (!ref_init_) {
    ee_target_world_ = target_raw;
    ref_init_ = true;
    RCLCPP_INFO(
        get_logger(),
        "Teleop EE target initialized: [%.3f, %.3f, %.3f]",
        ee_target_world_.translation().x(),
        ee_target_world_.translation().y(),
        ee_target_world_.translation().z());
    return true;
  }

  const double alpha = std::clamp(teleop_target_filter_, 0.0, 1.0);
  const double beta = 1.0 - alpha;
  const Eigen::Vector3d p =
      alpha * ee_target_world_.translation() + beta * target_raw.translation();
  const Eigen::Quaterniond q_raw(target_raw.rotation());
  const Eigen::Quaterniond q_prev(ee_target_world_.rotation());
  const Eigen::Quaterniond q = q_prev.slerp(beta, q_raw).normalized();
  ee_target_world_ = pinocchio::SE3(q.toRotationMatrix(), p);
  (void)dt;
  return true;
}

void EeStabilizationNode::JointFeedbackCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  for (size_t i = 0; i < PinocchioDynamicsModel::kDof; ++i) {
    const std::string name = arm_joint_names_[i];
    auto it = std::find(msg->name.begin(), msg->name.end(), name);
    if (it == msg->name.end()) {
      continue;
    }
    const size_t idx = static_cast<size_t>(std::distance(msg->name.begin(), it));
    if (idx < msg->position.size()) {
      q_[i] = msg->position[idx];
    }
    if (idx < msg->velocity.size()) {
      v_[i] = msg->velocity[idx];
    }
  }
  feedback_ready_ = true;
  ik_solver_->set_current_q(q_);
}

void EeStabilizationNode::MountDisturbanceCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (msg->data.size() < 6) {
    return;
  }
  external_mount_.position =
      Eigen::Vector3d(msg->data[0], msg->data[1], msg->data[2]);
  external_mount_.rpy = Eigen::Vector3d(msg->data[3], msg->data[4], msg->data[5]);
  const Eigen::AngleAxisd roll(external_mount_.rpy.x(), Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch(external_mount_.rpy.y(), Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw(external_mount_.rpy.z(), Eigen::Vector3d::UnitZ());
  external_mount_.orientation = (yaw * pitch * roll).normalized();
  external_mount_ready_ = true;
}

BaseDisturbanceGenerator::MountPose EeStabilizationNode::GetMountPose(double dt) {
  if (base_source_ == "external") {
    BaseDisturbanceGenerator::MountPose mount;
    if (!external_mount_ready_) {
      mount.position = Eigen::Vector3d::Zero();
      mount.rpy = Eigen::Vector3d::Zero();
      mount.orientation = Eigen::Quaterniond::Identity();
      return mount;
    }
    mount = external_mount_;
    if (dt > 1e-6) {
      mount.linear_velocity =
          (mount.position - prev_external_mount_.position) / dt;
      mount.angular_velocity = (mount.rpy - prev_external_mount_.rpy) / dt;
    }
    prev_external_mount_ = external_mount_;
    return mount;
  }
  if (base_source_ == "simulated") {
    return disturbance_->Step(dt);
  }
  if (base_source_ == "tf") {
    return GetMountFromTf(dt);
  }
  BaseDisturbanceGenerator::MountPose mount;
  mount.position = Eigen::Vector3d::Zero();
  mount.rpy = Eigen::Vector3d::Zero();
  mount.orientation = Eigen::Quaterniond::Identity();
  return mount;
}

bool EeStabilizationNode::LookupBaseTransform(
    Eigen::Vector3d* position, Eigen::Quaterniond* orientation) {
  if (!tf_buffer_) {
    return false;
  }
  try {
    const auto tf = tf_buffer_->lookupTransform(
        world_frame_, base_frame_name_, tf2::TimePointZero);
    position->x() = tf.transform.translation.x;
    position->y() = tf.transform.translation.y;
    position->z() = tf.transform.translation.z;
    orientation->x() = tf.transform.rotation.x;
    orientation->y() = tf.transform.rotation.y;
    orientation->z() = tf.transform.rotation.z;
    orientation->w() = tf.transform.rotation.w;
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "TF %s -> %s unavailable: %s",
        world_frame_.c_str(), base_frame_name_.c_str(), ex.what());
    return false;
  }
}

BaseDisturbanceGenerator::MountPose EeStabilizationNode::GetMountFromTf(double dt) {
  BaseDisturbanceGenerator::MountPose mount;
  Eigen::Vector3d base_pos = Eigen::Vector3d::Zero();
  Eigen::Quaterniond base_quat = Eigen::Quaterniond::Identity();
  if (!LookupBaseTransform(&base_pos, &base_quat)) {
    if (last_tf_position_.has_value()) {
      mount.position = *last_tf_position_;
      mount.orientation = *last_tf_orientation_;
      mount.rpy = QuatToRpy(mount.orientation);
      mount.linear_velocity = tf_lin_vel_filt_;
      mount.angular_velocity = tf_ang_vel_filt_;
    }
    return mount;
  }

  const pinocchio::SE3 T_world_base(base_quat.toRotationMatrix(), base_pos);
  const pinocchio::SE3 T_base_drone(
      Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, 0.0, -mount_base_offset_z_));
  const pinocchio::SE3 T_world_drone = T_world_base * T_base_drone;

  mount.position = T_world_drone.translation();
  mount.rpy = QuatToRpy(Eigen::Quaterniond(T_world_drone.rotation()));
  mount.orientation = Eigen::Quaterniond(T_world_drone.rotation());

  if (last_tf_position_.has_value() && dt > 1e-6) {
    const Eigen::Vector3d lin_vel_raw =
        (mount.position - *last_tf_position_) / dt;
    const Eigen::Matrix3d dR =
        T_world_drone.rotation() * last_tf_orientation_->toRotationMatrix().transpose();
    const Eigen::Vector3d ang_vel_raw = pinocchio::log3(dR) / dt;

    if (!tf_vel_filt_init_) {
      tf_lin_vel_filt_ = lin_vel_raw;
      tf_ang_vel_filt_ = ang_vel_raw;
      tf_vel_filt_init_ = true;
    } else {
      const double a = tf_velocity_filter_alpha_;
      tf_lin_vel_filt_ = a * tf_lin_vel_filt_ + (1.0 - a) * lin_vel_raw;
      tf_ang_vel_filt_ = a * tf_ang_vel_filt_ + (1.0 - a) * ang_vel_raw;
    }
    mount.linear_velocity = tf_lin_vel_filt_;
    mount.angular_velocity = tf_ang_vel_filt_;
  }

  last_tf_position_ = mount.position;
  last_tf_orientation_ = mount.orientation;
  last_tf_stamp_ = now();
  return mount;
}

void EeStabilizationNode::PublishHardwareCommand(
    const std::array<double, PinocchioDynamicsModel::kDof>& q_cmd,
    const std::array<double, PinocchioDynamicsModel::kDof>& dq_cmd,
    const Eigen::VectorXd& tau,
    const rclcpp::Time& stamp) {
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = stamp;
  msg.name = hardware_joint_names_;
  msg.position.assign(hardware_dof_, 0.0);
  msg.velocity.assign(hardware_dof_, 0.0);
  msg.effort.assign(hardware_dof_, 0.0);

  for (size_t i = 0; i < PinocchioDynamicsModel::kDof; ++i) {
    msg.position[i] = q_cmd[i];
    msg.velocity[i] = dq_cmd[i];
    if (i < static_cast<size_t>(tau.size())) {
      msg.effort[i] = tau[static_cast<Eigen::Index>(i)];
    }
  }
  if (hardware_dof_ >= 7) {
    msg.position[6] = joint7_value_;
  }
  student_cmd_pub_->publish(msg);
}
