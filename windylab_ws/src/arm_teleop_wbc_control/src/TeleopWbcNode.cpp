#include "TeleopWbcNode.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/spatial/se3.hpp>

namespace {
std::array<double, PinocchioArmModel::kDof> DefaultHome() {
  return {0.0, 0.35, -0.55, 0.0, 0.45, 0.0};
}

double Clamp(double value, double min_value, double max_value) {
  return std::max(min_value, std::min(max_value, value));
}

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
  next.rpy = mount.rpy;
  next.linear_velocity = mount.linear_velocity;
  next.angular_velocity = mount.angular_velocity;
  return next;
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
  marker.ns = "teleop_wbc";
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
double JointDistanceToRef(
    const std::array<double, PinocchioArmModel::kDof>& q,
    const std::array<double, PinocchioArmModel::kDof>& q_ref) {
  double dist = 0.0;
  for (size_t i = 0; i < PinocchioArmModel::kDof; ++i) {
    const double delta =
        VelocityWbcController::UnwrapNear(q[i], q_ref[i]) - q_ref[i];
    dist += delta * delta;
  }
  return dist;
}

double WristPoseCost(
    const PinocchioArmModel& arm,
    const pinocchio::SE3& target,
    const std::array<double, PinocchioArmModel::kDof>& q) {
  const pinocchio::SE3 pose = arm.ComputeEePoseInBase(arm.PackQ(q));
  const double pos_err = (pose.translation() - target.translation()).norm();
  const double orient_err =
      pinocchio::log3(pose.rotation().transpose() * target.rotation()).norm();
  return pos_err + orient_err;
}
}  // namespace

std::array<double, PinocchioArmModel::kDof> TeleopWbcNode::SelectContinuousWristBranch(
    const pinocchio::SE3& target,
    const std::array<double, PinocchioArmModel::kDof>& q_candidate,
    const std::array<double, PinocchioArmModel::kDof>& q_ref) const {
  constexpr size_t kJoint4 = 3;
  constexpr size_t kJoint6 = 5;
  const double ref_cost = WristPoseCost(*arm_, target, q_candidate);

  std::array<double, PinocchioArmModel::kDof> best = q_candidate;
  double best_dist = JointDistanceToRef(q_candidate, q_ref);

  for (const double sign : {1.0, -1.0}) {
    std::array<double, PinocchioArmModel::kDof> alt = q_candidate;
    alt[kJoint4] = VelocityWbcController::UnwrapNear(
        alt[kJoint4] + sign * M_PI, q_ref[kJoint4]);
    alt[kJoint6] = VelocityWbcController::UnwrapNear(
        alt[kJoint6] - sign * M_PI, q_ref[kJoint6]);
    const double alt_cost = WristPoseCost(*arm_, target, alt);
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

TeleopWbcNode::TeleopWbcNode() : Node("teleop_wbc") {
  mount_joint_names_ = {
      "mount_tx", "mount_ty", "mount_tz", "mount_rx", "mount_ry", "mount_rz"};
  arm_joint_names_ = {
      "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

  declare_parameter<std::string>("urdf_path", "");
  declare_parameter<std::string>("base_frame", "base_link");
  declare_parameter<std::string>("ee_frame", "link6");
  declare_parameter<double>("control_rate", 500.0);
  declare_parameter<std::string>("base_source", "static");
  declare_parameter<std::string>("mount_disturbance_topic", "/mount_disturbance/pose");
  declare_parameter<std::vector<double>>("mount_anchor", {0.0, 0.0, 0.0});
  declare_parameter<std::vector<double>>("q_home", std::vector<double>{});
  declare_parameter<double>("mount_base_offset_z", 0.02);
  declare_parameter<std::string>("arm_root_name", "机载端");
  declare_parameter<std::string>("arm_ee_name", "末端");
  declare_parameter<double>("disturbance_radius", 0.12);
  declare_parameter<double>("disturbance_orient_amp", 0.18);
  declare_parameter<double>("disturbance_time_constant", 1.0);
  declare_parameter<double>("disturbance_amplitude_scale", 0.92);
  declare_parameter<std::vector<double>>("sphere_center", {0.0, 0.0, 0.0});
  declare_parameter<std::string>("master_joint_topic", "/master/joint_states");
  declare_parameter<std::string>("master_urdf_path", "");
  declare_parameter<double>("teleop_target_filter", 0.35);
  declare_parameter<std::string>("teleop_control_mode", "ee_wbc");
  declare_parameter<int>("wbc_substeps", 8);
  declare_parameter<std::vector<double>>(
      "wbc_task_kp", {24.0, 24.0, 24.0, 18.0, 18.0, 18.0});
  declare_parameter<std::vector<double>>(
      "wbc_task_ki", {6.0, 6.0, 6.0, 4.0, 4.0, 4.0});
  declare_parameter<std::vector<double>>(
      "wbc_task_weight", {1.0, 1.0, 1.0, 0.8, 0.8, 0.8});
  declare_parameter<std::vector<double>>(
      "wbc_integral_limit", {0.4, 0.4, 0.4, 0.25, 0.25, 0.25});
  declare_parameter<double>("wbc_nullspace_weight", 0.2);
  declare_parameter<double>("wbc_nullspace_rate", 2.5);
  declare_parameter<double>("wbc_clik_damping", 0.035);
  declare_parameter<double>("wbc_regularization", 0.02);
  declare_parameter<double>("max_joint_velocity", 4.0);

  const auto urdf_path = get_parameter("urdf_path").as_string();
  const auto base_frame = get_parameter("base_frame").as_string();
  const auto ee_frame = get_parameter("ee_frame").as_string();
  control_rate_ = get_parameter("control_rate").as_double();
  base_source_ = get_parameter("base_source").as_string();
  mount_disturbance_topic_ = get_parameter("mount_disturbance_topic").as_string();
  mount_base_offset_z_ = get_parameter("mount_base_offset_z").as_double();
  arm_root_name_ = get_parameter("arm_root_name").as_string();
  arm_ee_name_ = get_parameter("arm_ee_name").as_string();
  master_joint_topic_ = get_parameter("master_joint_topic").as_string();
  teleop_target_filter_ = get_parameter("teleop_target_filter").as_double();
  teleop_control_mode_ = get_parameter("teleop_control_mode").as_string();
  if (teleop_control_mode_ == "ee_stabilization") {
    teleop_control_mode_ = "ee_wbc";
  }
  wbc_substeps_ = std::max(1, static_cast<int>(get_parameter("wbc_substeps").as_int()));

  std::array<double, PinocchioArmModel::kDof> q_home = DefaultHome();
  const auto q_home_vec = get_parameter("q_home").as_double_array();
  if (q_home_vec.size() == PinocchioArmModel::kDof) {
    for (size_t i = 0; i < PinocchioArmModel::kDof; ++i) {
      q_home[i] = q_home_vec[i];
    }
  }
  q_ = q_home;
  q_des_filtered_ = q_home;
  v_.fill(0.0);

  const auto anchor_vec = get_parameter("mount_anchor").as_double_array();
  if (anchor_vec.size() == 3) {
    mount_anchor_pos_ =
        Eigen::Vector3d(anchor_vec[0], anchor_vec[1], anchor_vec[2]);
  }

  arm_ = std::make_unique<PinocchioArmModel>(
      urdf_path, arm_joint_names_, base_frame, ee_frame);

  const auto kp_vec = get_parameter("wbc_task_kp").as_double_array();
  const auto ki_vec = get_parameter("wbc_task_ki").as_double_array();
  const auto weight_vec = get_parameter("wbc_task_weight").as_double_array();
  const auto int_lim_vec = get_parameter("wbc_integral_limit").as_double_array();
  for (size_t i = 0; i < 6; ++i) {
    wbc_params_.kp(static_cast<Eigen::Index>(i)) = kp_vec[i];
    wbc_params_.ki(static_cast<Eigen::Index>(i)) = ki_vec[i];
    wbc_params_.task_weight(static_cast<Eigen::Index>(i)) = weight_vec[i];
    wbc_params_.integral_limit(static_cast<Eigen::Index>(i)) = int_lim_vec[i];
  }
  wbc_params_.nullspace_weight = get_parameter("wbc_nullspace_weight").as_double();
  wbc_params_.nullspace_rate = get_parameter("wbc_nullspace_rate").as_double();
  wbc_params_.clik_damping = get_parameter("wbc_clik_damping").as_double();
  wbc_params_.regularization = get_parameter("wbc_regularization").as_double();
  wbc_params_.max_joint_velocity = get_parameter("max_joint_velocity").as_double();

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

  const auto master_urdf = get_parameter("master_urdf_path").as_string();
  const std::string master_model_path =
      master_urdf.empty() ? urdf_path : master_urdf;
  master_fk_ = std::make_unique<PinocchioArmModel>(
      master_model_path, arm_joint_names_, base_frame, ee_frame);

  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  if (base_source_ == "external") {
    mount_disturbance_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        mount_disturbance_topic_, rclcpp::SensorDataQoS(),
        std::bind(
            &TeleopWbcNode::MountDisturbanceCallback, this, std::placeholders::_1));
  }
  master_joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      master_joint_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TeleopWbcNode::MasterJointCallback, this, std::placeholders::_1));
  marker_array_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/teleop_wbc_markers", rclcpp::QoS(1).transient_local());
  error_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/teleop_wbc_error", 10);
  reference_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "/teleop_wbc_reference", 10);

  const auto period = std::chrono::duration<double>(1.0 / control_rate_);
  control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TeleopWbcNode::ControlLoop, this));

  RCLCPP_INFO(
      get_logger(),
      "QP-WBC teleop | base=%s | rate=%.0f Hz | mode=%s | substeps=%d",
      base_source_.c_str(),
      control_rate_,
      teleop_control_mode_.c_str(),
      wbc_substeps_);
  RCLCPP_INFO(
      get_logger(),
      "Arm endpoints: [%s]=%s, [%s]=%s",
      arm_root_name_.c_str(),
      base_frame.c_str(),
      arm_ee_name_.c_str(),
      ee_frame.c_str());
}

pinocchio::SE3 TeleopWbcNode::MountToWorldDrone(
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

pinocchio::SE3 TeleopWbcNode::MountToWorldBase(
    const BaseDisturbanceGenerator::MountPose& mount) const {
  const pinocchio::SE3 T_mount_to_base(
      Eigen::Matrix3d::Identity(),
      Eigen::Vector3d(0.0, 0.0, mount_base_offset_z_));
  return MountToWorldDrone(mount) * T_mount_to_base;
}

pinocchio::SE3 TeleopWbcNode::ComputeTargetInBase(
    const BaseDisturbanceGenerator::MountPose& mount) const {
  return MountToWorldBase(mount).actInv(ee_target_world_);
}

BaseDisturbanceGenerator::MountPose TeleopWbcNode::ApplyMountAnchor(
    const BaseDisturbanceGenerator::MountPose& mount) const {
  BaseDisturbanceGenerator::MountPose world = mount;
  world.position = mount_anchor_pos_ + mount.position;
  world.rpy = mount.rpy;
  world.orientation = mount.orientation;
  return world;
}

pinocchio::SE3 TeleopWbcNode::ComputeEeInWorld(
    const BaseDisturbanceGenerator::MountPose& mount) const {
  const pinocchio::SE3 T_base_ee = arm_->ComputeEePoseInBase(arm_->PackQ(q_));
  return MountToWorldBase(mount) * T_base_ee;
}

std::array<double, 6> TeleopWbcNode::MountToJointValues(
    const BaseDisturbanceGenerator::MountPose& mount) const {
  return {
      mount.position.x(),
      mount.position.y(),
      mount.position.z(),
      mount.rpy.x(),
      mount.rpy.y(),
      mount.rpy.z()};
}

Eigen::Matrix<double, 6, 1> TeleopWbcNode::ComputeFeedforwardTaskVelocity(
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

TeleopWbcNode::TaskMetrics TeleopWbcNode::ComputeTaskMetrics(
    const pinocchio::SE3& T_des_base) const {
  TaskMetrics metrics;
  const pinocchio::SE3 T_ee = arm_->ComputeEePoseInBase(arm_->PackQ(q_));
  metrics.task_error.head<3>() = T_des_base.translation() - T_ee.translation();
  metrics.task_error.tail<3>() = pinocchio::log3(
      T_ee.rotation().transpose() * T_des_base.rotation());
  metrics.position_error = metrics.task_error.head<3>().norm();
  metrics.orientation_error = metrics.task_error.tail<3>().norm();
  return metrics;
}

void TeleopWbcNode::RunJointMirrorMode(
    const pinocchio::SE3& T_des_base,
    std::array<double, PinocchioArmModel::kDof>* q_plan,
    std::array<double, PinocchioArmModel::kDof>* q_cmd,
    TaskMetrics* metrics) {
  std::array<double, PinocchioArmModel::kDof> q_des{};
  for (size_t i = 0; i < PinocchioArmModel::kDof; ++i) {
    q_des[i] = VelocityWbcController::UnwrapNear(master_q_[i], q_[i]);
  }
  q_des_filtered_ = q_des;
  q_ = q_des_filtered_;
  v_.fill(0.0);
  *q_plan = q_des;
  *q_cmd = q_des_filtered_;
  *metrics = ComputeTaskMetrics(T_des_base);
}

void TeleopWbcNode::RunEeWbcMode(
    const pinocchio::SE3& T_des_base,
    const Eigen::Matrix<double, 6, 1>& v_feedforward,
    const double dt,
    std::array<double, PinocchioArmModel::kDof>* q_plan,
    std::array<double, PinocchioArmModel::kDof>* q_cmd,
    TaskMetrics* metrics) {
  std::array<double, PinocchioArmModel::kDof> q_ref{};
  for (size_t i = 0; i < PinocchioArmModel::kDof; ++i) {
    q_ref[i] = VelocityWbcController::UnwrapNear(master_q_[i], q_[i]);
  }

  const double sub_dt = dt / static_cast<double>(wbc_substeps_);
  const auto solve_t0 = std::chrono::steady_clock::now();
  for (int sub = 0; sub < wbc_substeps_; ++sub) {
    const TaskMetrics sub_metrics = ComputeTaskMetrics(T_des_base);
    const Eigen::Matrix<double, 6, 1> v_cmd = wbc_.ComputeTaskCommand(
        sub_metrics.task_error,
        v_feedforward,
        wbc_params_,
        &wbc_state_,
        dt,
        sub == 0);
    const Eigen::VectorXd q_sub = arm_->PackQ(q_);
    const Eigen::Matrix<double, 6, 6> J = arm_->ComputeArmJacobian(q_sub);
    const auto q_dot = wbc_.SolveJointVelocity(
        J, v_cmd, q_, q_ref, wbc_params_, &wbc_state_);
    for (size_t i = 0; i < PinocchioArmModel::kDof; ++i) {
      q_[i] += q_dot[i] * sub_dt;
    }
  }

  q_ = SelectContinuousWristBranch(T_des_base, q_, q_ref);
  last_solve_time_us_ = static_cast<double>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - solve_t0)
          .count());
  q_des_filtered_ = q_;
  v_.fill(0.0);
  *q_plan = q_;
  *q_cmd = q_des_filtered_;
  *metrics = ComputeTaskMetrics(T_des_base);
}

void TeleopWbcNode::ControlLoop() {
  if (!master_ready_) {
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

  UpdateTeleopTarget(dt);

  const pinocchio::SE3 T_des_base = ComputeTargetInBase(mount_world);
  Eigen::Matrix<double, 6, 1> v_feedforward;
  if (teleop_vel_init_) {
    const pinocchio::SE3 delta = T_des_base * prev_T_des_base_.inverse();
    v_feedforward = pinocchio::log6(delta) / std::max(dt, 1e-6);
    for (int i = 0; i < 6; ++i) {
      v_feedforward[i] = Clamp(v_feedforward[i], -15.0, 15.0);
    }
  } else {
    v_feedforward = ComputeFeedforwardTaskVelocity(mount_world, dt);
  }
  prev_T_des_base_ = T_des_base;
  teleop_vel_init_ = true;

  std::array<double, PinocchioArmModel::kDof> q_plan{};
  std::array<double, PinocchioArmModel::kDof> q_cmd{};
  TaskMetrics metrics;

  const bool joint_mirror = teleop_control_mode_ == "joint_mirror";
  if (joint_mirror) {
    RunJointMirrorMode(T_des_base, &q_plan, &q_cmd, &metrics);
  } else {
    RunEeWbcMode(T_des_base, v_feedforward, dt, &q_plan, &q_cmd, &metrics);
  }

  ++control_step_;
  PublishJointStates(mount_world, stamp);
  PublishReferenceJoints(q_plan, q_cmd, stamp);
  PublishMarkers(mount_world, stamp);
  PublishMetrics(metrics, mount_world, stamp);
}

void TeleopWbcNode::PublishJointStates(
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

void TeleopWbcNode::PublishReferenceJoints(
    const std::array<double, PinocchioArmModel::kDof>& q_plan,
    const std::array<double, PinocchioArmModel::kDof>& q_cmd,
    const rclcpp::Time& stamp) {
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = stamp;
  msg.name.assign(arm_joint_names_.begin(), arm_joint_names_.end());
  msg.position.assign(q_plan.begin(), q_plan.end());
  msg.velocity.assign(q_cmd.begin(), q_cmd.end());
  reference_pub_->publish(msg);
}

void TeleopWbcNode::PublishMarkers(
    const BaseDisturbanceGenerator::MountPose& mount,
    const rclcpp::Time& stamp) {
  const auto lifetime = rclcpp::Duration::from_seconds(0.5);
  const auto actual_ee_pose = Se3ToPose(ComputeEeInWorld(mount));
  const auto target_pose = Se3ToPose(ee_target_world_);
  const Eigen::Vector3d ee_pos = ComputeEeInWorld(mount).translation();

  visualization_msgs::msg::MarkerArray array;
  visualization_msgs::msg::Marker sphere_boundary;
  sphere_boundary.header.frame_id = "world";
  sphere_boundary.header.stamp = stamp;
  sphere_boundary.ns = "teleop_wbc";
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
  array.markers.push_back(sphere_boundary);

  visualization_msgs::msg::Marker target_ee;
  target_ee.header.frame_id = "world";
  target_ee.header.stamp = stamp;
  target_ee.ns = "teleop_wbc";
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
  array.markers.push_back(target_ee);

  visualization_msgs::msg::Marker actual_marker;
  actual_marker.header.frame_id = "world";
  actual_marker.header.stamp = stamp;
  actual_marker.ns = "teleop_wbc";
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
  array.markers.push_back(actual_marker);

  base_trail_.push_back(ee_pos);
  if (base_trail_.size() > kTrailLength) {
    base_trail_.pop_front();
  }
  if (base_trail_.size() >= 2) {
    visualization_msgs::msg::Marker trail;
    trail.header.frame_id = "world";
    trail.header.stamp = stamp;
    trail.ns = "teleop_wbc";
    trail.id = 3;
    trail.type = visualization_msgs::msg::Marker::LINE_STRIP;
    trail.action = visualization_msgs::msg::Marker::ADD;
    trail.scale.x = 0.004;
    trail.color.r = 1.0f;
    trail.color.g = 0.55f;
    trail.color.b = 0.1f;
    trail.color.a = 0.85f;
    trail.lifetime = lifetime;
    for (const auto& pt : base_trail_) {
      geometry_msgs::msg::Point p;
      p.x = pt.x();
      p.y = pt.y();
      p.z = pt.z();
      trail.points.push_back(p);
    }
    array.markers.push_back(trail);
  }

  array.markers.push_back(MakeTextLabel(
      4,
      teleop_control_mode_ == "joint_mirror" ? "Mode A: joint mirror"
                                             : "Mode B: QP-WBC + integral",
      target_pose,
      0.2f,
      0.95f,
      0.35f,
      stamp,
      lifetime,
      0.08));

  marker_array_pub_->publish(array);
}

void TeleopWbcNode::PublishMetrics(
    const TaskMetrics& metrics,
    const BaseDisturbanceGenerator::MountPose& mount,
    const rclcpp::Time& stamp) {
  const pinocchio::SE3 T_ee_world = ComputeEeInWorld(mount);
  const double world_pos_err = (
      ee_target_world_.translation() - T_ee_world.translation()).norm();
  const double world_orient_err = pinocchio::log3(
      T_ee_world.rotation().transpose() * ee_target_world_.rotation()).norm();

  std_msgs::msg::Float64MultiArray msg;
  msg.data = {
      world_pos_err,
      world_orient_err,
      metrics.position_error,
      metrics.orientation_error,
      mount.position.x(),
      mount.position.y(),
      mount.position.z(),
      q_[3],
      wbc_state_.integral_error[0],
      wbc_state_.integral_error[1],
      wbc_state_.integral_error[2],
      wbc_state_.integral_error[3],
      wbc_state_.integral_error[4],
      wbc_state_.integral_error[5],
      last_solve_time_us_};
  msg.layout.dim.resize(1);
  msg.layout.dim[0].label = "metrics";
  msg.layout.dim[0].size = msg.data.size();
  msg.layout.dim[0].stride = msg.data.size();
  (void)stamp;
  error_pub_->publish(msg);

  if (control_step_ % static_cast<uint64_t>(control_rate_) == 0) {
    RCLCPP_INFO(
        get_logger(),
        "World EE err pos=%.4f m orient=%.4f rad | base task pos=%.4f m",
        world_pos_err,
        world_orient_err,
        metrics.position_error);
  }
}

void TeleopWbcNode::MasterJointCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  for (size_t i = 0; i < PinocchioArmModel::kDof; ++i) {
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

pinocchio::SE3 TeleopWbcNode::ComputeMasterEeInWorld() const {
  return master_fk_->ComputeEePoseInBase(master_fk_->PackQ(master_q_));
}

bool TeleopWbcNode::UpdateTeleopTarget(double dt) {
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

void TeleopWbcNode::MountDisturbanceCallback(
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

BaseDisturbanceGenerator::MountPose TeleopWbcNode::GetMountPose(double dt) {
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
  BaseDisturbanceGenerator::MountPose mount;
  mount.position = Eigen::Vector3d::Zero();
  mount.rpy = Eigen::Vector3d::Zero();
  mount.orientation = Eigen::Quaterniond::Identity();
  return mount;
}
