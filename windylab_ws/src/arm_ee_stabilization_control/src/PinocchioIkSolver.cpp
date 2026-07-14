#include "PinocchioIkSolver.hpp"

#include <cmath>
#include <stdexcept>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/log.hpp>

double PinocchioIkSolver::NormalizeAngle(double value) {
  while (value > M_PI) {
    value -= 2.0 * M_PI;
  }
  while (value < -M_PI) {
    value += 2.0 * M_PI;
  }
  return value;
}

double PinocchioIkSolver::UnwrapNear(double angle, double reference) {
  double value = NormalizeAngle(angle);
  while (value - reference > M_PI) {
    value -= 2.0 * M_PI;
  }
  while (value - reference < -M_PI) {
    value += 2.0 * M_PI;
  }
  return value;
}

void PinocchioIkSolver::set_reference_q(
    const std::array<double, kDof>& q_ref, double nullspace_gain) {
  reference_q_ = q_ref;
  nullspace_gain_ = nullspace_gain;
  use_reference_q_ = nullspace_gain > 1e-9;
}

void PinocchioIkSolver::clear_reference_q() {
  nullspace_gain_ = 0.0;
  use_reference_q_ = false;
}

double PinocchioIkSolver::Clamp(double value, double min_value, double max_value) {
  return std::max(min_value, std::min(max_value, value));
}

PinocchioIkSolver::PinocchioIkSolver(
    const std::string& urdf_path,
    const std::vector<std::string>& joint_names,
    const std::string& base_frame,
    const std::string& ee_frame,
    const Params& params)
    : params_(params),
      joint_names_(joint_names),
      base_frame_(base_frame),
      ee_frame_(ee_frame) {
  pinocchio::urdf::buildModel(urdf_path, model_);
  data_ = pinocchio::Data(model_);

  if (!model_.existFrame(base_frame_)) {
    throw std::runtime_error("Base frame not found: " + base_frame_);
  }
  if (!model_.existFrame(ee_frame_)) {
    throw std::runtime_error("End-effector frame not found: " + ee_frame_);
  }
  base_frame_id_ = model_.getFrameId(base_frame_);
  ee_frame_id_ = model_.getFrameId(ee_frame_);

  for (const auto& name : joint_names_) {
    if (!model_.existJointName(name)) {
      throw std::runtime_error("Joint not found in URDF: " + name);
    }
    const auto joint_id = model_.getJointId(name);
    const auto& joint_model = model_.joints[joint_id];
    if (joint_model.nq() != 1 || joint_model.nv() != 1) {
      throw std::runtime_error("Only 1-DoF joints are supported: " + name);
    }
    q_indices_.push_back(joint_model.idx_q());
    v_indices_.push_back(joint_model.idx_v());
  }
}

pinocchio::SE3 PinocchioIkSolver::TargetWorldPose(
    const pinocchio::SE3& target_in_base) const {
  Eigen::VectorXd q = pinocchio::neutral(model_);
  for (size_t i = 0; i < kDof; ++i) {
    q[q_indices_[i]] = current_q_[i];
  }
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  return data_.oMf[base_frame_id_] * target_in_base;
}

geometry_msgs::msg::Pose PinocchioIkSolver::ComputeEePoseInBase() const {
  Eigen::VectorXd q = pinocchio::neutral(model_);
  for (size_t i = 0; i < kDof; ++i) {
    q[q_indices_[i]] = current_q_[i];
  }

  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);

  const pinocchio::SE3 world_ee = data_.oMf[ee_frame_id_];
  const pinocchio::SE3 world_base = data_.oMf[base_frame_id_];
  const pinocchio::SE3 base_ee = world_base.actInv(world_ee);

  geometry_msgs::msg::Pose pose;
  pose.position.x = base_ee.translation().x();
  pose.position.y = base_ee.translation().y();
  pose.position.z = base_ee.translation().z();
  const Eigen::Quaterniond quat(base_ee.rotation());
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  pose.orientation.w = quat.w();
  return pose;
}

PinocchioIkSolver::SolveResult PinocchioIkSolver::IterateToward(
    const pinocchio::SE3& target_in_base, int max_iters) {
  SolveResult result;
  Eigen::VectorXd q = pinocchio::neutral(model_);
  for (size_t i = 0; i < kDof; ++i) {
    q[q_indices_[i]] = current_q_[i];
  }

  const pinocchio::SE3 target_pose = TargetWorldPose(target_in_base);

  for (int iter = 0; iter < max_iters; ++iter) {
    pinocchio::forwardKinematics(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);

    const auto& ee_pose = data_.oMf[ee_frame_id_];
    const Eigen::Vector3d position_error =
        target_pose.translation() - ee_pose.translation();
    const Eigen::Vector3d orientation_error =
        pinocchio::log3(target_pose.rotation() * ee_pose.rotation().transpose());
    result.position_error = position_error.norm();
    result.orientation_error = orientation_error.norm();
    result.iterations = iter + 1;

    result.success =
        result.position_error < params_.position_tolerance &&
        result.orientation_error < params_.orientation_tolerance;
    result.acceptable =
        result.position_error < params_.partial_position_tolerance &&
        result.orientation_error < params_.partial_orientation_tolerance;
    if (result.success) {
      break;
    }

    Eigen::Matrix<double, 6, 1> error;
    error.head<3>() = position_error;
    error.tail<3>() = params_.orientation_weight * orientation_error;

    Eigen::Matrix<double, 6, Eigen::Dynamic> frame_jacobian(6, model_.nv);
    frame_jacobian.setZero();
    pinocchio::computeFrameJacobian(
        model_, data_, q, ee_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED,
        frame_jacobian);

    Eigen::Matrix<double, 6, 6> active_jacobian;
    active_jacobian.setZero();
    for (size_t i = 0; i < kDof; ++i) {
      active_jacobian.col(i).head<3>() =
          frame_jacobian.col(v_indices_[i]).head<3>();
      active_jacobian.col(i).tail<3>() =
          params_.orientation_weight * frame_jacobian.col(v_indices_[i]).tail<3>();
    }

    const Eigen::Matrix<double, 6, 6> lhs =
        active_jacobian * active_jacobian.transpose() +
        params_.damping * params_.damping * Eigen::Matrix<double, 6, 6>::Identity();
    const Eigen::Matrix<double, 6, 6> jsharp =
        active_jacobian.transpose() * lhs.ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity());
    Eigen::Matrix<double, 6, 1> dq = jsharp * error;
    dq *= params_.step_scale;

    if (use_reference_q_) {
      const Eigen::Matrix<double, 6, 6> nullspace_projector =
          Eigen::Matrix<double, 6, 6>::Identity() - jsharp * active_jacobian;
      Eigen::Matrix<double, 6, 1> q_err;
      for (size_t i = 0; i < kDof; ++i) {
        const double q_i = q[q_indices_[i]];
        q_err[i] = UnwrapNear(reference_q_[i], q_i) - q_i;
      }
      dq += nullspace_gain_ * nullspace_projector * q_err;
    }

    for (size_t i = 0; i < kDof; ++i) {
      const double step = Clamp(dq[i], -params_.max_step, params_.max_step);
      q[q_indices_[i]] = UnwrapNear(q[q_indices_[i]] + step, current_q_[i]);
    }
  }

  for (size_t i = 0; i < kDof; ++i) {
    current_q_[i] = UnwrapNear(q[q_indices_[i]], current_q_[i]);
  }

  return result;
}

PinocchioIkSolver::SolveResult PinocchioIkSolver::SolveSe3(
    const pinocchio::SE3& target_in_base, int max_iters) {
  return IterateToward(target_in_base, max_iters);
}

PinocchioIkSolver::SolveResult PinocchioIkSolver::RefineSe3(
    const pinocchio::SE3& target_in_base, int extra_iters) {
  return IterateToward(target_in_base, extra_iters);
}

PinocchioIkSolver::SolveResult PinocchioIkSolver::Solve(
    const geometry_msgs::msg::Pose& target_pose_msg) {
  const Eigen::Vector3d target_position(
      target_pose_msg.position.x,
      target_pose_msg.position.y,
      target_pose_msg.position.z);
  Eigen::Quaterniond target_quaternion(
      target_pose_msg.orientation.w,
      target_pose_msg.orientation.x,
      target_pose_msg.orientation.y,
      target_pose_msg.orientation.z);
  if (target_quaternion.norm() < 1e-9) {
    target_quaternion = Eigen::Quaterniond::Identity();
  }
  target_quaternion.normalize();

  const pinocchio::SE3 target_pose_in_base(
      target_quaternion.toRotationMatrix(), target_position);
  return SolveSe3(target_pose_in_base, params_.max_iters);
}
