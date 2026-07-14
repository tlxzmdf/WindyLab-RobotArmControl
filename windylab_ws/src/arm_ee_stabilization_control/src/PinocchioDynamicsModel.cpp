#include "PinocchioDynamicsModel.hpp"

#include <algorithm>
#include <cmath>

#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>

PinocchioDynamicsModel::PinocchioDynamicsModel(
    const std::string& urdf_path,
    const std::vector<std::string>& joint_names,
    const std::string& base_frame,
    const std::string& ee_frame) {
  pinocchio::urdf::buildModel(urdf_path, model_);
  data_ = pinocchio::Data(model_);

  if (!model_.existFrame(base_frame)) {
    throw std::runtime_error("Base frame not found: " + base_frame);
  }
  if (!model_.existFrame(ee_frame)) {
    throw std::runtime_error("End-effector frame not found: " + ee_frame);
  }
  base_frame_id_ = model_.getFrameId(base_frame);
  ee_frame_id_ = model_.getFrameId(ee_frame);

  for (const auto& name : joint_names) {
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

Eigen::VectorXd PinocchioDynamicsModel::PackQ(
    const std::array<double, kDof>& q) const {
  Eigen::VectorXd q_full = pinocchio::neutral(model_);
  for (size_t i = 0; i < kDof; ++i) {
    q_full[q_indices_[i]] = q[i];
  }
  return q_full;
}

Eigen::VectorXd PinocchioDynamicsModel::PackV(
    const std::array<double, kDof>& v) const {
  Eigen::VectorXd v_full = Eigen::VectorXd::Zero(model_.nv);
  for (size_t i = 0; i < kDof; ++i) {
    v_full[v_indices_[i]] = v[i];
  }
  return v_full;
}

void PinocchioDynamicsModel::UnpackQ(
    const Eigen::VectorXd& q, std::array<double, kDof>& out) const {
  for (size_t i = 0; i < kDof; ++i) {
    out[i] = q[q_indices_[i]];
  }
}

void PinocchioDynamicsModel::UnpackV(
    const Eigen::VectorXd& v, std::array<double, kDof>& out) const {
  for (size_t i = 0; i < kDof; ++i) {
    out[i] = v[v_indices_[i]];
  }
}

Eigen::MatrixXd PinocchioDynamicsModel::ComputeMassMatrix(
    const Eigen::VectorXd& q) {
  pinocchio::crba(model_, data_, q);
  data_.M.triangularView<Eigen::StrictlyLower>() =
      data_.M.transpose().triangularView<Eigen::StrictlyLower>();
  return data_.M;
}

Eigen::VectorXd PinocchioDynamicsModel::ComputeNonLinearEffects(
    const Eigen::VectorXd& q, const Eigen::VectorXd& v) {
  return pinocchio::nonLinearEffects(model_, data_, q, v);
}

Eigen::VectorXd PinocchioDynamicsModel::ComputeTrackingTorque(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& v,
    const Eigen::VectorXd& q_d,
    const Eigen::VectorXd& v_d,
    const Eigen::VectorXd& a_d,
    const Eigen::VectorXd& kp,
    const Eigen::VectorXd& kd) {
  const Eigen::VectorXd e = q_d - q;
  const Eigen::VectorXd ed = v_d - v;
  const Eigen::VectorXd a_cmd = a_d + kp.cwiseProduct(e) + kd.cwiseProduct(ed);

  const Eigen::MatrixXd M = ComputeMassMatrix(q);
  const Eigen::VectorXd nle = ComputeNonLinearEffects(q, v);
  return M * a_cmd + nle;
}

Eigen::VectorXd PinocchioDynamicsModel::ForwardDynamics(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& v,
    const Eigen::VectorXd& tau) {
  return pinocchio::aba(model_, data_, q, v, tau);
}

pinocchio::SE3 PinocchioDynamicsModel::ComputeEePoseInBase(
    const Eigen::VectorXd& q) const {
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  return data_.oMf[ee_frame_id_];
}

Eigen::Matrix<double, 6, 6> PinocchioDynamicsModel::ComputeArmJacobian(
    const Eigen::VectorXd& q) const {
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);

  Eigen::Matrix<double, 6, Eigen::Dynamic> J_lwa(6, model_.nv);
  pinocchio::computeFrameJacobian(
      model_, data_, q, ee_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J_lwa);
  Eigen::Matrix<double, 6, Eigen::Dynamic> J_local(6, model_.nv);
  pinocchio::computeFrameJacobian(
      model_, data_, q, ee_frame_id_, pinocchio::LOCAL, J_local);

  Eigen::Matrix<double, 6, 6> J;
  J.setZero();
  for (size_t i = 0; i < kDof; ++i) {
    const Eigen::Index col = static_cast<Eigen::Index>(i);
    J.block<3, 1>(0, col) = J_lwa.block<3, 1>(0, v_indices_[i]);
    J.block<3, 1>(3, col) = J_local.block<3, 1>(3, v_indices_[i]);
  }
  return J;
}

std::array<double, PinocchioDynamicsModel::kDof>
PinocchioDynamicsModel::ComputeClikJointVelocity(
    const Eigen::VectorXd& q,
    const pinocchio::SE3& T_des_base,
    const Eigen::Matrix<double, 6, 1>& v_task_des,
    const Eigen::Matrix<double, 6, 1>& clik_kp,
    const double damping,
    const double max_joint_velocity) const {
  const pinocchio::SE3 T_ee = ComputeEePoseInBase(q);

  Eigen::Matrix<double, 6, 1> err;
  err.head<3>() = T_des_base.translation() - T_ee.translation();
  err.tail<3>() = pinocchio::log3(
      T_ee.rotation().transpose() * T_des_base.rotation());

  const Eigen::Matrix<double, 6, 6> J = ComputeArmJacobian(q);
  const Eigen::Matrix<double, 6, 1> twist_cmd =
      clik_kp.cwiseProduct(err) + v_task_des;
  const Eigen::Matrix<double, 6, 6> lhs =
      J * J.transpose() +
      damping * damping * Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix<double, 6, 1> dq =
      J.transpose() * lhs.ldlt().solve(twist_cmd);

  std::array<double, kDof> dq_out{};
  for (size_t i = 0; i < kDof; ++i) {
    dq_out[i] = std::max(
        -max_joint_velocity,
        std::min(max_joint_velocity, dq(static_cast<Eigen::Index>(i))));
  }
  return dq_out;
}

namespace {
double UnwrapNearAngle(double angle, double reference) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  while (angle - reference > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle - reference < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}
}  // namespace

std::array<double, PinocchioDynamicsModel::kDof>
PinocchioDynamicsModel::ComputeClikJointVelocityWithNullSpace(
    const Eigen::VectorXd& q,
    const pinocchio::SE3& T_des_base,
    const Eigen::Matrix<double, 6, 1>& v_task_des,
    const std::array<double, kDof>& q_ref,
    const Eigen::Matrix<double, 6, 1>& clik_kp,
    const double damping,
    const double nullspace_gain,
    const double max_joint_velocity) const {
  const pinocchio::SE3 T_ee = ComputeEePoseInBase(q);

  Eigen::Matrix<double, 6, 1> err;
  err.head<3>() = T_des_base.translation() - T_ee.translation();
  err.tail<3>() = pinocchio::log3(
      T_ee.rotation().transpose() * T_des_base.rotation());

  const Eigen::Matrix<double, 6, 6> J = ComputeArmJacobian(q);
  const Eigen::Matrix<double, 6, 1> twist_cmd =
      clik_kp.cwiseProduct(err) + v_task_des;
  const Eigen::Matrix<double, 6, 6> lhs =
      J * J.transpose() +
      damping * damping * Eigen::Matrix<double, 6, 6>::Identity();
  const Eigen::Matrix<double, 6, 6> jsharp =
      J.transpose() * lhs.ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity());
  Eigen::Matrix<double, 6, 1> dq = jsharp * twist_cmd;

  if (nullspace_gain > 1e-9) {
    const Eigen::Matrix<double, 6, 6> nullspace_projector =
        Eigen::Matrix<double, 6, 6>::Identity() - jsharp * J;
    Eigen::Matrix<double, 6, 1> q_err;
    std::array<double, kDof> q_arr{};
    UnpackQ(q, q_arr);
    for (size_t i = 0; i < kDof; ++i) {
      q_err[static_cast<Eigen::Index>(i)] =
          UnwrapNearAngle(q_ref[i], q_arr[i]) - q_arr[i];
    }
    dq += nullspace_gain * nullspace_projector * q_err;
  }

  std::array<double, kDof> dq_out{};
  for (size_t i = 0; i < kDof; ++i) {
    dq_out[i] = std::max(
        -max_joint_velocity,
        std::min(max_joint_velocity, dq(static_cast<Eigen::Index>(i))));
  }
  return dq_out;
}

Eigen::VectorXd PinocchioDynamicsModel::ComputeOperationalTorque(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& v,
    const pinocchio::SE3& T_des_base,
    const Eigen::Matrix<double, 6, 1>& v_des,
    const OperationalGains& gains,
    OperationalResult* metrics) {
  pinocchio::forwardKinematics(model_, data_, q, v);
  pinocchio::updateFramePlacements(model_, data_);
  const pinocchio::SE3& T_ee = data_.oMf[ee_frame_id_];

  Eigen::Matrix<double, 6, 1> err;
  err.head<3>() = T_des_base.translation() - T_ee.translation();
  err.tail<3>() = pinocchio::log3(
      T_ee.rotation().transpose() * T_des_base.rotation());

  const Eigen::Matrix<double, 6, 6> J = ComputeArmJacobian(q);

  const Eigen::Matrix<double, 6, 1> v_ee = J * v;
  const Eigen::Matrix<double, 6, 1> e_dot = v_des - v_ee;

  pinocchio::crba(model_, data_, q);
  data_.M.triangularView<Eigen::StrictlyLower>() =
      data_.M.transpose().triangularView<Eigen::StrictlyLower>();

  Eigen::Matrix<double, 6, 6> M_arm;
  for (size_t i = 0; i < kDof; ++i) {
    for (size_t j = 0; j < kDof; ++j) {
      M_arm(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) =
          data_.M(v_indices_[i], v_indices_[j]);
    }
  }

  const Eigen::Matrix<double, 6, 6> M_inv = M_arm.ldlt().solve(
      Eigen::Matrix<double, 6, 6>::Identity());
  Eigen::Matrix<double, 6, 6> Lambda_inv = J * M_inv * J.transpose();
  Lambda_inv += gains.lambda * gains.lambda * Eigen::Matrix<double, 6, 6>::Identity();
  const Eigen::Matrix<double, 6, 6> Lambda =
      Lambda_inv.ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity());

  const Eigen::Matrix<double, 6, 1> a_task =
      gains.kp.cwiseProduct(err) + gains.kd.cwiseProduct(e_dot);
  Eigen::Matrix<double, 6, 1> a_clamped = a_task;
  for (int i = 0; i < 6; ++i) {
    a_clamped[i] = std::max(-220.0, std::min(220.0, a_clamped[i]));
  }
  const Eigen::Matrix<double, 6, 1> f_task = Lambda * a_clamped;

  Eigen::VectorXd tau = Eigen::VectorXd::Zero(model_.nv);
  for (size_t i = 0; i < kDof; ++i) {
    tau[v_indices_[i]] = J.col(static_cast<Eigen::Index>(i)).dot(f_task);
  }

  const Eigen::VectorXd nle = pinocchio::nonLinearEffects(model_, data_, q, v);
  tau += nle;

  if (metrics != nullptr) {
    metrics->task_error = err;
    metrics->position_error = err.head<3>().norm();
    metrics->orientation_error = err.tail<3>().norm();
  }

  return tau;
}

Eigen::Vector3d PinocchioDynamicsModel::ComputeEePositionInWorld(
    const Eigen::VectorXd& q) const {
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  return data_.oMf[ee_frame_id_].translation();
}

void PinocchioDynamicsModel::AddArmJointTorques(
    Eigen::VectorXd& tau,
    const std::array<double, kDof>& arm_tau) const {
  for (size_t i = 0; i < kDof; ++i) {
    tau[v_indices_[i]] += arm_tau[i];
  }
}

Eigen::Matrix<double, 6, 6> PinocchioDynamicsModel::ExtractArmMassMatrix(
    const Eigen::MatrixXd& M_full) const {
  Eigen::Matrix<double, 6, 6> M_arm = Eigen::Matrix<double, 6, 6>::Zero();
  for (size_t i = 0; i < kDof; ++i) {
    for (size_t j = 0; j < kDof; ++j) {
      M_arm(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) =
          M_full(v_indices_[i], v_indices_[j]);
    }
  }
  return M_arm;
}

Eigen::Matrix<double, 6, 1> PinocchioDynamicsModel::ExtractArmSubvector(
    const Eigen::VectorXd& full) const {
  Eigen::Matrix<double, 6, 1> arm = Eigen::Matrix<double, 6, 1>::Zero();
  for (size_t i = 0; i < kDof; ++i) {
    arm[static_cast<Eigen::Index>(i)] = full[v_indices_[i]];
  }
  return arm;
}
