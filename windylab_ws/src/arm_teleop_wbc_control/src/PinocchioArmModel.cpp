#include "PinocchioArmModel.hpp"

#include <stdexcept>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

PinocchioArmModel::PinocchioArmModel(
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
  ee_frame_id_ = model_.getFrameId(ee_frame);

  for (const auto& name : joint_names) {
    if (!model_.existJointName(name)) {
      throw std::runtime_error("Joint not found: " + name);
    }
    const auto joint_id = model_.getJointId(name);
    const auto& joint_model = model_.joints[joint_id];
    if (joint_model.nq() != 1 || joint_model.nv() != 1) {
      throw std::runtime_error("Only 1-DoF joints supported: " + name);
    }
    q_indices_.push_back(joint_model.idx_q());
    v_indices_.push_back(joint_model.idx_v());
  }
}

Eigen::VectorXd PinocchioArmModel::PackQ(const std::array<double, kDof>& q) const {
  Eigen::VectorXd q_full = pinocchio::neutral(model_);
  for (size_t i = 0; i < kDof; ++i) {
    q_full[q_indices_[i]] = q[i];
  }
  return q_full;
}

pinocchio::SE3 PinocchioArmModel::ComputeEePoseInBase(const Eigen::VectorXd& q) const {
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  return data_.oMf[ee_frame_id_];
}

Eigen::Matrix<double, 6, 6> PinocchioArmModel::ComputeArmJacobian(
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
