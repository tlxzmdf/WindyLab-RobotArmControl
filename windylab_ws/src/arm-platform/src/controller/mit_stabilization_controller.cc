#include <algorithm>

#include <manipulator/controller/mit_stabilization_controller.h>

namespace manipulator::controller {

MitStabilizationController::MitStabilizationController() {
  // Default matches stabilization_hw_student_arm.yaml (2026-07-17 HW tune):
  // Phase 1: proximal kp60/kd2.2; wrist lower kp / higher kd (anti-sawtooth).
  kp_ = {60.0, 60.0, 60.0, 2.5, 2.5, 2.5, 1.0};
  kd_ = {2.2, 2.2, 2.2, 0.45, 0.45, 0.45, 0.1};
}

void MitStabilizationController::SetKpKd(
    const std::vector<double>& kp, const std::vector<double>& kd) {
  kp_ = kp;
  kd_ = kd;
}

void MitStabilizationController::SetTorqueLimit(double torque_limit) {
  torque_limit_ = torque_limit;
}

JointCommand MitStabilizationController::Compute(
    const JointStates& joint_states,
    const JointSetpoint& joint_setpoint,
    double dt) {
  (void)dt;
  JointCommand cmd;
  const size_t num_joints = joint_states.position.size();
  if (joint_setpoint.q.size() == 0) {
    return cmd;
  }

  cmd.position.resize(num_joints, 0.0);
  cmd.velocity.resize(num_joints, 0.0);
  cmd.current.resize(num_joints, 0.0);
  cmd.p.resize(num_joints, 0.0);
  cmd.d.resize(num_joints, 0.0);

  for (size_t i = 0; i < num_joints; ++i) {
    cmd.p[i] = (i < kp_.size()) ? kp_[i] : kp_.back();
    cmd.d[i] = (i < kd_.size()) ? kd_[i] : kd_.back();
  }

  const size_t n = std::min(num_joints, static_cast<size_t>(joint_setpoint.q.size()));
  for (size_t i = 0; i < n; ++i) {
    cmd.position[i] = joint_setpoint.q[static_cast<Eigen::Index>(i)];
    if (joint_setpoint.dq.size() > static_cast<Eigen::Index>(i)) {
      cmd.velocity[i] = joint_setpoint.dq[static_cast<Eigen::Index>(i)];
    }
    if (joint_setpoint.tau.size() > static_cast<Eigen::Index>(i)) {
      cmd.current[i] = std::clamp(
          joint_setpoint.tau[static_cast<Eigen::Index>(i)],
          -torque_limit_, torque_limit_);
    }
  }
  return cmd;
}

}  // namespace manipulator::controller
