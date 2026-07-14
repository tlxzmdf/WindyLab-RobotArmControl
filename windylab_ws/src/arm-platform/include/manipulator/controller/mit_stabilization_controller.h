#pragma once

#include <vector>

#include <manipulator/controller/i_arm_controller.h>

namespace manipulator::controller {

/// MIT motor command mapper for EE stabilization on real hardware.
/// Maps joint setpoint (q, dq, tau feedforward) to MotorControl (p, d, position, velocity, current).
class MitStabilizationController : public IArmController {
 public:
  MitStabilizationController();
  ~MitStabilizationController() override = default;

  void SetKpKd(const std::vector<double>& kp, const std::vector<double>& kd);
  void SetTorqueLimit(double torque_limit);

  JointCommand Compute(
      const JointStates& joint_states,
      const JointSetpoint& joint_setpoint,
      double dt) override;

 private:
  std::vector<double> kp_;
  std::vector<double> kd_;
  double torque_limit_{9.0};
};

}  // namespace manipulator::controller
