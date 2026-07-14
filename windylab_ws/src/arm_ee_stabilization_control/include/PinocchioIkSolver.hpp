#pragma once

#include <array>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/se3.hpp>

/// Pinocchio 6-DOF IK solver with continuous damped least-squares steps.
class PinocchioIkSolver {
 public:
  static constexpr size_t kDof = 6;

  struct Params {
    int max_iters{1000};
    double position_tolerance{1e-4};
    double orientation_tolerance{1e-2};
    double partial_position_tolerance{0.004};
    double partial_orientation_tolerance{0.06};
    double orientation_weight{1.0};
    double damping{1e-3};
    double step_scale{0.5};
    double max_step{0.12};
  };

  struct SolveResult {
    bool success{false};
    bool acceptable{false};
    double position_error{0.0};
    double orientation_error{0.0};
    int iterations{0};
  };

  PinocchioIkSolver(
      const std::string& urdf_path,
      const std::vector<std::string>& joint_names,
      const std::string& base_frame,
      const std::string& ee_frame,
      const Params& params);

  const std::array<double, kDof>& current_q() const { return current_q_; }
  void set_current_q(const std::array<double, kDof>& q) { current_q_ = q; }
  void set_reference_q(const std::array<double, kDof>& q_ref, double nullspace_gain);
  void clear_reference_q();

  static double NormalizeAngle(double value);
  static double UnwrapNear(double angle, double reference);

  geometry_msgs::msg::Pose ComputeEePoseInBase() const;
  SolveResult Solve(const geometry_msgs::msg::Pose& target_in_base);
  SolveResult SolveSe3(const pinocchio::SE3& target_in_base, int max_iters);
  SolveResult RefineSe3(const pinocchio::SE3& target_in_base, int extra_iters);

 private:
  static double Clamp(double value, double min_value, double max_value);
  pinocchio::SE3 TargetWorldPose(const pinocchio::SE3& target_in_base) const;
  SolveResult IterateToward(const pinocchio::SE3& target_in_base, int max_iters);

  Params params_;
  std::vector<std::string> joint_names_;
  std::string base_frame_;
  std::string ee_frame_;
  pinocchio::Model model_;
  mutable pinocchio::Data data_{pinocchio::Model()};
  pinocchio::FrameIndex base_frame_id_{0};
  pinocchio::FrameIndex ee_frame_id_{0};
  std::vector<int> q_indices_;
  std::vector<int> v_indices_;
  std::array<double, kDof> current_q_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, kDof> reference_q_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  double nullspace_gain_{0.0};
  bool use_reference_q_{false};
};
