#pragma once

#include <array>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/se3.hpp>

class PinocchioArmModel {
 public:
  static constexpr size_t kDof = 6;

  PinocchioArmModel(
      const std::string& urdf_path,
      const std::vector<std::string>& joint_names,
      const std::string& base_frame,
      const std::string& ee_frame);

  Eigen::VectorXd PackQ(const std::array<double, kDof>& q) const;
  pinocchio::SE3 ComputeEePoseInBase(const Eigen::VectorXd& q) const;
  Eigen::Matrix<double, 6, 6> ComputeArmJacobian(const Eigen::VectorXd& q) const;

 private:
  pinocchio::Model model_;
  mutable pinocchio::Data data_{pinocchio::Model()};
  pinocchio::FrameIndex ee_frame_id_{0};
  std::vector<int> q_indices_;
  std::vector<int> v_indices_;
};
