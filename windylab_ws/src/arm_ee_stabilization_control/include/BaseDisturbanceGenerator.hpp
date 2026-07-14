#pragma once

#include <functional>
#include <random>

#include <Eigen/Core>
#include <Eigen/Geometry>

/// Large-amplitude mount motion inside a sphere for UAV-arm disturbance simulation.
class BaseDisturbanceGenerator {
 public:
  struct MountPose {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpy{Eigen::Vector3d::Zero()};
    Eigen::Vector3d linear_velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  };

  using GoalValidator = std::function<bool(const MountPose&)>;

  BaseDisturbanceGenerator(
      const Eigen::Vector3d& sphere_center,
      double sphere_radius,
      double orientation_amplitude_rad,
      double time_constant,
      double amplitude_scale = 0.92,
      unsigned seed = 42);

  MountPose Step(double dt);

  void SetCenter(const Eigen::Vector3d& center);
  void SetGoalValidator(GoalValidator validator);

  const Eigen::Vector3d& sphere_center() const { return center_; }
  double sphere_radius() const { return radius_; }

 private:
  void PickNewWaypoint();
  double SmoothStep(double u) const;

  Eigen::Vector3d center_;
  double radius_;
  double orient_amp_;
  double time_constant_;
  double amplitude_scale_;
  std::mt19937 rng_;
  std::uniform_real_distribution<double> uniform_{0.0, 1.0};

  Eigen::Vector3d pos_start_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d pos_goal_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d rpy_start_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d rpy_goal_{Eigen::Vector3d::Zero()};
  double segment_elapsed_{0.0};
  MountPose current_{};
  GoalValidator goal_validator_{};
};
