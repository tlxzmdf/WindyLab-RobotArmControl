#include "BaseDisturbanceGenerator.hpp"

#include <algorithm>
#include <cmath>

namespace {
Eigen::Quaterniond RpyToQuat(const Eigen::Vector3d& rpy) {
  const Eigen::AngleAxisd roll(rpy.x(), Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch(rpy.y(), Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw(rpy.z(), Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q = yaw * pitch * roll;
  q.normalize();
  return q;
}

Eigen::Vector3d RandomUnitVector(std::mt19937& rng) {
  std::uniform_real_distribution<double> dist(-1.0, 1.0);
  Eigen::Vector3d v(dist(rng), dist(rng), dist(rng));
  const double norm = v.norm();
  if (norm < 1e-9) {
    return Eigen::Vector3d::UnitX();
  }
  return v / norm;
}
}  // namespace

BaseDisturbanceGenerator::BaseDisturbanceGenerator(
    const Eigen::Vector3d& sphere_center,
    double sphere_radius,
    double orientation_amplitude_rad,
    double time_constant,
    double amplitude_scale,
    unsigned seed)
    : center_(sphere_center),
      radius_(sphere_radius),
      orient_amp_(orientation_amplitude_rad),
      time_constant_(time_constant),
      amplitude_scale_(std::clamp(amplitude_scale, 0.2, 1.0)),
      rng_(seed) {
  current_.position = center_;
  current_.rpy.setZero();
  current_.orientation = Eigen::Quaterniond::Identity();
  pos_start_ = center_;
  rpy_start_.setZero();
  PickNewWaypoint();
}

void BaseDisturbanceGenerator::PickNewWaypoint() {
  pos_start_ = current_.position;
  rpy_start_ = current_.rpy;

  const double radial = radius_ * amplitude_scale_;
  constexpr int kMaxTries = 20;
  bool found = false;
  for (int attempt = 0; attempt < kMaxTries; ++attempt) {
    pos_goal_ = center_ + radial * RandomUnitVector(rng_);
    rpy_goal_ = Eigen::Vector3d(
        orient_amp_ * (2.0 * uniform_(rng_) - 1.0),
        orient_amp_ * (2.0 * uniform_(rng_) - 1.0),
        orient_amp_ * (2.0 * uniform_(rng_) - 1.0));

    MountPose candidate;
    candidate.position = pos_goal_;
    candidate.rpy = rpy_goal_;
    candidate.orientation = RpyToQuat(rpy_goal_);
    if (!goal_validator_ || goal_validator_(candidate)) {
      found = true;
      break;
    }
  }
  if (!found) {
    pos_goal_ = current_.position;
    rpy_goal_ = current_.rpy;
  }

  segment_elapsed_ = 0.0;
}

void BaseDisturbanceGenerator::SetGoalValidator(GoalValidator validator) {
  goal_validator_ = std::move(validator);
}

double BaseDisturbanceGenerator::SmoothStep(double u) const {
  const double x = std::clamp(u, 0.0, 1.0);
  return x * x * (3.0 - 2.0 * x);
}

BaseDisturbanceGenerator::MountPose BaseDisturbanceGenerator::Step(double dt) {
  const Eigen::Vector3d prev_pos = current_.position;
  const Eigen::Vector3d prev_rpy = current_.rpy;

  segment_elapsed_ += dt;
  const double u = segment_elapsed_ / std::max(time_constant_, 0.05);
  const double s = SmoothStep(u);
  const double sdot = (u < 1.0) ? (6.0 * u * (1.0 - u) / std::max(time_constant_, 0.05)) : 0.0;

  const Eigen::Vector3d pos_delta = pos_goal_ - pos_start_;
  const Eigen::Vector3d rpy_delta = rpy_goal_ - rpy_start_;

  current_.position = pos_start_ + s * pos_delta;
  current_.rpy = rpy_start_ + s * rpy_delta;
  current_.linear_velocity = sdot * pos_delta;
  current_.angular_velocity = sdot * rpy_delta;
  current_.orientation = RpyToQuat(current_.rpy);

  if (u >= 1.0) {
    PickNewWaypoint();
  }

  return current_;
}

void BaseDisturbanceGenerator::SetCenter(const Eigen::Vector3d& center) {
  center_ = center;
  pos_start_ = center_;
  pos_goal_ = center_;
  current_.position = center_;
  segment_elapsed_ = 0.0;
  PickNewWaypoint();
}
