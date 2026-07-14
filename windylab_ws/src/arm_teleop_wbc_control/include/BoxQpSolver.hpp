#pragma once

#include <Eigen/Dense>

/// Box-constrained convex QP: min 0.5 x^T H x + g^T x  s.t. lb <= x <= ub
class BoxQpSolver {
 public:
  struct Params {
    int max_iterations = 80;
    double step_size = 0.18;
    double tolerance = 1e-5;
  };

  BoxQpSolver();
  explicit BoxQpSolver(Params params);

  Eigen::VectorXd Solve(
      const Eigen::MatrixXd& H,
      const Eigen::VectorXd& g,
      const Eigen::VectorXd& lb,
      const Eigen::VectorXd& ub,
      const Eigen::VectorXd& x0) const;

 private:
  Params params_;
};
