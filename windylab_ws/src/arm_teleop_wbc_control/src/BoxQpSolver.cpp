#include "BoxQpSolver.hpp"

#include <algorithm>

BoxQpSolver::BoxQpSolver() : params_{} {}

BoxQpSolver::BoxQpSolver(Params params) : params_(params) {}

Eigen::VectorXd BoxQpSolver::Solve(
    const Eigen::MatrixXd& H,
    const Eigen::VectorXd& g,
    const Eigen::VectorXd& lb,
    const Eigen::VectorXd& ub,
    const Eigen::VectorXd& x0) const {
  (void)x0;
  Eigen::LDLT<Eigen::MatrixXd> ldlt(H);
  Eigen::VectorXd x = ldlt.solve(-g);
  if (ldlt.info() != Eigen::Success || !x.allFinite()) {
    x = Eigen::VectorXd::Zero(g.size());
  }

  for (Eigen::Index i = 0; i < x.size(); ++i) {
    x[i] = std::clamp(x[i], lb[i], ub[i]);
  }

  for (int iter = 0; iter < params_.max_iterations; ++iter) {
    const Eigen::VectorXd grad = H * x + g;
    Eigen::VectorXd x_new = x;
    for (Eigen::Index i = 0; i < x.size(); ++i) {
      const double xi = x[i] - params_.step_size * grad[i];
      x_new[i] = std::clamp(xi, lb[i], ub[i]);
    }
    if ((x_new - x).norm() < params_.tolerance) {
      x = x_new;
      break;
    }
    x = x_new;
  }
  return x;
}
