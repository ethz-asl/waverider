#include "chomp_ros/chomp_optimizer.h"

#include <iostream>

#include <Eigen/Cholesky>
#include <tracy/Tracy.hpp>

// NOTE: This CHOMP implementation is borrowed from Helen's mav_local_avoidance:
//       https://github.com/ethz-asl/mav_local_avoidance/blob/master/local_trajectory_opt/src/chomp/chomp_optimizer.cpp
namespace chomp {
void SmoothnessFunc::initFromTrajectory(const ChompTrajectory& traj) {
  // N+1 x N square matrix, derivative of trajectory.
  Eigen::MatrixXd K(traj.N - 1, traj.N - 2);
  K.setZero();
  K.diagonal().setOnes();
  K.diagonal(-1).setConstant(-1.0);

  // Represents fixed points in trajectory (costs don't contribute), N+1 x D.
  Eigen::MatrixXd e(traj.N - 1, params_.D);
  e.setZero();
  e.topRows<1>() = -traj.start;
  e.bottomRows<1>() = traj.goal;

  // Values used for cost computation.
  A_.resize(traj.N - 2, traj.N - 2);
  A_ = K.transpose() * K;
  b_ = K.transpose() * e;
  c_ = 0.5 * (e.transpose() * e);
}

double SmoothnessFunc::getCost(const ChompTrajectory& traj) const {
  // Not clear what to do here: MATLAB implementation multiplies by N here.
  // From derivation, seems unnecessary, but I guess it normalizes by
  // trajectory size. Could be rolled into A, b, c, as well.

  // Will take straight from MATLAB implementation.
  // There, N-1 is used?

  // Multiply each dimension by A.

  double cost =
      (traj.N - 1) *
      (0.5 * traj.trajectory.block(1, 0, traj.N - 2, params_.D).transpose() *
           A_ * traj.trajectory.block(1, 0, traj.N - 2, params_.D) +
       traj.trajectory.block(1, 0, traj.N - 2, params_.D).transpose() * b_ + c_)
          .trace();
  return cost;
}

void SmoothnessFunc::getGradient(const ChompTrajectory& traj,
                                 Eigen::MatrixXd* gradient) const {
  CHECK_NOTNULL(gradient);
  *gradient = (traj.N - 1) *
              (A_ * traj.trajectory.block(1, 0, traj.N - 2, params_.D) + b_);
}

double CollisionFunc::potentialFunction(double distance) const {
  double result = 0.0;
  if (distance < 0) {
    result = -distance + 0.5 * params_.epsilon;
  } else if (distance <= params_.epsilon) {
    double epsilon_distance = distance - params_.epsilon;
    result = 0.5 * 1.0 / params_.epsilon * epsilon_distance * epsilon_distance;
  } else {
    result = 0.0;
  }
  // std::cout << "Distance: " << distance << " Potential: " << result <<
  // std::endl;
  return result;
}

double CollisionFunc::getCost(const ChompTrajectory& traj) const {
  // Just do it one by one per unit...
  double total_cost = 0.0;
  double last_distance = distance_function_(traj.trajectory.row(0));
  double last_potential = potentialFunction(last_distance);

  for (int i = 1; i < traj.trajectory.rows() - 1; ++i) {
    // Get the distance at this position in the trajectory.
    double distance = distance_function_(traj.trajectory.row(i));
    double potential = potentialFunction(distance);

    // Now scale by workspace velocity.
    Eigen::VectorXd velocity = traj.getVelocity(i);

    // paper says average together, MATLAB says use just one...
    double cost =
        0.5 / (traj.N - 1) * velocity.norm() * (potential + last_potential);
    // double cost = 0.5 / (traj.N - 1) * velocity.norm() * potential;
    total_cost += cost;

    last_distance = distance;
    last_potential = potential;
  }
  return total_cost;
}

void CollisionFunc::getGradient(const ChompTrajectory& traj,
                                Eigen::MatrixXd* gradient) const {
  // Gradient timeeee!
  // Using the chain rule here, grad C = dC(d)/dd * dd(q)/dq
  CHECK_NOTNULL(gradient);
  gradient->resize(traj.N - 2, params_.D);
  gradient->setZero();

  for (int i = 1; i < traj.trajectory.rows() - 1; ++i) {
    Eigen::VectorXd dc_dq(params_.D);
    double d = distance_function_(traj.trajectory.row(i));
    double c = potentialFunction(d);

    Eigen::VectorXd distance_gradient(params_.D);

    Eigen::VectorXd increment(params_.D);
    // Get numeric derivative for each dim.
    for (int j = 0; j < params_.D; ++j) {
      // Gradient computations for a single dimension.
      increment.setZero();
      increment(j) = params_.map_resolution;
      double left_distance = potentialFunction(
          distance_function_(traj.trajectory.row(i) - increment.transpose()));
      double right_distance = potentialFunction(
          distance_function_(traj.trajectory.row(i) + increment.transpose()));
      double dim_gradient =
          (right_distance - left_distance) / (2.0 * params_.map_resolution);
      distance_gradient(j) = dim_gradient;
    }

    dc_dq = distance_gradient;

    // Now compute the actual gradient.
    Eigen::VectorXd velocity = traj.getVelocity(i);
    Eigen::VectorXd acceleration = traj.getAcceleration(i);

    Eigen::MatrixXd P(params_.D, params_.D), K(params_.D, params_.D);
    P = Eigen::MatrixXd::Identity(params_.D, params_.D) -
        velocity * velocity.transpose() / (velocity.norm() * velocity.norm());
    K = (P * acceleration) / velocity.squaredNorm();

    Eigen::MatrixXd local_gradient = velocity.norm() * (P * dc_dq - K * c);
    gradient->row(i - 1) = 1.0 / traj.N * local_gradient.transpose();
  }
}

void ChompOptimizer::createTrajectory(const Eigen::VectorXd& start,
                                      const Eigen::VectorXd& goal, int N,
                                      ChompTrajectory* traj) const {
  CHECK_NOTNULL(traj);
  CHECK_EQ(start.size(), goal.size());
  CHECK_EQ(start.size(), params_.D);
  traj->N = N;
  traj->start = start;
  traj->goal = goal;
  traj->trajectory.resize(N, params_.D);

  for (Eigen::Index i = 0; i < start.size(); i++) {
    traj->trajectory.col(i).setLinSpaced(N, start(i), goal(i));
  }
}

// Returns cost.
void ChompOptimizer::getGradient(const ChompTrajectory& traj,
                                 Eigen::MatrixXd* gradient) const {
  CHECK_NOTNULL(gradient);
  if (!f_smooth_.isInitialized()) {
    std::cout << "Cost functions uninitialized!\n";
    return;
  }

  Eigen::MatrixXd grad_smooth, grad_collision;
  {
    ZoneScopedN("chomp/gradient_smooth");
    f_smooth_.getGradient(traj, &grad_smooth);
  }

  if (f_collision_.isInitialized()) {
    ZoneScopedN("chomp/gradient_coll");
    f_collision_.getGradient(traj, &grad_collision);
  } else {
    grad_collision.resize(grad_smooth.rows(), grad_smooth.cols());
    grad_collision.setZero();
  }

  *gradient =
      params_.w_smooth * grad_smooth + params_.w_collision * grad_collision;
}

double ChompOptimizer::getCost(const ChompTrajectory& traj) const {
  if (!f_smooth_.isInitialized()) {
    std::cout << "Cost functions uninitialized!\n";
    return -1.0;
  }
  double cost_smooth{};
  {
    ZoneScopedN("chomp/cost_smooth");
    cost_smooth = f_smooth_.getCost(traj);
  }

  double cost_collision = 0.0;
  if (f_collision_.isInitialized()) {
    ZoneScopedN("chomp/cost_collision");
    cost_collision = f_collision_.getCost(traj);
  }
  double cost =
      params_.w_smooth * cost_smooth + params_.w_collision * cost_collision;

  return cost;
}

void ChompOptimizer::solveProblem(const Eigen::VectorXd& start,
                                  const Eigen::VectorXd& goal, int N,
                                  ChompTrajectory* solution) {
  // TODO(helenol): verify inputs!!!! That they match ChompParams.
  ChompTrajectory traj;
  createTrajectory(start, goal, N, &traj);
  setupFromTrajectory(traj);

  doGradientDescent(&traj);
  *solution = traj;
}

void ChompOptimizer::setupFromTrajectory(const ChompTrajectory& traj) {
  f_smooth_.initFromTrajectory(traj);
  f_collision_.initFromTrajectory(traj);

  // Smoothing function -> M
  M_ = f_smooth_.getA();
}

void ChompOptimizer::doGradientDescent(ChompTrajectory* traj) {
  ZoneScopedN("chomp/optimize");

  // TODO: fill in M.
  // Get initial cost and gradient.
  double cost = getCost(*traj);
  double prev_cost = cost;
  Eigen::MatrixXd gradient;
  // TODO(helenol): don't need to do this twice.
  getGradient(*traj, &gradient);
  if (params_.verbose) {
    std::cout << "[GD] Initial cost: " << cost << std::endl;
  }

  int i = 0;

  for (i = 0; i < params_.max_iter; ++i) {
    cost = getCost(*traj);
    getGradient(*traj, &gradient);

    // TODO: implement decreased weighting.
    double step_size = 1.0 / params_.lambda;
    if (params_.decrease_step_size) {
      step_size *= 1.0 / std::sqrt(i + 1);
    }

    Eigen::MatrixXd traj_increment(traj->trajectory.rows(),
                                   traj->trajectory.cols());

    traj_increment.setZero();
    // M \ grad

    traj_increment.block(1, 0, traj->N - 2, params_.D) =
        M_.ldlt().solve(gradient);
    // Don't increment start or end.
    // TODO(helenol): double check, but I think this is taken care by the
    // structure of the matrix now...
    // traj_increment.topRows<1>().setZero();
    // traj_increment.bottomRows<1>().setZero();

    traj->trajectory -= step_size * traj_increment;

    if (params_.verbose && i % 1 == 0) {
      std::cout << "[GD] i: " << i << " step size: " << step_size
                << " cost: " << cost << " gradient norm: " << gradient.norm()
                << std::endl;
    }

    // Check for tolerance - minimum number of iters is 5 though.
    if (i > 5) {
      // Use this formulation to avoid numerical issues near 0.
      double rel_error = fabs(cost - prev_cost) / (prev_cost + 1);
      if (rel_error < params_.rel_tol) {
        if (params_.verbose) {
          std::cout << "[GD][Finished][Tol] Iter " << i
                    << " finished with cost " << cost << " with rel_error "
                    << rel_error << std::endl;
        }
        break;
      }
      prev_cost = cost;
    }
  }

  if (params_.verbose) {
    std::cout << "[GD][Finished][Iter] Final cost after " << i
              << " iterations: " << cost << std::endl;
  }
}

}  //  namespace chomp
