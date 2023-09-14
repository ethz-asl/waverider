#ifndef CHOMP_ROS_CHOMP_OPTIMIZER_H_
#define CHOMP_ROS_CHOMP_OPTIMIZER_H_

#include <Eigen/Core>
#include <glog/logging.h>

// NOTE: This CHOMP implementation is borrowed from Helen's mav_local_avoidance:
//       https://github.com/ethz-asl/mav_local_avoidance/blob/master/local_trajectory_opt/include/local_trajectory_opt/chomp/chomp_optimizer.h
namespace chomp {
struct ChompParameters {
 public:
  ChompParameters()
      : D(3),
        w_smooth(1.0),
        w_collision(5.0),
        epsilon(1.0),
        lambda(10.0),  // Suggested value: 1000?
        rel_tol(1e-5),
        max_iter(100),
        decrease_step_size(false),
        map_resolution(0.015),
        verbose(true) {}

  int D;  // Number of dimensions in the trajectory. Trajectories are N x D.

  // Math parameters.
  double w_smooth;     // f_smooth weight. (lambda in MATLAB).
  double w_collision;  // f_collision weight.

  double epsilon;  // Radius of robot sphere.

  // Optimization parameters.
  double lambda;  // Step size of gradient descent (eta in MATLAB).

  double rel_tol;  // Relative tolerance on cost before optimization terminates.
  int max_iter;    // Maximum number of iterations.
  bool decrease_step_size;  // Decreases step size by sqrt(i) each iteration.

  // Setup parameters.
  double map_resolution;  // In metric units. This is to estimate the gradient
  // of the collision potential function.

  // TODO(helenol): add weighing parameters.
  bool verbose;
};

struct ChompTrajectory {
 public:
  ChompTrajectory() : N(0) {}
  virtual ~ChompTrajectory() {}

  // Full trajectory including start and goal.
  Eigen::MatrixXd trajectory;
  Eigen::VectorXd start;
  Eigen::VectorXd goal;
  int N;  // Number of elements in discretized trajectory.

  // Inner trajectory, without start or goal.
  Eigen::MatrixXd innerTraj() const {
    Eigen::MatrixXd copy = trajectory.block(1, 0, N - 2, trajectory.cols());
    return copy;
  }

  void clear() {
    N = 0;
    trajectory.resize(0, 0);
    start.resize(0, 0);
    goal.resize(0, 0);
  }

  // Convenience functions for derivates.
  Eigen::VectorXd getVelocity(int row) const {
    if (row == 0) {
      // Forward differentiation at first points
      return N * (trajectory.row(row + 1) - trajectory.row(row)) / 2.0;
    } else if (row == trajectory.rows() - 1) {
      // Backwards at last point
      return N * (trajectory.row(row) - trajectory.row(row - 1)) / 2.0;
    } else {
      // Middle everywhere else
      return N * (trajectory.row(row + 1) - trajectory.row(row - 1)) / 2.0;
    }
  }

  // Acceleration assumes N >= 2, which kind of makes sense anyway.
  Eigen::VectorXd getAcceleration(int row) const {
    if (row == 0) {
      // Forward differentiation at first points
      return N * N *
             (trajectory.row(row + 2) - 2 * trajectory.row(row + 1) +
              trajectory.row(row));
    } else if (row == trajectory.rows() - 1) {
      // Backwards at last point
      return N * N *
             (trajectory.row(row - 2) - 2 * trajectory.row(row - 1) +
              trajectory.row(row));
    } else {
      // Middle everywhere else
      return N * N *
             (trajectory.row(row + 1) - 2 * trajectory.row(row) +
              trajectory.row(row - 1));
    }
  }
};

class ChompCostFunc {
 public:
  virtual void initFromTrajectory(const ChompTrajectory& /*traj*/) {}

  virtual double getCost(const ChompTrajectory& /*traj*/) const { return 0.0; }
  virtual void getGradient(const ChompTrajectory& traj,
                           Eigen::MatrixXd* gradient) const {
    CHECK_NOTNULL(gradient);
    gradient->resize(traj.N - 2, params_.D);
    gradient->setZero();
  }

  void setParameters(const ChompParameters& params) { params_ = params; }

  virtual bool isInitialized() const { return true; }

 protected:
  ChompParameters params_;
};

class SmoothnessFunc : public ChompCostFunc {
 public:
  virtual void initFromTrajectory(const ChompTrajectory& traj);

  virtual double getCost(const ChompTrajectory& traj) const;
  virtual void getGradient(const ChompTrajectory& traj,
                           Eigen::MatrixXd* gradient) const;

  // Check that everything is set up correctly to use for evaluating costs.
  virtual bool isInitialized() const {
    return A_.rows() != 0 && A_.cols() != 0;
  }

  // For debugging.
  Eigen::MatrixXd getA() { return A_; }
  Eigen::MatrixXd getb() { return b_; }
  Eigen::MatrixXd getc() { return c_; }

 private:
  Eigen::MatrixXd A_;
  Eigen::MatrixXd b_;
  Eigen::MatrixXd c_;
};

class CollisionFunc : public ChompCostFunc {
 public:
  typedef std::function<double(const Eigen::VectorXd& position)>
      DistanceFunctionType;
  typedef std::function<Eigen::VectorXd(const Eigen::VectorXd& position)>
      DistanceGradientFunctionType;

  virtual double getCost(const ChompTrajectory& traj) const;
  virtual void getGradient(const ChompTrajectory& traj,
                           Eigen::MatrixXd* gradient) const;

  // Set the function that will get the distance between a current point
  // and the nearest obstacle (i.e., query a distance map).
  void setDistanceFunction(const DistanceFunctionType& function) {
    distance_function_ = function;
  }

  // Check that everything is set up correctly to use for evaluating costs.
  virtual bool isInitialized() const { return distance_function_ != nullptr; }

  // Convenience methods for the rest.
  // Computes the potential function (Fig. 2 in conference paper).
  double potentialFunction(double distance) const;
  double getDistance(const Eigen::VectorXd& pos) const {
    return distance_function_(pos);
  }

 protected:
  DistanceFunctionType distance_function_;
};

class ChompOptimizer {
 public:
  void setParameters(const ChompParameters& params) {
    params_ = params;
    f_smooth_.setParameters(params_);
    f_collision_.setParameters(params_);
  }

  void solveProblem(const Eigen::VectorXd& start, const Eigen::VectorXd& goal,
                    int N, ChompTrajectory* solution);

  // Setup.
  void setupFromTrajectory(const ChompTrajectory& traj);
  void setDistanceFunction(
      const CollisionFunc::DistanceFunctionType& function) {
    f_collision_.setDistanceFunction(function);
  }

  // Convenience functions.
  void createTrajectory(const Eigen::VectorXd& start,
                        const Eigen::VectorXd& goal, int N,
                        ChompTrajectory* traj) const;
  void getGradient(const ChompTrajectory& traj,
                   Eigen::MatrixXd* gradient) const;
  double getCost(const ChompTrajectory& traj) const;

  // Actual optimization.
  void doGradientDescent(ChompTrajectory* traj);

  // For testing.
  void setM(const Eigen::MatrixXd& M) { M_ = M; }
  Eigen::MatrixXd getM() { return M_; }

 private:
  ChompParameters params_;

  // Is this necessary to store?
  ChompTrajectory traj_;

  // Cost functions.
  SmoothnessFunc f_smooth_;
  CollisionFunc f_collision_;

  // Math.
  Eigen::MatrixXd M_;
};

}  //  namespace chomp

#endif  // CHOMP_ROS_CHOMP_OPTIMIZER_H_
