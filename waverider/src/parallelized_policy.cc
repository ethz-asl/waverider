#include "waverider/parallelized_policy.h"

namespace waverider {
ParallelizedPolicy::ParallelizedPolicy(uint num_policies, PolicyTuning tuning)
    : tuning_(tuning) {
  f.setZero(Eigen::NoChange_t::NoChange, num_policies);
  A.setZero(Eigen::NoChange_t::NoChange, num_policies);
  num_policies_ = num_policies;
}

void ParallelizedPolicy::init(const std::vector<Eigen::Vector3f>& x_obs,
                              const Eigen::Vector3f& x,
                              const Eigen::Vector3f& xdot) {
  const float r = tuning_.r;
  const float nu_rep = tuning_.nu_rep;
  const float eta_rep = tuning_.eta_rep;

  const float nu_damp = tuning_.nu_damp;
  const float eta_damp = tuning_.eta_damp;

  for (uint i = 0; i < num_policies_; ++i) {
    // normalize gradient
    Eigen::Vector3f grad_d = x - x_obs[i];
    const double d_x = std::max(grad_d.norm(), 0.0001f);
    grad_d.normalize();

    // calculate repulsive part (f_rep)
    const double alpha_rep = eta_rep * exp(-(d_x / nu_rep));
    const Eigen::Vector3f f_rep = alpha_rep * grad_d;

    // calculate dampening part (f_damp)
    const double epsilon = 1e-6;  // Added term for numerical stability
    const double alpha_damp = eta_damp / ((d_x / nu_damp) + epsilon);

    // eq 68 in RMP paper
    const Eigen::Vector3f P_obs_x = std::max(0.0f, -xdot.dot(grad_d)) *
                                    (grad_d * grad_d.transpose()) * (xdot);
    const Eigen::Vector3f f_damp = alpha_damp * P_obs_x;

    // set f
    Eigen::Vector3f f_temp{Eigen::Vector3f::Zero()};
    if (tuning_.enable_repulsor) {
      f_temp += f_rep;
    }
    if (tuning_.enable_damper) {
      f_temp -= f_damp;
    }

    f.col(i) = f_temp;

    // calculation of metric
    // transpose should work outside s() as well
    const Eigen::Vector3f v = s(f_temp);
    Eigen::Matrix3f A_temp =
        wr(static_cast<float>(d_x), r) * (v * v.transpose());
    Eigen::Map<Eigen::Matrix<float, 9, 1>> A_temp_v(A_temp.data(),
                                                    A_temp.size());

    // set A
    A.col(i) = A_temp_v;
    A_sum += A_temp;
    Af_sum += (A_temp * f_temp);
  }
}
}  // namespace waverider
