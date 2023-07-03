//
// Created by mpantic on 06.06.23.
//

#ifndef MAV_REACTIVE_PLANNING_WAVEMAP_AVOIDANCE_H
#define MAV_REACTIVE_PLANNING_WAVEMAP_AVOIDANCE_H
#include <mav_reactive_planning/policies/world/wavemap/wavemap_policy_filter.h>
#include <rmpcpp/core/policy_value.h>
#include <ros/ros.h>

struct PolicyTuning{
  float r {0.1};
  float c {0.1};
  float eta_rep {80}; // n
  float nu_rep {1}; // v
  float eta_damp  {14}; // n
  float nu_damp  {1}; // v
  bool enable_damper = true;
  bool enable_repulsor = true;
};

class ParallelizedPolicy{
 public:

  ParallelizedPolicy(uint num_policies, PolicyTuning tuning) : tuning_(tuning){
    f.resize( Eigen::NoChange_t::NoChange,num_policies);
    A.resize( Eigen::NoChange_t::NoChange,num_policies);
    num_policies_  = num_policies;
  }

  void init(const std::vector<Eigen::Vector3f> x_obs, const Eigen::Vector3f x, const Eigen::Vector3f xdot, double size=0, double level=0){

    float r = tuning_.r;
    float nu_rep = tuning_.nu_rep;
    float eta_rep = tuning_.eta_rep;

    float nu_damp = tuning_.nu_damp;
    float eta_damp = tuning_.eta_damp;
    bool debug_output = false;

    if(debug_output) std::cout << "r " << r << std::endl;
    if(debug_output) std::cout << "nu_rep" << nu_rep << std::endl;
    if(debug_output) std::cout << "eta_rep" << eta_rep << std::endl;
    //nu_rep_ = 5*r_;
    // get f_rep
    for(uint i = 0; i< num_policies_; ++i ){

      // normalize gradient
      Eigen::Vector3f grad_d = x-x_obs[i];
      double d_x = std::max(grad_d.norm(),0.0001f);
      if(debug_output) std::cout << "d_x " << d_x << std::endl;
      if(debug_output) std::cout << "grad_d " << grad_d.transpose() << std::endl;

      grad_d.normalize();

      // calculate repulsive part (f_rep)
      double alpha_rep = eta_rep * exp(-(d_x / nu_rep));
      Eigen::Vector3f f_rep = alpha_rep * grad_d;

      // calculate dampening part (f_damp)
      double epsilon = 1e-6;  // Added term for numerical stability
      double alpha_damp = eta_damp / ((d_x / nu_damp) + epsilon);  // 1e-6 is

      // eq 68 in RMP paper
      Eigen::Vector3f P_obs_x = std::max(0.0f, -xdot.dot(grad_d)) *
                     (grad_d * grad_d.transpose()) * (xdot);
      Eigen::Vector3f f_damp = alpha_damp * P_obs_x;

      // set f
     Eigen::Vector3f f_temp {Eigen::Vector3f::Zero()};
     if(tuning_.enable_repulsor){
       if(debug_output) std::cout << "f_rep " << f_rep.transpose() << std::endl;
       f_temp += f_rep;
     }
     if(tuning_.enable_damper){
       f_temp -= f_damp;
       if(debug_output) std::cout << "f_damp " << -f_damp.transpose() << std::endl;
     }

     f.col(i) = f_temp;

      // calculation of metric
      // transpose should work outside of s() as well
      Eigen::Vector3f v = s(f_temp);
      Eigen::Matrix3f A_temp = wr(d_x, r) *( v * v.transpose()); /// num_policies_ ;
      Eigen::Map<Eigen::Matrix<float,9,1>> A_temp_v(A_temp.data(), A_temp.size());

      // set A
      A.col(i) = A_temp_v;
      A_sum += A_temp;
      Af_sum += (A_temp * f_temp);
    }
  }

  rmpcpp::PolicyValue<3> getPolicy(){
    return {(A_sum.completeOrthogonalDecomposition().pseudoInverse() * Af_sum).cast<double>(),
        A_sum.cast<double>()};
  }

  inline Eigen::Vector3f s(Eigen::Vector3f x) { return x / h(x.norm()); }

  /**
   * Softmax helper function
   */
  inline float h(const float z) {
    return (z + tuning_.c * log(1 + exp(-2 * tuning_.c * z)));
  }

  inline float wr(float s, float r) {
    double c2 = 1 / (r * r);
    double c1 = -2 / r;
    return (c2 * s * s) + (c1 * s) + 1.0;
  }

  void setR(float r){

  }
 private:
  PolicyTuning tuning_;

  uint num_policies_;
  Eigen::Vector3f Af_sum {Eigen::Vector3f::Zero()};
  Eigen::Matrix3f A_sum{Eigen::Matrix3f::Zero()};
  Eigen::Matrix<float, 3,  Eigen::Dynamic> f;
  Eigen::Matrix<float, 9, Eigen::Dynamic> A;

};

/*
class WavemapAvoidance{
 public:
  WavemapAvoidance(){
  }

  void setFilter(double value){
    filter_val_ = value;
  }

  void run(std::shared_ptr<wavemap::HashedWaveletOctree> map, Eigen::Vector3f x, Eigen::Vector3f xdot){
    WavemapPolicyFilter fltr;
    fltr.updateFilter(filter_val_);
    fltr.updateMap(map);

    Eigen::Vector3f robot_pos = {20.01,-2.54,0.1};
    Eigen::Vector3f robot_vel = {0.0,0.0,0.0};
    fltr.updateRobotPosition(robot_pos);

    std::cout << "Map loaded.. ."  << std::endl;

    fltr.generatePolicies();
    auto policy_cells = fltr.getPolicyCells();

    std::cout << "Map parsed.. ."  << std::endl;


    // get number of policies
    uint num_policies = 0;
    for(const auto& block : policy_cells.blocks){
      num_policies += block.size();
    }

    // create policies for level 1 for test
    ParallelizedPolicy pol(policy_cells.blocks[1].size());
    pol.setR(policy_cells.block_size[1]*3);
    std::cout << policy_cells.blocks[1].size() << std::endl;
    pol.init(policy_cells.blocks[1], robot_pos, robot_vel);
    auto policy = pol.getPolicy();


    std::cout << policy.f_.transpose() <<std::endl;


    // generate diff d(x)



  }
  double filter_val_{-0.19};

};*/

#endif  // MAV_REACTIVE_PLANNING_WAVEMAP_AVOIDANCE_H
