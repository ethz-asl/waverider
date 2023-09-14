#include "waverider_eval/waverider_evaluator.h"
#include <wavemap_io/file_conversions.h>
#include <wavemap_ros_conversions/map_msg_conversions.h>

#include <omav_msgs/conversions.h>
#include <omav_msgs/eigen_omav_msgs.h>
#include <rmpcpp/geometry/partial_geometry.h>
#include <tracy/Tracy.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <wavemap/config/param.h>
#include <wavemap_ros_conversions/config_conversions.h>
#include <wavemap/utils/stopwatch.h>
#include <wavemap_msgs/Map.h>

#include "waverider_ros/policy_visuals.h"

namespace waverider {
DECLARE_CONFIG_MEMBERS(WaveriderEvaluatorConfig,
                      (hans));

bool WaveriderEvaluatorConfig::isValid(bool verbose) const {
  bool all_valid = true;

  all_valid &= IS_PARAM_NE(hans, std::string(""), verbose);

  return all_valid;
}

void WaveriderEvaluator::publishState(Eigen::Vector3d pos, Eigen::Vector3d vel){
    nav_msgs::Odometry msg;
    msg.header.frame_id = "map";
    msg.child_frame_id = "rmp_state";
    msg.pose.pose.position.x = pos.x();
    msg.pose.pose.position.y = pos.y();
    msg.pose.pose.position.z = pos.z();

    debug_pub_odom_.publish(msg);

}
WaveriderEvaluator::WaveriderEvaluator(const WaveriderEvaluatorConfig& config, bool only_highest_res)
    : config_(config.checkValid()), only_highest_res_(only_highest_res) {
  ros::NodeHandle nh;
  debug_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
      "filtered_obstacles", 1);

  debug_pub_odom_ = nh.advertise<nav_msgs::Odometry>(
      "rmp_state", 1);

  map_pub_ = nh.advertise<wavemap_msgs::Map>("map", 1, true);
}

void WaveriderEvaluator::loadMap(std::string path) {
  wavemap::VolumetricDataStructureBase::Ptr map_ptr;
  if(!wavemap::io::fileToMap(path, map_ptr)){
    LOG(FATAL) << "MAP NOT LOADED";
  }
  map_ = std::dynamic_pointer_cast<wavemap::HashedWaveletOctree>(map_ptr);
  LOG(INFO) << "Map " << path << " loaded.";

  wavemap_msgs::Map  map_msg;
  wavemap::convert::mapToRosMsg(*map_ptr, "map", ros::Time::now(), map_msg);
  map_pub_.publish(map_msg);
}

WaveriderEvaluator::Result WaveriderEvaluator::plan(Eigen::Vector3d start,
                                                    Eigen::Vector3d end) {
  // rest to rest trajectories!
  rmpcpp::State<3> start_r3;
  start_r3.pos_ = start;
  start_r3.vel_ = Eigen::Vector3d::Zero();
  start_r3.acc_ = Eigen::Vector3d::Zero();

  // configure policies
  rmpcpp::SimpleTargetPolicy<rmpcpp::Space<3>> target_policy;
  target_policy.setTuning(10, 15, 0.01);
  target_policy.setTarget(end);
  target_policy.setA(Eigen::Matrix3d::Identity()*10);

  WaveriderPolicy waverider_policy;
  if(only_highest_res_){
    waverider_policy.run_all_levels_ = false;
  }
  std::vector<Eigen::Vector3d> trajectory;

  rmpcpp::TrapezoidalIntegrator<rmpcpp::State<3>> integrator(start_r3, 0.01);
  Eigen::Vector3d last_updated_pos = {-10000.0, -10000.0, -10000.0};
  int i =0;
  // lambda to make victor happy
  // tiny bit more efficient -> victor only slightly angry/disappointed.
  auto policy_sum = [&](const rmpcpp::State<3>& state) {

    trajectory.push_back(state.pos_);

    // update obstacles at current position




    // sum policies
    if((last_updated_pos - state.pos_).norm() > 0.1) {
     waverider_policy.updateObstacles(*map_, state.pos_.cast<float>());
     //visualization_msgs::MarkerArray marker_array;
      //addFilteredObstaclesToMarkerArray(waverider_policy.getObstacleCells(),"map", marker_array);
     // debug_pub_.publish(marker_array);
      last_updated_pos = state.pos_;
    }


    publishState(state.pos_, state.vel_);


    auto waverider_result =waverider_policy.evaluateAt(state);
    auto target_result = target_policy.evaluateAt(state);



    // return
    return (target_result+waverider_result).f_;
  };



  WaveriderEvaluator::Result planning_result;
  wavemap::Stopwatch watch;
  watch.start();
  bool got_to_rest =
      integrator.integrateSteps(policy_sum, max_integration_steps_);
  watch.stop();

  planning_result.success = got_to_rest;
  planning_result.states_out = trajectory;

  planning_result.duration = std::chrono::duration_cast< std::chrono::steady_clock::duration>(std::chrono::duration<double>(watch.getLastEpisodeDuration()));
  return planning_result;
}

}  // namespace waverider
