//
// Created by mpantic on 28.06.23.
//

#ifndef MAV_REACTIVE_PLANNING_WAVEMAP_POLICY_H
#define MAV_REACTIVE_PLANNING_WAVEMAP_POLICY_H
#include <gtest/gtest.h>
#include <wavemap/config/config_base.h>
#include <wavemap/test/config_generator.h>

#include "mav_reactive_planning/policies/world/wavemap_avoidance.h"
#include "mav_reactive_planning/policies/world/wavemap/policy_debug.h"
#include <rmpcpp/core/policy_base.h>
#include <wavemap_ros_conversions/map_msg_conversions.h>
#include <rmpcpp/core/state.h>
#include <wavemap_msgs/Map.h>
#include <wavemap_io/file_conversions.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rmpcpp/eval/integrator.h>

using namespace std::chrono;
#include <ros/ros.h>

class WaveriderPolicy : public rmpcpp::PolicyBase<rmpcpp::Space<3>>{
 public:
  WaveriderPolicy(){

   // pub_map_ = nh_.advertise<wavemap_msgs::Map>("/wavemap/map",1, true);
    pub_debug_ = nh_.advertise<visualization_msgs::MarkerArray>("/waverider/debug", 1);
  }

  void updateTuning(PolicyTuning tuning){
    tuning_ = tuning;
  }

    void setRunAllLevels(bool run_all_levels){
    run_all_levels_ = run_all_levels;
  }

  void updateMap(std::shared_ptr<wavemap::VolumetricDataStructureBase> map){
    loaded_map_ = map;
    if(loaded_map_){
    policy_filter_.updateMap(std::static_pointer_cast<wavemap::HashedWaveletOctree>(loaded_map_));
    }
    //wavemap_msgs::Map map_msg;
    //wavemap::convert::mapToRosMsg(*map, "odom", ros::Time::now(), map_msg);
    //pub_map_.publish(map_msg);
  }

  virtual rmpcpp::PolicyValue<3> evaluateAt( const rmpcpp::State<3>& x) {
    if(!loaded_map_){
      return rmpcpp::PolicyValue<3>(Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero());
    }
    Eigen::Vector3f x_pos = x.pos_.cast<float>();
    Eigen::Vector3f x_vel = x.vel_.cast<float>();
    Eigen::Vector3f x_acc = x.acc_.cast<float>();

    // get all cells where we should attach a policy

    policy_filter_.updateRobotPosition(x_pos);
    policy_filter_.generatePolicies();
    auto policy_cells = policy_filter_.getPolicyCells();

    // create policies for one level now
    //policy_cells.blocks[0].clear();
    //policy_cells.blocks[2].clear();
    //policy_cells.blocks[3].clear();
    //policy_cells.blocks[4].clear();
    //policy_cells.blocks[5].clear();
    //policy_cells.blocks[6].clear();

    std::vector<rmpcpp::PolicyValue<3>> all_policies;
    for(int i=0; i<policy_cells.block_size.size(); i++) {
    //int i = 1;
    //policy_cells.blocks[i].resize(1);
    //policy_cells.blocks[i].at(0) << 0.0, 0.0, 0.5;

      if(i==1 || run_all_levels_) {
        //policy_cells.blocks[i].resize(1);
        //policy_cells.blocks[i].at(0) << 0.0, 0.0, 1.0;
        ParallelizedPolicy pol_generator(policy_cells.blocks[i].size(), tuning_);
        pol_generator.setR(policy_cells.block_size[i]);

        pol_generator.init(policy_cells.blocks[i], x_pos, x_vel, policy_cells.block_size[i], i);
        all_policies.push_back(pol_generator.getPolicy());
      }
    }

    auto avoidance_policy = rmpcpp::PolicyValue<3>::sum(all_policies);


    rmpcpp::PolicyValue<3> scaled_avoidance = {avoidance_policy.f_, avoidance_policy.A_};
    std::cout << scaled_avoidance.f_.transpose() << std::endl;
    std::cout << scaled_avoidance.A_ << std::endl;
    if(++steps_since_visualized_ == 10){
    visualize(policy_cells, x_pos);
    }
    return scaled_avoidance;
  /*
    rmpcpp::PolicyValue<3> manual_vel_policy {(manual_input_vel_ - x_vel_) - 0.5 * x_acc, Eigen::Matrix3d::Identity()};
    rmpcpp::PolicyValue<3> damper {-1*x_vel, Eigen::Matrix3d::Identity()};


    auto full_policy = scaled_avoidanec  + damper;

    rmpcpp::TrapezoidalIntegrator<rmpcpp::State<3>> integrator(x,0.01);
    integrator.integrateSteps(full_policy.f_, 5); // run at 20 hz

    std::cout << robot_state_.pos_ << std::endl;*/


  }


  void visualize(const PolicyCells& policy_blocks, Eigen::Vector3f robot_pos){
    steps_since_visualized_ =0;
    uint levels = policy_blocks.block_size.size();
    visualization_msgs::MarkerArray mrkr_array;

    for(uint i=0; i<levels; i++){
      if(policy_blocks.blocks[i].empty()){
        continue;
      }

      mrkr_array.markers.push_back(
          createPointCloudForLevel(i,
                                   policy_blocks.block_size[i],
                                   policy_blocks.blocks[i])
      );
    }

    // add robot position
    visualization_msgs::Marker mrkr_r;
    mrkr_r.pose.orientation.w =1.0;
    mrkr_r.pose.position.x = robot_pos.x();
    mrkr_r.pose.position.y = robot_pos.y();
    mrkr_r.pose.position.z = robot_pos.z();
    mrkr_r.id = 100;
    mrkr_r.ns = "robot";
    mrkr_r.header.frame_id = "odom";
    mrkr_r.type = visualization_msgs::Marker::SPHERE;
    mrkr_r.action = visualization_msgs::Marker::ADD;
    mrkr_r.scale.x = 0.6;
    mrkr_r.scale.y = 0.6;
    mrkr_r.scale.z = 0.4;
    mrkr_r.color.r = 1.0;
    mrkr_r.color.a = 1.0;
    mrkr_array.markers.push_back(mrkr_r);
    pub_debug_.publish(mrkr_array);
  }

  visualization_msgs::Marker createPointCloudForLevel(uint lvl, double size, std::vector<Eigen::Vector3f> midpoints){
    visualization_msgs::Marker mrkr;
    mrkr.header.frame_id = "odom";
    mrkr.type = visualization_msgs::Marker::CUBE_LIST;
    mrkr.action = visualization_msgs::Marker::ADD;
    mrkr.id = lvl;
    mrkr.ns = "level_" + std::to_string((int)lvl);
    mrkr.scale.x = size;
    mrkr.scale.y = size;
    mrkr.scale.z = size;
    mrkr.color.r = 0.0;
    mrkr.color.b = 1.0;
    mrkr.color.g = 0.0;
    mrkr.color.a = 1.0;
    mrkr.pose.orientation.w = 1.0;
    mrkr.points.resize(midpoints.size());
    for(uint i=0 ;i<midpoints.size();++i){
      mrkr.points[i].x = midpoints[i].x();
      mrkr.points[i].y = midpoints[i].y();
      mrkr.points[i].z = midpoints[i].z();
    }
    return mrkr;
  }

  void setFilter(double value){
    policy_filter_.updateFilter(value);
  }
 private:
  ros::NodeHandle nh_;
  //ros::Publisher pub_map_;
  int steps_since_visualized_ = 0;
  ros::Publisher pub_debug_;
  PolicyTuning tuning_;
  std::shared_ptr<wavemap::VolumetricDataStructureBase> loaded_map_;
  WavemapPolicyFilter policy_filter_;
  bool run_all_levels_{true};



};

#endif  // MAV_REACTIVE_PLANNING_WAVEMAP_POLICY_H
