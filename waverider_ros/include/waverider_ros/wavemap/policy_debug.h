//
// Created by mpantic on 09.06.23.
//

#ifndef MAV_REACTIVE_PLANNING_POLICY_DEBUG_H
#define MAV_REACTIVE_PLANNING_POLICY_DEBUG_H

#include <mav_reactive_planning/policies/world/wavemap/wavemap_policy_filter.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>

class PolicyDebug{
 public:
  PolicyDebug(ros::NodeHandle nh):nh_(nh){
    pub_debug_ = nh.advertise<visualization_msgs::MarkerArray>("/waverider/debug",10);
  }

  void visualize(const PolicyCells& policy_blocks, Eigen::Vector3f robot_pos){
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
    mrkr_r.header.frame_id = "map";
    mrkr_r.action = visualization_msgs::Marker::SPHERE;
    mrkr_r.action = visualization_msgs::Marker::ADD;
    mrkr_r.scale.x = 0.25;
    mrkr_r.scale.y = 0.25;
    mrkr_r.scale.z = 0.25;
    mrkr_r.color.r = 1.0;
    mrkr_r.color.a = 1.0;
    mrkr_array.markers.push_back(mrkr_r);
    pub_debug_.publish(mrkr_array);
  }

  visualization_msgs::Marker createPointCloudForLevel(uint lvl, double size, std::vector<Eigen::Vector3f> midpoints){
    visualization_msgs::Marker mrkr;
    mrkr.header.frame_id = "map";
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
    mrkr.color.a = 0.2;
    mrkr.pose.orientation.w = 1.0;
    mrkr.points.resize(midpoints.size());
    for(uint i=0 ;i<midpoints.size();++i){
      mrkr.points[i].x = midpoints[i].x();
      mrkr.points[i].y = midpoints[i].y();
      mrkr.points[i].z = midpoints[i].z();
    }
    return mrkr;
  }


 private:
  ros::NodeHandle nh_;
  ros::Publisher pub_debug_;

};

#endif  // MAV_REACTIVE_PLANNING_POLICY_DEBUG_H
