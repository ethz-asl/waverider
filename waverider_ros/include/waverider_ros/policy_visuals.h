#ifndef WAVERIDER_ROS_POLICY_VISUALS_H_
#define WAVERIDER_ROS_POLICY_VISUALS_H_

#include <string>
#include <vector>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <waverider/obstacle_filter.h>

namespace waverider {


void addFilteredObstaclesToMarkerArray(
    const ObstacleCells& policy_blocks, const std::string& world_frame,
    visualization_msgs::MarkerArray& marker_array);


visualization_msgs::Marker generateClearingMarker();

visualization_msgs::Marker robotPositionToMarker(
    const Eigen::Vector3f& robot_pos, const std::string& world_frame);

visualization_msgs::Marker filteredObstacleLevelToMarker(
    int lvl, double size, const std::vector<Eigen::Vector3f>& obstacle_centers,
    const std::string& world_frame);
}  // namespace waverider

#endif  // WAVERIDER_ROS_POLICY_VISUALS_H_
