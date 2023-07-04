#ifndef WAVERIDER_ROS_POLICY_VISUALS_H_
#define WAVERIDER_ROS_POLICY_VISUALS_H_

#include <vector>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <waverider/obstacle_filter.h>

namespace waverider {
visualization_msgs::Marker robotPositionToMarker(
    const Eigen::Vector3f& robot_pos);

visualization_msgs::MarkerArray filteredObstaclesToMarkerArray(
    const ObstacleCells& policy_blocks);

visualization_msgs::Marker filteredObstacleLevelToPointcloudMarker(
    int lvl, double size, const std::vector<Eigen::Vector3f>& obstacle_centers);
}  // namespace waverider

#endif  // WAVERIDER_ROS_POLICY_VISUALS_H_
