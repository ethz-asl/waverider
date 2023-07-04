#include "waverider_ros/policy_visuals.h"

namespace waverider {
visualization_msgs::Marker robotPositionToMarker(
    const Eigen::Vector3f& robot_pos) {
  visualization_msgs::Marker marker;

  marker.pose.orientation.w = 1.0;
  marker.pose.position.x = robot_pos.x();
  marker.pose.position.y = robot_pos.y();
  marker.pose.position.z = robot_pos.z();
  marker.id = 100;
  marker.ns = "robot";
  marker.header.frame_id = "odom";
  marker.action = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.6;
  marker.scale.y = 0.6;
  marker.scale.z = 0.4;
  marker.color.r = 1.0;
  marker.color.a = 1.0;

  return marker;
}

visualization_msgs::MarkerArray filteredObstaclesToMarkerArray(
    const waverider::ObstacleCells& policy_blocks) {
  visualization_msgs::MarkerArray marker_array;

  // Add a marker for each resolution level
  const auto num_levels = static_cast<int>(policy_blocks.cell_widths.size());
  for (int i = 0; i < num_levels; i++) {
    if (policy_blocks.centers[i].empty()) {
      continue;
    }
    marker_array.markers.emplace_back(filteredObstacleLevelToPointcloudMarker(
        i, policy_blocks.cell_widths[i], policy_blocks.centers[i]));
  }

  return marker_array;
}

visualization_msgs::Marker filteredObstacleLevelToPointcloudMarker(
    int lvl, double size,
    const std::vector<Eigen::Vector3f>& obstacle_centers) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "odom";
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = lvl;
  marker.ns = "level_" + std::to_string(lvl);
  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;
  marker.color.r = 0.0;
  marker.color.b = 1.0;
  marker.color.g = 0.0;
  marker.color.a = 0.2;
  marker.pose.orientation.w = 1.0;

  marker.points.resize(obstacle_centers.size());
  for (size_t i = 0; i < obstacle_centers.size(); ++i) {
    marker.points[i].x = obstacle_centers[i].x();
    marker.points[i].y = obstacle_centers[i].y();
    marker.points[i].z = obstacle_centers[i].z();
  }

  return marker;
}
}  // namespace waverider
