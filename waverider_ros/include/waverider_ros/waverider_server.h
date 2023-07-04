#ifndef WAVERIDER_ROS_WAVERIDER_SERVER_H_
#define WAVERIDER_ROS_WAVERIDER_SERVER_H_

#include <thread>

#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <wavemap/data_structure/volumetric/volumetric_data_structure_base.h>
#include <waverider/waverider_policy.h>

namespace waverider {
class WaveriderServer {
 public:
  WaveriderServer(ros::NodeHandle nh, ros::NodeHandle nh_private)
      : waverider_policy_() {
    subscribeToTopics(nh);
    advertiseTopics(nh_private);
  }

  void updateMap(const wavemap::VolumetricDataStructureBase& map);

  void startPlanningAsync();
  void stopPlanningAsync() { continue_async_planning_ = false; }

  // ROS interfaces
  void currentReferenceCallback(
      const trajectory_msgs::MultiDOFJointTrajectory& trajectory_msg);

 private:
  std::optional<rmpcpp::SE3State> world_state_;

  // Wavemap-based obstacle avoidance policy
  WaveriderPolicy waverider_policy_;

  // Asynchronous plan policy publishing logic
  std::atomic<bool> continue_async_planning_ = false;
  std::thread async_planning_thread_;

  // ROS interfaces
  void subscribeToTopics(ros::NodeHandle& nh);
  ros::Subscriber current_reference_sub_;

  void advertiseTopics(ros::NodeHandle& nh_private);
  ros::Publisher policy_pub_;
  ros::Publisher debug_pub_;
};
}  // namespace waverider

#endif  // WAVERIDER_ROS_WAVERIDER_SERVER_H_
