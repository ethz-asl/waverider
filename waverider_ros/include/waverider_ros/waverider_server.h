#ifndef WAVERIDER_ROS_WAVERIDER_SERVER_H_
#define WAVERIDER_ROS_WAVERIDER_SERVER_H_

#include <string>
#include <thread>

#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <wavemap/config/config_base.h>
#include <wavemap/data_structure/volumetric/volumetric_data_structure_base.h>
#include <wavemap_ros/tf_transformer.h>
#include <waverider/waverider_policy.h>

#include <std_srvs/Empty.h>

namespace waverider {
struct WaveriderServerConfig : wavemap::ConfigBase<WaveriderServerConfig, 3> {
  std::string world_frame = "odom";

  int publish_debug_visuals_every_n_iterations = 20;
  std::string get_state_from_tf_frame;  // Leave blank to disable

  static MemberMap memberMap;

  bool isValid(bool verbose) const override;
};

class WaveriderServer {
 public:
  WaveriderServer(ros::NodeHandle nh, ros::NodeHandle nh_private);
  WaveriderServer(ros::NodeHandle nh, ros::NodeHandle nh_private,
                  const WaveriderServerConfig& config);

  void updateMap(const wavemap::VolumetricDataStructureBase& map);

  void startPlanningAsync();
  void stopPlanningAsync() { continue_async_planning_ = false; }

  // ROS interfaces
  void currentReferenceCallback(
      const trajectory_msgs::MultiDOFJointTrajectory& trajectory_msg);
  void estimateStateFromTf();

  bool toggleServiceCallback(std_srvs::Empty::Request  &req,
                                              std_srvs::Empty::Response &re);

 private:
  const WaveriderServerConfig config_;

  // Wavemap-based obstacle avoidance policy
  std::optional<rmpcpp::SE3State> world_state_;
  WaveriderPolicy waverider_policy_;
  std::mutex mutex_;  // Use for both world_state_ and waverider_policy_

  // Asynchronous plan policy publishing logic
  std::atomic<bool> continue_async_planning_ = false;
  std::thread async_planning_thread_;

  // ROS interfaces
  void subscribeToTopics(ros::NodeHandle& nh);
  ros::Subscriber current_reference_sub_;
  wavemap::TfTransformer transformer_;

  void subscribeToTimers(const ros::NodeHandle& nh);
  ros::Timer state_from_tf_timer_;

  void advertiseTopics(ros::NodeHandle& nh_private);
  ros::Publisher policy_pub_;
  ros::Publisher debug_pub_;


  ros::ServiceServer srv_level_toggle_;



};
}  // namespace waverider

#endif  // WAVERIDER_ROS_WAVERIDER_SERVER_H_
