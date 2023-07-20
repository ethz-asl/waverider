#include "waverider_ros/waverider_server.h"

#include <mav_reactive_planning/PolicyValue.h>
#include <omav_msgs/conversions.h>
#include <omav_msgs/eigen_omav_msgs.h>
#include <rmpcpp/geometry/partial_geometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <wavemap/config/param.h>
#include <wavemap_ros_conversions/config_conversions.h>

#include "waverider_ros/policy_visuals.h"

namespace waverider {
DECLARE_CONFIG_MEMBERS(WaveriderServerConfig,
                      (world_frame)
                      (publish_debug_visuals_every_n_iterations)
                      (get_state_from_tf_frame));

bool WaveriderServerConfig::isValid(bool verbose) const {
  bool all_valid = true;

  all_valid &= IS_PARAM_NE(world_frame, std::string(""), verbose);

  return all_valid;
}

WaveriderServer::WaveriderServer(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : WaveriderServer(
          nh, nh_private,
          WaveriderServerConfig::from(
              wavemap::param::convert::toParamMap(nh_private, ""))) {}

WaveriderServer::WaveriderServer(ros::NodeHandle nh, ros::NodeHandle nh_private,
                                 const WaveriderServerConfig& config)
    : config_(config.checkValid()) {
  subscribeToTopics(nh);
  subscribeToTimers(nh);
  advertiseTopics(nh_private);

  srv_level_toggle_ = nh.advertiseService(
      "toggle_levels", &WaveriderServer::toggleServiceCallback, this);
}

void WaveriderServer::updateMap(
    const wavemap::VolumetricDataStructureBase& map) {
  mapper_waiting_ = true;
  std::scoped_lock lock(mutex_);

  if (!world_state_.has_value()) {
    ROS_WARN("World state not yet initialized.");
    return;
  }

  if (auto hashed_map = dynamic_cast<const wavemap::HashedWaveletOctree*>(&map);
      hashed_map) {
    const wavemap::Point3D robot_position =
        world_state_->p().cast<wavemap::FloatingPoint>();
    waverider_policy_.updateObstacles(*hashed_map, robot_position);
  } else {
    ROS_WARN(
        "Waverider policies can currently only be extracted from maps of "
        "type wavemap::HashedWaveletOctree.");
  }

  mapper_waiting_ = false;
  mapper_waiting_cv_.notify_all();
}
void WaveriderServer::startPlanningAsync() {
  if (continue_async_planning_) {
    ROS_INFO("Async planning already enabled.");
    return;
  }

  continue_async_planning_ = true;
  async_planning_thread_ =
      std::thread(&WaveriderServer::asyncPlanningLoop, this);
}

void WaveriderServer::currentReferenceCallback(
    const trajectory_msgs::MultiDOFJointTrajectory& trajectory_msg) {
  const auto current_setpoint = trajectory_msg.points.front();
  const omav_msgs::EigenTrajectoryPoint current_setpoint_eigen =
      omav_msgs::eigenTrajectoryPointFromMsg(current_setpoint);

  // Setpoint transform from body to global
  Eigen::Affine3d T_odom_body_ref = Eigen::Affine3d::Identity();
  T_odom_body_ref.translation() = current_setpoint_eigen.getPosition_W();
  T_odom_body_ref.linear() =
      current_setpoint_eigen.getOrientation_W_B().toRotationMatrix();
  // Velocity and acceleration in odom
  Eigen::Vector3d v = current_setpoint_eigen.getVelocity_B();
  Eigen::Vector3d vdot = current_setpoint_eigen.getAcceleration_B();
  // Body angular velocity and acceleration
  Eigen::Vector3d w = current_setpoint_eigen.getAngularVelocity_B();
  Eigen::Vector3d wdot = current_setpoint_eigen.getAngularAcceleration_B();

  // Convert into world state
  std::scoped_lock lock(mutex_);
  world_state_.emplace();
  world_state_->p() = T_odom_body_ref.translation();
  world_state_->q() = T_odom_body_ref.rotation();
  world_state_->v() = T_odom_body_ref.rotation() * v;
  world_state_->a() = T_odom_body_ref.rotation() * vdot;
  world_state_->w() = T_odom_body_ref.rotation() * w;
  world_state_->dw() = T_odom_body_ref.rotation() * wdot;
}

void WaveriderServer::estimateStateFromTf() {
  wavemap::Transformation3D T_W_R_current;
  if (transformer_.lookupLatestTransform(config_.world_frame,
                                         config_.get_state_from_tf_frame,
                                         T_W_R_current)) {
    std::scoped_lock lock(mutex_);
    world_state_.emplace();
    world_state_->p() = T_W_R_current.getPosition().cast<double>();
    world_state_->q() =
        T_W_R_current.getRotation().getRotationMatrix().cast<double>();
    world_state_->v().setZero();
    world_state_->a().setZero();
    world_state_->w().setZero();
    world_state_->dw().setZero();
  } else {
    ROS_WARN_STREAM_THROTTLE(
        1, "Could not estimate state from TFs. Poses of frame '"
               << config_.get_state_from_tf_frame << "' in '"
               << config_.world_frame << "' at current time is not available.");
  }
}

bool WaveriderServer::toggleServiceCallback(std_srvs::Empty::Request& req,
                                            std_srvs::Empty::Response& re) {
  waverider_policy_.obstacle_filter_.use_only_lowest_level_ =
      !waverider_policy_.obstacle_filter_.use_only_lowest_level_;
  std::cout << "EVAL\t" << ros::Time::now() << "\tLEVELS TOGGLED" << std::endl;
  return true;
}

void WaveriderServer::asyncPlanningLoop() {
  ros::Rate rate(200.0);
  while (ros::ok() && continue_async_planning_) {
    {
      std::unique_lock lock(mutex_);
      if (mapper_waiting_) {
        mapper_waiting_cv_.wait(lock,
                                [&]() -> bool { return !mapper_waiting_; });
      }
      if (waverider_policy_.isReady() && world_state_.has_value()) {
        evaluateAndPublishPolicy();
      }
    }
    rate.sleep();
  }
  ROS_INFO("Stopped async planning.");
}

void WaveriderServer::evaluateAndPublishPolicy() {
  std::string policy_name = std::to_string(ros::Time::now().toSec());
  std::cout << "EVAL\t" << ros::Time::now() << "\tCREATED\t" << policy_name
            << std::endl;

  // Compute the policy
  const auto val_wavemap_r3_W =
      waverider_policy_.evaluateAt(world_state_->r3());

  // Publish the policy
  mav_reactive_planning::PolicyValue policy_msg;
  policy_msg.Name = policy_name;
  policy_msg.f = std::vector<double>(
      val_wavemap_r3_W.f_.data(),
      val_wavemap_r3_W.f_.data() + val_wavemap_r3_W.f_.size());
  policy_msg.A = std::vector<double>(
      val_wavemap_r3_W.A_.data(),
      val_wavemap_r3_W.A_.data() + val_wavemap_r3_W.A_.size());
  std::cout << "EVAL\t" << ros::Time::now() << "\tPUBLISHING\t" << policy_name
            << std::endl;

  policy_pub_.publish(policy_msg);

  // Publish debug visuals
  static int i = 0;
  if (++i % config_.publish_debug_visuals_every_n_iterations == 0) {
    visualization_msgs::MarkerArray marker_array;
    // marker_array.markers.emplace_back(generateClearingMarker());
    addFilteredObstaclesToMarkerArray(waverider_policy_.getObstacleCells(),
                                      config_.world_frame, marker_array);
    marker_array.markers.emplace_back(robotPositionToMarker(
        world_state_->p().cast<float>(), config_.world_frame));
    debug_pub_.publish(marker_array);
  }
}

void WaveriderServer::subscribeToTopics(ros::NodeHandle& nh) {
  current_reference_sub_ = nh.subscribe(
      "current_reference", 1, &WaveriderServer::currentReferenceCallback, this);
}

void WaveriderServer::subscribeToTimers(const ros::NodeHandle& nh) {
  if (!config_.get_state_from_tf_frame.empty()) {
    state_from_tf_timer_ =
        nh.createTimer(ros::Duration(0.02),
                       [this](const auto /*event*/) { estimateStateFromTf(); });
  }
}

void WaveriderServer::advertiseTopics(ros::NodeHandle& nh_private) {
  policy_pub_ =
      nh_private.advertise<mav_reactive_planning::PolicyValue>("policy", 1);
  debug_pub_ = nh_private.advertise<visualization_msgs::MarkerArray>(
      "filtered_obstacles", 1);
}
}  // namespace waverider
