#include "waverider_ros/waverider_server.h"

#include <mav_reactive_planning/PolicyValue.h>
#include <omav_msgs/conversions.h>
#include <omav_msgs/eigen_omav_msgs.h>
#include <rmpcpp/geometry/partial_geometry.h>
#include <visualization_msgs/MarkerArray.h>

#include "waverider_ros/policy_visuals.h"

namespace waverider {
void WaveriderServer::updateMap(
    const wavemap::VolumetricDataStructureBase& map) {
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
}
void WaveriderServer::startPlanningAsync() {
  if (continue_async_planning_) {
    ROS_INFO("Async planning already enabled.");
    return;
  }

  continue_async_planning_ = true;
  async_planning_thread_ = std::thread(
      [&waverider_policy = waverider_policy_, &world_state = world_state_,
       &policy_pub = policy_pub_, &debug_pub = debug_pub_,
       &should_continue = continue_async_planning_]() {
        ros::Rate rate(200.0);
        while (ros::ok() && should_continue) {
          if (waverider_policy.isReady() && world_state.has_value()) {
            // Compute the policy
            const auto val_wavemap_r3_W =
                waverider_policy.evaluateAt(world_state->r3());
            rmpcpp::R3toSE3 geometry;
            const rmpcpp::PolicyValue<7> val_wavemap_W =
                geometry.at(world_state->r3()).pull(val_wavemap_r3_W);

            // Publish the policy
            mav_reactive_planning::PolicyValue policy_msg;
            policy_msg.Name = "waverider";
            policy_msg.f = std::vector<double>(
                val_wavemap_W.f_.data(),
                val_wavemap_W.f_.data() + val_wavemap_W.f_.size());
            policy_msg.A = std::vector<double>(
                val_wavemap_W.A_.data(),
                val_wavemap_W.A_.data() + val_wavemap_W.A_.size());
            policy_pub.publish(policy_msg);

            // Publish debug visuals
            static int i = 0;
            if (++i % 20) {
              visualization_msgs::MarkerArray marker_array =
                  filteredObstaclesToMarkerArray(
                      waverider_policy.getObstacleCells());
              marker_array.markers.emplace_back(
                  robotPositionToMarker(world_state->p().cast<float>()));
              debug_pub.publish(marker_array);
            }
          }
          rate.sleep();
        }
        ROS_INFO("Stopped async planning.");
      });
}

void WaveriderServer::currentReferenceCallback(
    const trajectory_msgs::MultiDOFJointTrajectory& trajectory_msg) {
  const auto latest = trajectory_msg.points.at(0);
  omav_msgs::EigenTrajectoryPoint latest_eigen =
      omav_msgs::eigenTrajectoryPointFromMsg(latest);

  // Setpoint transform from body to global
  Eigen::Affine3d T_odom_body_ref = Eigen::Affine3d::Identity();
  T_odom_body_ref.translation() = latest_eigen.getPosition_W();
  T_odom_body_ref.linear() =
      latest_eigen.getOrientation_W_B().toRotationMatrix();
  // Velocity and acceleration in odom
  Eigen::Vector3d v = latest_eigen.getVelocity_B();
  Eigen::Vector3d vdot = latest_eigen.getAcceleration_B();
  // Body angular velocity and acceleration
  Eigen::Vector3d w = latest_eigen.getAngularVelocity_B();
  Eigen::Vector3d wdot = latest_eigen.getAngularAcceleration_B();

  // Convert into world state
  world_state_.emplace();
  world_state_->p() = T_odom_body_ref.translation();
  world_state_->q() = T_odom_body_ref.rotation();
  world_state_->v() = T_odom_body_ref.rotation() * v;
  world_state_->a() = T_odom_body_ref.rotation() * vdot;
  world_state_->w() = T_odom_body_ref.rotation() * w;
  world_state_->dw() = T_odom_body_ref.rotation() * wdot;
}

void WaveriderServer::subscribeToTopics(ros::NodeHandle& nh) {
  current_reference_sub_ = nh.subscribe(
      "current_reference", 1, &WaveriderServer::currentReferenceCallback, this);
}

void WaveriderServer::advertiseTopics(ros::NodeHandle& nh_private) {
  policy_pub_ =
      nh_private.advertise<mav_reactive_planning::PolicyValue>("policy", 1);
  debug_pub_ = nh_private.advertise<visualization_msgs::MarkerArray>(
      "filtered_obstacles", 1);
}
}  // namespace waverider
