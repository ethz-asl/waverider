#include <filesystem>
#include <fstream>

#include <glog/logging.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <wavemap/data_structure/volumetric/hashed_blocks.h>
#include <wavemap/data_structure/volumetric/volumetric_data_structure_base.h>
#include <wavemap/utils/esdf/collision_utils.h>
#include <wavemap/utils/esdf/esdf_generator.h>
#include <wavemap/utils/interpolation_utils.h>
#include <wavemap_io/file_conversions.h>
#include <wavemap_msgs/Map.h>
#include <wavemap_ros_conversions/map_msg_conversions.h>
#include "chomp_ros/chomp_eval_planner.h"
#include "chomp_ros/chomp_optimizer.h"

int main(int argc, char** argv) {
  // Initialize logging
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  // Initialize ROS and advertise publishers
  ros::init(argc, argv, "chomp_planner");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");
  ros::Publisher occupancy_pub =
      nh.advertise<wavemap_msgs::Map>("map", 10, true);
  ros::Publisher esdf_pub = nh.advertise<wavemap_msgs::Map>("esdf", 10, true);
  ros::Publisher trajectory_pub =
      nh.advertise<visualization_msgs::Marker>("trajectory", 10, true);

  // Settings
  constexpr float kOccupancyThreshold = -0.1f;
  constexpr float kRobotRadius = 1.f;
  constexpr float kChompRobotRadiusPadding = 0.5f;
  std::filesystem::path occupancy_file_path =
      "/home/victor/data/wavemaps/newer_college_mine_5cm.wvmp";
  std::filesystem::path logging_dir = "/home/victor/data/waverider/chomp_stats";

  // Create the logging dir if needed
  std::filesystem::create_directories(logging_dir);

  // Load the occupancy map
  wavemap::VolumetricDataStructureBase::Ptr occupancy_map;
  wavemap::io::fileToMap(occupancy_file_path, occupancy_map);
  CHECK_NOTNULL(occupancy_map);

  // Publish the occupancy map
  wavemap_msgs::Map occupancy_map_msg;
  wavemap::convert::mapToRosMsg(*occupancy_map, "odom", ros::Time::now(),
                                occupancy_map_msg);
  occupancy_pub.publish(occupancy_map_msg);
  ros::spinOnce();

  // Currently, only hashed wavelet octree maps are supported as input
  const auto hashed_map =
      std::dynamic_pointer_cast<wavemap::HashedWaveletOctree>(occupancy_map);
  if (!hashed_map) {
    return EXIT_FAILURE;
  }

  // Load or generate the ESDF
  wavemap::HashedBlocks::Ptr esdf;
  const std::filesystem::path esdf_file_path =
      std::filesystem::path(occupancy_file_path).replace_extension("dwvmp");
  if (std::filesystem::exists(esdf_file_path)) {
    // Load the ESDF
    LOG(INFO) << "Loading ESDF from path: " << esdf_file_path;
    wavemap::VolumetricDataStructureBase::Ptr esdf_tmp;
    if (!wavemap::io::fileToMap(esdf_file_path, esdf_tmp)) {
      LOG(ERROR) << "Could not load ESDF";
      return EXIT_FAILURE;
    }
    esdf = std::dynamic_pointer_cast<wavemap::HashedBlocks>(esdf_tmp);
    if (!esdf) {
      LOG(ERROR) << "Loaded ESDF is not of type hashed blocks";
      return EXIT_FAILURE;
    }
  } else {
    // Generate the ESDF
    LOG(INFO) << "Generating ESDF";
    constexpr float kMaxDistance = 2.f;
    esdf = std::make_shared<wavemap::HashedBlocks>(
        generateEsdf(*hashed_map, kOccupancyThreshold, kMaxDistance));

    // Save the ESDF
    LOG(INFO) << "Saving ESDF to path: " << esdf_file_path;
    if (!wavemap::io::mapToFile(*esdf, esdf_file_path)) {
      LOG(ERROR) << "Could not save ESDF";
      return EXIT_FAILURE;
    }
  }

  // Publish the ESDF
  wavemap_msgs::Map msg;
  wavemap::convert::mapToRosMsg(*esdf, "odom", ros::Time::now(), msg);
  esdf_pub.publish(msg);

  // Define the ESDF distance getter
  auto distance_getter = [&occupancy_map,
                          &esdf](const Eigen::Vector3d& position_d) {
    const wavemap::Point3D position = position_d.cast<wavemap::FloatingPoint>();
    if (wavemap::interpolateTrilinear(*occupancy_map, position) <
        kOccupancyThreshold) {
      return wavemap::interpolateTrilinear(*esdf, position);
    } else {
      return 0.f;
    }
  };

  // Initialize CHOMP
  chomp::ChompOptimizer chomp;
  chomp::ChompParameters params;
  params.map_resolution = esdf->getMinCellWidth();
  params.w_collision = 10.0;
  params.w_smooth = 0.1;
  params.lambda = 100;  // 20.1
  params.max_iter = 100;
  params.epsilon = kRobotRadius + kChompRobotRadiusPadding;
  params.decrease_step_size = true;
  chomp.setParameters(params);
  chomp.setDistanceFunction(distance_getter);

  // Loop forever (for debugging)
  size_t run_idx = 0u;
  while (true) {
    // Sample random start and goal positions until a
    // collision-free trajectory between them is found
    chomp::ChompTrajectory chomp_output;
    while (true) {
      // Exit if ctrl+c is pressed
      if (!ros::ok()) {
        break;
      }

      // Get random start and goal positions
      const auto start = wavemap::getCollisionFreePosition(*occupancy_map,
                                                           *esdf, kRobotRadius);
      const auto goal = wavemap::getCollisionFreePosition(*occupancy_map, *esdf,
                                                          kRobotRadius);
      if (!start || !goal) {
        LOG(ERROR) << "Could not find collision free start and goal positions";
        return EXIT_FAILURE;
      } else {
        LOG(INFO) << "Found collision free start and goal positions";
      }

      // Publish the start and goal positions
      {
        // Set up the marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time::now();
        marker.id = 100;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = kRobotRadius;
        marker.scale.y = kRobotRadius;
        marker.scale.z = kRobotRadius;
        marker.color.a = 1.0;
        marker.pose.orientation.w = 1.0;
        // Publish the start position
        marker.ns = "start";
        marker.color.b = 1.0;
        marker.pose.position.x = start->x();
        marker.pose.position.y = start->y();
        marker.pose.position.z = start->z();
        trajectory_pub.publish(marker);
        // Publish the goal position
        marker.ns = "goal";
        marker.color.b = 0.0;
        marker.color.g = 1.0;
        marker.pose.position.x = goal->x();
        marker.pose.position.y = goal->y();
        marker.pose.position.z = goal->z();
        trajectory_pub.publish(marker);
      }

      // Compute trajectory with CHOMP
      LOG(INFO) << "Computing trajectory with CHOMP";
      int N = 500;
      chomp.solveProblem(start->cast<double>(), goal->cast<double>(), N,
                         &chomp_output);

      // Create a log file for the trajectory
      const auto log_file_path =
          logging_dir / ("run_" + std::to_string(run_idx));
      std::ofstream log_file_ostream(
          log_file_path, std::ofstream::out | std::ofstream::binary);
      if (!log_file_ostream.is_open()) {
        LOG(WARNING) << "Could not open file " << log_file_path
                     << " for writing. Error: " << strerror(errno);
        return EXIT_FAILURE;
      }

      // Check the trajectory and log stats
      bool is_collision_free = true;
      for (int idx = 0; idx < chomp_output.trajectory.rows(); ++idx) {
        const auto position = chomp_output.trajectory.row(idx);
        const float esdf_distance = distance_getter(position);
        // Check if we collided
        if (esdf_distance <= kRobotRadius) {
          is_collision_free = false;
        }
        // Log position and ESDF value to file
        log_file_ostream << position.x() << ", " << position.y() << ", "
                         << position.z() << ", " << esdf_distance << "\n";
      }

      // Close the log file and check if writing succeeded
      log_file_ostream.close();
      if (!log_file_ostream) {
        LOG(WARNING) << "Failed to write to file " << log_file_path
                     << ". Error: " << strerror(errno);
        return EXIT_FAILURE;
      }
      ++run_idx;

      if (is_collision_free) {
        LOG(INFO) << "Solution trajectory is collision free";
        break;
      } else {
        LOG(INFO) << "Solution trajectory is NOT collision free";
      }
    }

    // Publish trajectory
    LOG(INFO) << "Publishing trajectory";
    visualization_msgs::Marker trajectory_msg;
    trajectory_msg.header.frame_id = "odom";
    trajectory_msg.header.stamp = ros::Time::now();
    trajectory_msg.type = visualization_msgs::Marker::LINE_STRIP;
    trajectory_msg.action = visualization_msgs::Marker::ADD;
    trajectory_msg.id = 100;
    trajectory_msg.ns = "trajectory";
    trajectory_msg.scale.x = kRobotRadius;
    trajectory_msg.scale.y = kRobotRadius;
    trajectory_msg.scale.z = kRobotRadius;
    trajectory_msg.color.r = 1.0;
    trajectory_msg.color.a = 1.0;
    trajectory_msg.pose.orientation.w = 1.0;
    for (int idx = 0; idx < chomp_output.trajectory.rows(); ++idx) {
      const auto position = chomp_output.trajectory.row(idx);
      auto& position_msg = trajectory_msg.points.emplace_back();
      position_msg.x = position.x();
      position_msg.y = position.y();
      position_msg.z = position.z();
    }
    trajectory_pub.publish(trajectory_msg);

    // Check if we should continue
    if (!ros::ok()) {
      // Exit if ctrl+c is pressed
      break;
    } else {
      // Give us some time to look at the trajectory
      ros::Duration(3).sleep();
    }
  }

  ros::spin();
}
