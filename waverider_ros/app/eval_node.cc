#include <gflags/gflags.h>
#include <wavemap_ros/wavemap_server.h>

#include "waverider_eval/waverider_evaluator.h"
#include <wavemap_io/file_conversions.h>
#include <wavemap/utils/esdf/collision_utils.h>
#include <wavemap/utils/esdf/esdf_generator.h>
#include <wavemap/utils/interpolation_utils.h>
#include <visualization_msgs/MarkerArray.h>
#include <chomp_ros/chomp_eval_planner.h>
ros::Publisher trajectory_pub;
ros::Publisher esdf_pub;

int traject_id = 0;
void publishTrajectory(std::vector<Eigen::Vector3d>& trajectory, std::vector<Eigen::Vector3d>& colors, std::string name){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.id = traject_id++;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 0.8;
  marker.pose.orientation.w = 1.0;
  marker.ns = name;

  for(int i=0; i< trajectory.size(); i+=1){
    geometry_msgs::Point pt;
    pt.x = trajectory[i].x();
    pt.y = trajectory[i].y();
    pt.z = trajectory[i].z();
    marker.points.push_back(pt);

    std_msgs::ColorRGBA clr;
    clr.a = 1.0;
    clr.r = colors[i].x();
    clr.g = colors[i].y();
    clr.b = colors[i].z();
    marker.colors.push_back(clr);
  }



  trajectory_pub.publish(marker);

}

int main(int argc, char** argv) {
  // Register with ROS
  ros::init(argc, argv, "waverider_eval");
  ros::NodeHandle nh("");

  esdf_pub = nh.advertise<wavemap_msgs::Map>("esdf", 10, true);
  trajectory_pub =
      nh.advertise<visualization_msgs::Marker>("trajectory", 10, true);
  // Setup GLOG and register a failure signal handler that prints the callstack
  // if the program is killed (e.g. SIGSEGV)
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  // load map and ESDF
  std::string occupancy_file_path =
      "/home/mpantic/Work/waverider/meps/newer_college_mine_10cm.wvmp";
  wavemap::VolumetricDataStructureBase::Ptr occupancy_map;
  wavemap::io::fileToMap(occupancy_file_path, occupancy_map);


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
    constexpr float kOccupancyThreshold = 0.0f;
    constexpr float kRobotRadius = 1.f;
    const auto hashed_map =
        std::dynamic_pointer_cast<wavemap::HashedWaveletOctree>(occupancy_map);
    if (!hashed_map) {
      return EXIT_FAILURE;
    }
    esdf = std::make_shared<wavemap::HashedBlocks>(
        generateEsdf(*hashed_map, kOccupancyThreshold, kMaxDistance));

    // Save the ESDF
    LOG(INFO) << "Saving ESDF to path: " << esdf_file_path;
    if (!wavemap::io::mapToFile(*esdf, esdf_file_path)) {
      LOG(ERROR) << "Could not save ESDF";
      return EXIT_FAILURE;
    }
  }

  wavemap_msgs::Map esdf_msg;
  wavemap::convert::mapToRosMsg(*esdf, "map", ros::Time::now(), esdf_msg);
  esdf_pub.publish(esdf_msg);

  // create the evaluator
  waverider::WaveriderEvaluatorConfig cfg;
  cfg.hans = "hans";

  std::vector<waverider::EvalPlanner*> planners;

  auto* planner_waverider = new waverider::WaveriderEvaluator(cfg, false);
  auto* planner_waverider_1 = new waverider::WaveriderEvaluator(cfg, true, 1.0);
  auto* planner_waverider_3 = new waverider::WaveriderEvaluator(cfg, true, 3.0);
  auto* planner_chomp = new ChompEvalPlanner(occupancy_map, esdf);
  planner_chomp->color = {0.5, 0.5, 0.0};
  planner_waverider->loadMap(occupancy_file_path);
  planner_waverider->color = {1.0, 0,0 };
  planner_waverider_1->loadMap(occupancy_file_path);
  planner_waverider_1->color = {0.0, 1.0,0 };

  planner_waverider_3->color = {0.0, 0,1.0 };
  planner_waverider_3->loadMap(occupancy_file_path);
  planners.push_back(planner_chomp);
  planners.push_back(planner_waverider);
  planners.push_back(planner_waverider_1);
  planners.push_back(planner_waverider_3);

  constexpr float kOccupancyThreshold =-0.1;

  auto distance_getter = [&occupancy_map,
                          &esdf](const Eigen::Vector3d& position_d) {
    const wavemap::Point3D position =
        position_d.cast<wavemap::FloatingPoint>();
    return wavemap::interpolateTrilinear(*esdf, position);
  };

  for (int i = 0; i < 500; ++i) {
    double dist = 10000;
    Eigen::Vector3d start_3d, goal_3d;

    while(dist > 30.0 || dist < 5.0) {
      wavemap::AABB<wavemap::Point3D> bounding_start;
      bounding_start.max = {18.8, 3, 4};
      bounding_start.min = {-2.5, -12.5, -3};

      const auto start =
          wavemap::getCollisionFreePosition(*occupancy_map, *esdf, 1,bounding_start);
      const auto goal =
          wavemap::getCollisionFreePosition(*occupancy_map, *esdf, 1,bounding_start);

      if(start && goal){

        dist = (start.value() -goal.value()).norm();
        start_3d = start.value().cast<double>();
        goal_3d = goal.value().cast<double>();

      }
    }

    // Publish the start and goal positions
    {
      // Set up the marker
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.id = traject_id;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;
      marker.color.a = 1.0;
      marker.pose.orientation.w = 1.0;
      // Publish the start position
      marker.ns = "start";
      marker.color.b = 1.0;
      marker.pose.position.x = start_3d.x();
      marker.pose.position.y = start_3d.y();
      marker.pose.position.z = start_3d.z();
      trajectory_pub.publish(marker);
      // Publish the goal position
      marker.ns = "goal";
      marker.color.b = 0.0;
      marker.color.g = 1.0;
      marker.pose.position.x = goal_3d.x();
      marker.pose.position.y = goal_3d.y();
      marker.pose.position.z = goal_3d.z();
      trajectory_pub.publish(marker);
    }


    for (auto planner : planners) {
      auto result = planner->plan(start_3d,goal_3d);
      // check goal distance
      double goal_dist = (result.states_out.back()-goal_3d).norm();

      const auto log_file_path =
          "/tmp/waverider_run_" +planner->getName() + std::to_string(i)+"_"+"_.log";
      std::ofstream log_file_ostream(
          log_file_path, std::ofstream::out | std::ofstream::binary);
      if (!log_file_ostream.is_open()) {
        LOG(WARNING) << "Could not open file " << log_file_path
                     << " for writing. Error: " << strerror(errno);
        return EXIT_FAILURE;
      }

      std::vector<Eigen::Vector3d> colors;
      colors.resize(result.states_out.size());
      // check for collisions
      bool is_collision_free = true;
      for (int idx = 0; idx < result.states_out.size(); ++idx) {
        const auto position = result.states_out[idx];
        const float esdf_distance = distance_getter(position);
        log_file_ostream << position.x() << ", " << position.y() << ", "
                         << position.z() << ", " << esdf_distance << "\n";

        colors[idx].setZero();
        colors[idx] = planner->color;
        if(goal_dist > 0.1){
          colors[idx] *= 0.5;
        }

        // Check if we collided
        if (esdf_distance <= 0.25) {
          std::cout << "FAIL " << planner->getName() << "\t" <<position.transpose() << "\t" << esdf_distance <<std::endl;
          is_collision_free = false;
          colors[idx] *= 0.5;
        }
      }
      log_file_ostream.close();

      std::cout << "EVAL " << planner->getName()+"_" + std::to_string(i) << "\t" << result.success << "\t"
                << ((double)result.duration.count())/1e9 << "\t" << result.states_out.size() << "\t" << goal_dist
                <<  "\t" << is_collision_free<<  std::endl;


        publishTrajectory(result.states_out, colors, planner->getName());

    }
  }
  return 0;
}
