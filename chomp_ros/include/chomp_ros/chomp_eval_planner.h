//
// Created by mpantic on 14.09.23.
//

#ifndef CHOMP_ROS_CHOMP_EVAL_PLANNER_H
#define CHOMP_ROS_CHOMP_EVAL_PLANNER_H
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
#include <wavemap/utils/stopwatch.h>

#include "chomp_ros/chomp_optimizer.h"
#include <waverider/eval_planner.h>

class ChompEvalPlanner : public waverider::EvalPlanner{

public:
    ChompEvalPlanner( wavemap::VolumetricDataStructureBase::Ptr occupancy_map,
    wavemap::HashedBlocks::Ptr esdf): occupancy_map_(occupancy_map),
    esdf_(esdf){


    }


    virtual std::string getName() {
        return "CHOMP";
    }

    virtual Result plan(Eigen::Vector3d start, Eigen::Vector3d end) {
      constexpr float kOccupancyThreshold = -0.1f;
        constexpr float kRobotRadius = 1.f;
        constexpr float kChompRobotRadiusPadding = 0.5f;

        // Define the ESDF distance getter
        auto distance_getter = [&](const Eigen::Vector3d& position_d) {
            const wavemap::Point3D position = position_d.cast<wavemap::FloatingPoint>();
            if (wavemap::interpolateTrilinear(*occupancy_map_, position) <
                kOccupancyThreshold) {
                return wavemap::interpolateTrilinear(*esdf_, position);
            } else {
                return 0.f;
            }
        };


        chomp::ChompOptimizer chomp;
        chomp::ChompParameters params;
        params.map_resolution = esdf_->getMinCellWidth();
        params.w_collision = 10.0;
        params.w_smooth = 0.1;
        params.lambda = 100;  // 20.1
        params.max_iter = 100;
        params.epsilon = kRobotRadius + kChompRobotRadiusPadding;
        params.decrease_step_size = true;
        chomp.setParameters(params);
        chomp.setDistanceFunction(distance_getter);

        chomp::ChompTrajectory chomp_output;

        wavemap::Stopwatch watch;
        watch.start();
        int N = 500;
        chomp.solveProblem(start, end, N,
                           &chomp_output);

        watch.stop();
        Result result;
        result.duration = std::chrono::duration_cast< std::chrono::steady_clock::duration>(std::chrono::duration<double>(watch.getLastEpisodeDuration()));



        result.states_out.resize(chomp_output.trajectory.rows());
        for (int idx = 0; idx < chomp_output.trajectory.rows(); ++idx) {
            result.states_out[idx] =  chomp_output.trajectory.row(idx);;
        }




      return result;
    }

    wavemap::VolumetricDataStructureBase::Ptr occupancy_map_;
    wavemap::HashedBlocks::Ptr esdf_;

};

#endif //CHOMP_ROS_CHOMP_EVAL_PLANNER_H
