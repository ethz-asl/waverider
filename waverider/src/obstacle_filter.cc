#include "waverider/obstacle_filter.h"

#include <iostream>
#include <mutex>
#include <thread>

#include <tracy/Tracy.hpp>

namespace waverider {
void ObstacleCells::swap(ObstacleCells& other) {
  ZoneScoped;
  centers.swap(other.centers);
  cell_widths.swap(other.cell_widths);
}

const ObstacleCells& WavemapObstacleFilter::getObstacleCells() {
  ZoneScoped;

  // This method is only safe to call from a single thread, warn otherwise
  static auto last_thread_id = std::this_thread::get_id();
  const auto current_thread_id = std::this_thread::get_id();
  if (current_thread_id != last_thread_id) {
    LOG(ERROR) << "Method getObstacles() should always be called from the "
                  "same thread.";
  }
  last_thread_id = current_thread_id;

  // See if updated obstacles are ready to be swapped in
  // NOTE: We use the double-checked locking pattern.
  if (new_obstacle_cells_.ready.load(std::memory_order_acquire)) {
    std::scoped_lock lock(new_obstacle_cells_.mutex);
    if (new_obstacle_cells_.ready.load(std::memory_order_acquire)) {
      obstacle_cells_.swap(new_obstacle_cells_.data);
      new_obstacle_cells_.ready.store(false, std::memory_order_release);
    }
  }

  return obstacle_cells_;
}

void WavemapObstacleFilter::update(const wavemap::HashedWaveletOctree& map,
                                   const Point3D& robot_position) {
  ZoneScoped;

  std::scoped_lock lock(new_obstacle_cells_.mutex);
  new_obstacle_cells_.ready = false;

  // Cache constants
  min_cell_width_ = map.getMinCellWidth();
  tree_height_ = map.getTreeHeight();
  constexpr FloatingPoint kNumericalNoise = 1e-3f;
  min_log_odds_shrunk_ = map.getMinLogOdds() + kNumericalNoise;
    double max_block_distance = maxRangeForHeight(6);

    if(use_only_lowest_level_){
        max_block_distance = lowest_level_radius_;
    }


  // Reset counters
  function_evals_ = 0;

  // Initialize obstacle cell array
  new_obstacle_cells_.data.centers.resize(tree_height_ + 1);
  new_obstacle_cells_.data.cell_widths.resize(tree_height_ + 1);
  for (int i = 0; i <= tree_height_; ++i) {
    new_obstacle_cells_.data.centers[i].clear();
    new_obstacle_cells_.data.cell_widths[i] =
        wavemap::convert::heightToCellWidth(min_cell_width_, i);
  }

  // Iterate over all blocks and extract obstacles
  for (const auto& [block_idx, block] : map.getBlocks()) {
    // Skip empty blocks
    if (block.empty()) {
      continue;
    }

    // Skip blocks that are too far away
    const auto block_node_index = OctreeIndex{tree_height_, block_idx};
    const wavemap::AABB<Point3D> W_block_aabb =
        wavemap::convert::nodeIndexToAABB(block_node_index, min_cell_width_);
    double block_dist = W_block_aabb.minDistanceTo(robot_position);
    if (max_block_distance < block_dist) {
      continue;
    }

    // Extract obstacles at the appropriate resolution
   if (use_only_lowest_level_) {
      // All at highest available (leaf) resolution
      leafObstacleFilter(block_idx, block, robot_position);
    } else {
      // Adaptive resolution
      adaptiveObstacleFilter(robot_position, block_node_index,
                             block.getRootNode(), block.getRootScale());
    }
  }

  // Print debug info
 /* size_t num_policies = 0;
  for (int i = 0; i <= tree_height_; ++i) {
    std::cout << "EVAL\t"
              << "LEVEL" << i << "\t"
              << new_obstacle_cells_.data.centers[i].size() << std::endl;
    num_policies += new_obstacle_cells_.data.centers[i].size();
  }
  std::cout << "EVAL\t"
            << "TOTAL\t" << num_policies << std::endl;
  std::cout << "EVAL\t"
            << "FUNC\t" << function_evals_ << std::endl;
*/
  // Indicate that the new obstacle array is ready
  new_obstacle_cells_.ready = true;
}

void WavemapObstacleFilter::leafObstacleFilter(
    const HashedWaveletOctreeBlock::BlockIndex& block_index,
    const HashedWaveletOctreeBlock& block, Point3D robot_pos) {
  block.forEachLeaf(block_index, [robot_pos_l = robot_pos, occupancy_threshold = occupancy_threshold_,
                                  min_cell_width = min_cell_width_,
                                  &new_obstacle_cells =
                                      new_obstacle_cells_.data,
          lowest_level_radius= lowest_level_radius_](
                                     const OctreeIndex& node_index,
                                     FloatingPoint node_occupancy) {
    if (occupancy_threshold < node_occupancy) {
      const Point3D node_center =
          wavemap::convert::nodeIndexToCenterPoint(node_index, min_cell_width);
      if((node_center-robot_pos_l).norm()<lowest_level_radius){
      new_obstacle_cells.centers[node_index.height].emplace_back(node_center);}
    }
  });
}

void WavemapObstacleFilter::adaptiveObstacleFilter(  // NOLINT
    const Point3D& robot_position, const OctreeIndex& node_index,
    const HashedWaveletOctreeBlock::NodeType& node,
    FloatingPoint node_occupancy) {
  ++function_evals_;

  // Skip nodes that are saturated free
  // NOTE: Such nodes are guaranteed to have no occupied children.
  if (node_occupancy < min_log_odds_shrunk_) {
    return;
  }

  // Constants
  const FloatingPoint node_width =
      wavemap::convert::heightToCellWidth(min_cell_width_, node_index.height);
  const Point3D node_center =
      wavemap::convert::indexToCenterPoint(node_index.position, node_width);

  // Check if we reached the max resolution given the node's distance
  const FloatingPoint d_robot_node = (node_center - robot_position).norm();
  //const int min_height_at_range = minHeightForRange(d_robot_node);
  double maxRange  = maxRangeForHeight(node_index.height);


  if (d_robot_node > maxRange) {
    // Add the node as an obstacle if the node itself or
    // any of its children is occupied
    if (occupancy_threshold_ < node_occupancy ||
        nodeHasOccupiedChild(node, node_occupancy)) {
      new_obstacle_cells_.data.centers[node_index.height].emplace_back(
          node_center);
    }
  } else {
    // Otherwise, keep descending the tree
    // Decompress child values
    const HashedWaveletOctreeBlock::Coefficients::CoefficientsArray
        child_occupancy_array = HashedWaveletOctreeBlock::Transform::backward(
            {node_occupancy, {node.data()}});

    // Evaluate all children
    for (wavemap::NdtreeIndexRelativeChild child_idx = 0;
         child_idx < OctreeIndex::kNumChildren; ++child_idx) {
      const OctreeIndex child_node_index =
          node_index.computeChildIndex(child_idx);
      const FloatingPoint child_occupancy = child_occupancy_array[child_idx];

      // If the child node has children, recurse
      if (node.hasChild(child_idx)) {
        const auto& child_node = *node.getChild(child_idx);
        adaptiveObstacleFilter(robot_position, child_node_index, child_node,
                               child_occupancy);
      } else {
        // Otherwise, the node must be a leaf
        // Add it as an obstacle if it's occupied
        if (occupancy_threshold_ < child_occupancy) {
          const Point3D child_center = wavemap::convert::nodeIndexToCenterPoint(
              child_node_index, min_cell_width_);
          new_obstacle_cells_.data.centers[child_node_index.height]
              .emplace_back(child_center);
        }
      }
    }
  }
}

bool WavemapObstacleFilter::nodeHasOccupiedChild(  // NOLINT
    const HashedWaveletOctreeBlock::NodeType& node,
    FloatingPoint node_occupancy) {
  const HashedWaveletOctreeBlock::Coefficients::CoefficientsArray
      child_occupancy_array = HashedWaveletOctreeBlock::Transform::backward(
          {node_occupancy, {node.data()}});

  for (wavemap::NdtreeIndexRelativeChild child_idx = 0;
       child_idx < OctreeIndex::kNumChildren; ++child_idx) {
    const FloatingPoint child_occupancy = child_occupancy_array[child_idx];
    if (occupancy_threshold_ < child_occupancy) {
      return true;
    } else if (min_log_odds_shrunk_ < child_occupancy) {
      if (node.hasChild(child_idx)) {
        const HashedWaveletOctreeBlock::NodeType& child_node =
            *node.getChild(child_idx);
        if (nodeHasOccupiedChild(child_node, child_occupancy)) {
          return true;
        }
      }
    }
  }
  return false;
}
}  // namespace waverider
