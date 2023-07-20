#include "waverider/obstacle_filter.h"
#include<iostream>
namespace waverider {
void WavemapObstacleFilter::update(const wavemap::HashedWaveletOctree& map,
                                   const wavemap::Point3D& robot_position) {



  // this is the maximum distance we care about
  f_lvl_cutoff_ = [](uint level) -> double { return std::exp((level+1.0)/2.0); };
  //f_lvl_cutoff_ = [](uint level) { return level+1.5; };
  if(use_only_lowest_level_) {
    // replacing a lambda.. bad style.
    // its worse than i thought
    f_lvl_cutoff_ =  [](uint level) -> double { return std::exp((6+1.0)/2.0); };

  }

  const double max_cutoff = f_lvl_cutoff_(6);
  // init debug structure
  obstacle_cells_.centers.resize(map.getTreeHeight() + 1);
  obstacle_cells_.cell_widths.resize(map.getTreeHeight() + 1);
  for (int i = 0; i <= map.getTreeHeight(); ++i) {
    obstacle_cells_.centers[i].clear();
    obstacle_cells_.cell_widths[i] =
        wavemap::convert::heightToCellWidth(map.getMinCellWidth(), i);
  }

  function_evals = 0;
  // for now, we do these naively by doing a full check each time
  // in the future, we could cache the AABB tree's for all blocks
  // and simply check if there is any new block to worry about
  for (const auto& [block_idx, block] : map.getBlocks()) {
    if (block.empty()) {
      continue;  // skip - its empty
    }
    // get aabb tree for block
    wavemap::AABB<wavemap::Point3D> W_cell_aabb =
        wavemap::convert::nodeIndexToAABB(
            wavemap::OctreeIndex{map.getTreeHeight(), block_idx},
            map.getMinCellWidth());

    // check if closest distance is within the radius we care about
    double block_dist = W_cell_aabb.minDistanceTo(robot_position);
    if (block_dist > max_cutoff) {
      continue;  // skip this block - it's too far away
    }
    // traverse block
    recursiveObstacleFilter(
        map, robot_position,
        wavemap::OctreeIndex{map.getTreeHeight(), block_idx},
        block.getRootNode(), block.getRootScale());
  }
  int all_policies = 0;
  for (int i = 0; i <= map.getTreeHeight(); ++i) {
    std::cout << "EVAL\t" <<  "LEVEL"<< i<< "\t" <<obstacle_cells_.centers[i].size()<< std::endl;
    all_policies += obstacle_cells_.centers[i].size();
  }
  std::cout << "EVAL\t" <<  "TOTAL\t" <<all_policies<< std::endl;
  std::cout << "EVAL\t" << "FUNC\t" << function_evals << std::endl;

}

bool WavemapObstacleFilter::recursiveObstacleFilter(  // NOLINT
    const wavemap::HashedWaveletOctree& map,
    const wavemap::Point3D& robot_position,
    const wavemap::OctreeIndex& node_index,
    const wavemap::HashedWaveletOctreeBlock::NodeType& node,
    const wavemap::FloatingPoint node_scale_coefficient) {
    ++function_evals;
  if (node_scale_coefficient < map.getMinLogOdds() + 1e-4f) {
    return false;
  }

  const wavemap::FloatingPoint node_width = wavemap::convert::heightToCellWidth(
      map.getMinCellWidth(), node_index.height);
  const Eigen::Vector3f center_point =
      wavemap::convert::indexToCenterPoint(node_index.position, node_width);
  const bool within_search_scope =
      (center_point - robot_position).norm() <
      f_lvl_cutoff_(node_index.height) + sqrt(3 * node_width * node_width);
  if (!within_search_scope) {
    return false;
  }

  // if we make it to here, that means something is occupied and we are
  // potentially in the region where the children could be in scope.
  const wavemap::HashedWaveletOctreeBlock::Coefficients::CoefficientsArray
      child_scale_coefficients =
          wavemap::HashedWaveletOctreeBlock::Transform::backward(
              {node_scale_coefficient, {node.data()}});

  bool any_of_kids_added = false;
  for (wavemap::NdtreeIndexRelativeChild child_idx = 0;
       child_idx < wavemap::OctreeIndex::kNumChildren; ++child_idx) {
    const wavemap::OctreeIndex child_node_index =
        node_index.computeChildIndex(child_idx);

    const wavemap::FloatingPoint child_scale_coefficient =
        child_scale_coefficients[child_idx];

    // add all children to be visited
    if (node.hasChild(child_idx)) {
      const wavemap::HashedWaveletOctreeBlock::NodeType& child_node =
          *node.getChild(child_idx);
      any_of_kids_added |=
          recursiveObstacleFilter(map, robot_position, child_node_index,
                                  child_node, child_scale_coefficient);
    } else {
      if (child_scale_coefficient > occupancy_threshold_) {
        const Eigen::Vector3f center_point_child =
            wavemap::convert::nodeIndexToCenterPoint(child_node_index,
                                                     map.getMinCellWidth());

        const wavemap::FloatingPoint child_node_width =
            wavemap::convert::heightToCellWidth(map.getMinCellWidth(),
                                                child_node_index.height);
        const bool within_boundaries_child =
            (center_point_child - robot_position).norm() <
            f_lvl_cutoff_(child_node_index.height) +
                sqrt(3 * child_node_width * child_node_width);

        if (within_boundaries_child) {
          // add policy!
          obstacle_cells_.centers[child_node_index.height].emplace_back(
              center_point_child);
          any_of_kids_added = true;
        }
      }
    }
  }

  // do DFS recursion to test if any of the children are within scope and
  // occupied if so, add those

  // if not, check if itself is within scope and add if in scope
  if (!use_only_lowest_level_ && !any_of_kids_added && node_scale_coefficient > occupancy_threshold_) {
    // check within scope
    const bool within_boundaries =
        (center_point - robot_position).norm() <
        f_lvl_cutoff_(node_index.height) + sqrt(3 * node_width * node_width);

    if (within_boundaries) {
      // add policy!
      obstacle_cells_.centers[node_index.height].emplace_back(center_point);
      return true;
    }
  }
  // else return false - upper level should decide
  return any_of_kids_added;
}
}  // namespace waverider
