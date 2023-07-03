//
// Created by mpantic on 07.06.23.
//

#ifndef MAV_REACTIVE_PLANNING_WAVEMAP_POLICY_FILTER_H
#define MAV_REACTIVE_PLANNING_WAVEMAP_POLICY_FILTER_H
#include <iostream>
#include <utility>

#include <Eigen/Dense>
#include <wavemap/data_structure/volumetric/hashed_wavelet_octree.h>

#include "wavemap/data_structure/volumetric/cell_types/haar_coefficients.h"
#include "wavemap/data_structure/volumetric/cell_types/haar_transform.h"

struct PolicyCells {
  std::vector<std::vector<Eigen::Vector3f>> blocks;
  std::vector<float> block_size;
};

class WavemapPolicyFilter {
  struct StackElement {
    const wavemap::OctreeIndex node_index;
    const wavemap::HashedWaveletOctreeBlock::NodeType& node;
    const wavemap::HashedWaveletOctreeBlock::Coefficients::Scale
        scale_coefficient{};
  };

 public:
  WavemapPolicyFilter() = default;

  void updateFilter(double cutoff) { filter_cutoff_ = cutoff; }
  void updateMap(std::shared_ptr<wavemap::HashedWaveletOctree> map) {
    wavemap_ = std::move(map);
  }

  void updateRobotPosition(const Eigen::Vector3f& robot_position) {
    robot_position_ = robot_position;
    policy_cells_.blocks.clear();
  }

  void generatePolicies() {
    // this is the maximum distance we care about
    std::cout << "Running generate Policies  " << filter_cutoff_ << std::endl;
    f_lvl_cutoff_ = [](uint level) { return (level + 1); };
    double max_cutoff = f_lvl_cutoff_(6);
    // init debug structure
    policy_cells_.blocks.resize(wavemap_->getTreeHeight() + 1);
    policy_cells_.block_size.resize(wavemap_->getTreeHeight() + 1);
    for (int i = 0; i <= wavemap_->getTreeHeight(); ++i) {
      policy_cells_.block_size[i] =
          wavemap::convert::heightToCellWidth(wavemap_->getMinCellWidth(), i);
      // std::cout << i << "\t" << policy_cells_.block_size[i] << "\t" <<
      // f_lvl_cutoff_(i) << std::endl;
    }

    // for now, we do these naively by doing a full check each time
    // in the future, we could cache the AABB tree's for all blocks
    // and simply check if there is any new block to worry about
    for (const auto& [block_idx, block] : wavemap_->getBlocks()) {
      //    for (const auto& block : wavemap_->getBlocks()) {

      if (block.empty() && std::abs(block.getRootScale()) < 1e-6) {
        // std::cout << "\t empty." << std::endl;
        continue;  // skip - its empty
      }
      // get aabb tree for block
      wavemap::AABB<wavemap::Point3D> W_cell_aabb =
          wavemap::convert::nodeIndexToAABB(
              (wavemap::OctreeIndex){wavemap_->getTreeHeight(), block_idx},
              wavemap_->getMinCellWidth());

      // check if closest distance is within the radius we care about
      double block_dist = W_cell_aabb.minDistanceTo(robot_position_);
      if (block_dist > max_cutoff) {
        // std::cout << "\t too far away." << std::endl;
        continue;  // skip this block - it's too far away
      }
      // std::cout << "visiting block " << block.first.transpose() << std::endl;
      //  traverse block.
      visitNode({wavemap::OctreeIndex({wavemap_->getTreeHeight(), block_idx}),
                 block.getRootNode(), block.getRootScale()});
    }
  }

  // recursive version -> if none of the children is to be added as a policy
  // add itself as a policy
  // this can happen if all the children are unoccupied or out of region of
  // interest returns true if a policy was added
  bool visitNode(const StackElement& node) {  // NOLINT
    if (node.node_index.height == 0) {
      std::cout << ".";
    }
    // std::cout << "\t \t visiting lvl " << node.node_index.height << " " <<
    // node.node_index.position.transpose() << std::endl;

    // std::cout << "KEEP" << cell_odds << " " << log_odds << std::endl;
    if (node.scale_coefficient < wavemap_->getMinLogOdds() + 1E-6) {
      return false;
    }

    const wavemap::FloatingPoint node_width =
        wavemap::convert::heightToCellWidth(wavemap_->getMinCellWidth(),
                                            node.node_index.height);
    Eigen::Vector3f center_point = wavemap::convert::indexToCenterPoint(
        node.node_index.position, node_width);
    bool within_search_scope = (center_point - robot_position_).norm() <
                               f_lvl_cutoff_(node.node_index.height) +
                                   sqrt(3 * node_width * node_width);
    if (!within_search_scope) {
      return false;
    }

    // if we make it to here, that means something is occupied and we are
    // potentially in the region where the children could be in scope.

    const wavemap::HashedWaveletOctreeBlock::Coefficients::CoefficientsArray
        child_scale_coefficients =
            wavemap::HashedWaveletOctreeBlock::Transform::backward(
                {node.scale_coefficient, {node.node.data()}});

    bool any_of_kids_added = false;
    for (wavemap::NdtreeIndexRelativeChild child_idx = 0;
         child_idx < wavemap::OctreeIndex::kNumChildren; ++child_idx) {
      const wavemap::OctreeIndex child_node_index =
          node.node_index.computeChildIndex(child_idx);

      const wavemap::FloatingPoint child_scale_coefficient =
          child_scale_coefficients[child_idx];

      // add all children to be visited
      if (node.node.hasChild(child_idx)) {
        const wavemap::HashedWaveletOctreeBlock::NodeType& child_node =
            *node.node.getChild(child_idx);
        any_of_kids_added |= visitNode(StackElement{
            child_node_index, child_node, child_scale_coefficient});
      } else {
        if (child_scale_coefficient > filter_cutoff_) {
          Eigen::Vector3f center_point_child =
              wavemap::convert::nodeIndexToCenterPoint(
                  child_node_index, wavemap_->getMinCellWidth());

          const wavemap::FloatingPoint child_node_width =
              wavemap::convert::heightToCellWidth(wavemap_->getMinCellWidth(),
                                                  child_node_index.height);
          bool within_boundaries_child =
              (center_point_child - robot_position_).norm() <
              f_lvl_cutoff_(child_node_index.height) +
                  sqrt(3 * child_node_width * child_node_width);

          if (within_boundaries_child) {
            // add policy!
            // std::cout << "\t \t \t added at lvl " << node.node_index.height
            // << "/" << center_point.transpose() << std::endl;
            policy_cells_.blocks[child_node_index.height].push_back(
                center_point_child);
            any_of_kids_added = true;
          }
        }
      }
    }

    // do DFS recursion to test if any of the children are within scope and
    // occupied if so, add those

    // if not, check if itself is within scope and add if in scope
    if (!any_of_kids_added && node.scale_coefficient > filter_cutoff_) {
      // check within scope
      bool within_boundaries = (center_point - robot_position_).norm() <
                               f_lvl_cutoff_(node.node_index.height) +
                                   sqrt(3 * node_width * node_width);

      if (within_boundaries) {
        // add policy!
        // std::cout << "\t \t \t added at lvl " << node.node_index.height <<
        // "/" << center_point.transpose() << std::endl;
        policy_cells_.blocks[node.node_index.height].push_back(center_point);
        if (node.node_index.height == 0) {
          std::cout << "!";
        }
        return true;
      }
    }
    // else return false - upper level should decide
    return any_of_kids_added;
  }

  void publishDebug() {
    // console output
    for (int i = 0; i < policy_cells_.block_size.size(); ++i) {
      std::cout << i << "\t" << policy_cells_.block_size[i] << std::endl;
      for (auto pol : policy_cells_.blocks[i]) {
        std::cout << "\t" << pol.transpose() << std::endl;
      }
    }
  }

  PolicyCells& getPolicyCells() { return policy_cells_; }

 private:
  double filter_cutoff_{-0.6};
  Eigen::Vector3f robot_position_;
  PolicyCells policy_cells_;
  std::shared_ptr<wavemap::HashedWaveletOctree> wavemap_;
  std::function<double(uint)>
      f_lvl_cutoff_;  // function that defines the radius we care about for each
                      // tree level
  // std::pair<uint, uint> active_lvl_range_ {0,7};
};

#endif  // MAV_REACTIVE_PLANNING_WAVEMAP_POLICY_FILTER_H
