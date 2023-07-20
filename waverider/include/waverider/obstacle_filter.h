#ifndef WAVERIDER_OBSTACLE_FILTER_H_
#define WAVERIDER_OBSTACLE_FILTER_H_

#include <wavemap/data_structure/volumetric/hashed_wavelet_octree.h>

#include <utility>
#include <vector>

namespace waverider {
struct ObstacleCells {
  std::vector<std::vector<Eigen::Vector3f>> centers;
  std::vector<float> cell_widths;
};

class WavemapObstacleFilter {
 public:
  WavemapObstacleFilter() = default;

  void setOccupancyThreshold(wavemap::FloatingPoint threshold) { occupancy_threshold_ = threshold; }

  void update(const wavemap::HashedWaveletOctree& map, const wavemap::Point3D& robot_position);

  bool isReady() const { return !obstacle_cells_.centers.empty(); }
  const ObstacleCells& getObstacleCells() const { return obstacle_cells_; }

  bool use_only_lowest_level_ = false;

 private:
  size_t function_evals{0};
  wavemap::FloatingPoint occupancy_threshold_ = 0.001f;
  ObstacleCells obstacle_cells_;
  // function that defines the radius we care about for each tree level
  std::function<double(uint)> f_lvl_cutoff_;

  // recursive version -> if none of the children is to be added as a policy
  // add itself as a policy
  // this can happen if all the children are unoccupied or out of region of
  // interest returns true if a policy was added
  bool recursiveObstacleFilter(const wavemap::HashedWaveletOctree& map,
                               const wavemap::Point3D& robot_position,
                               const wavemap::OctreeIndex& node_index,
                               const wavemap::HashedWaveletOctreeBlock::NodeType& node,
                               wavemap::FloatingPoint node_scale_coefficient);
};
}  // namespace waverider

#endif  // WAVERIDER_OBSTACLE_FILTER_H_
