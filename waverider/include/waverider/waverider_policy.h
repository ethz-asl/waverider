#ifndef WAVERIDER_WAVERIDER_POLICY_H_
#define WAVERIDER_WAVERIDER_POLICY_H_

#include <rmpcpp/core/policy_base.h>
#include <rmpcpp/core/state.h>
#include <wavemap/data_structure/volumetric/hashed_wavelet_octree.h>

#include "waverider/parallelized_policy.h"

namespace waverider {
class WaveriderPolicy : public rmpcpp::PolicyBase<rmpcpp::Space<3>> {
 public:
  WaveriderPolicy() = default;

  void setOccupancyThreshold(wavemap::FloatingPoint value) {
    obstacle_filter_.setOccupancyThreshold(value);
  }
  void setRunAllLevels(bool run_all_levels) {
    run_all_levels_ = run_all_levels;
  }
  const ObstacleCells& getObstacleCells() const {
    return obstacle_filter_.getObstacleCells();
  }

  void updateObstacles(const wavemap::HashedWaveletOctree& map,
                       const wavemap::Point3D& robot_position);
  void updateTuning(PolicyTuning tuning) { policy_tuning_ = tuning; }

  bool isReady() const { return obstacle_filter_.isReady(); }

  rmpcpp::PolicyValue<3> evaluateAt(const rmpcpp::State<3>& x) override;

 public: // hack for now
  WavemapObstacleFilter obstacle_filter_;
  bool run_all_levels_ = true;
  PolicyTuning policy_tuning_;
};
}  // namespace waverider

#endif  // WAVERIDER_WAVERIDER_POLICY_H_
