#include "waverider/waverider_policy.h"
#include "waverider/obstacle_filter.h"
#include <tracy/Tracy.hpp>

namespace waverider {
void WaveriderPolicy::updateObstacles(const wavemap::HashedWaveletOctree& map,
                                      const Point3D& robot_position) {
  ZoneScoped;
  obstacle_filter_.update(map, robot_position);
}

rmpcpp::PolicyValue<3> WaveriderPolicy::evaluateAt(const rmpcpp::State<3>& x) {
  ZoneScoped;
  if (!isReady()) {
    return {Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero()};
  }

  const Eigen::Vector3f x_pos = x.pos_.cast<float>();
  const Eigen::Vector3f x_vel = x.vel_.cast<float>();

  // get all cells where we should attach a policy
  const auto& policy_cells = obstacle_filter_.getObstacleCells();

  std::vector<rmpcpp::PolicyValue<3>> all_policies;
  for (size_t i = 0; i < policy_cells.cell_widths.size(); i++) {
    if (i == 1 || run_all_levels_) {
        std::cout << "N"<< i << policy_cells.centers[i].size() << std::endl;
      ParallelizedPolicy pol_generator(policy_cells.centers[i].size(),
                                       policy_tuning_);
      pol_generator.setR(WavemapObstacleFilter::maxRangeForHeight(i)*1.1);

      pol_generator.init(policy_cells.centers[i], x_pos, x_vel);
      all_policies.emplace_back(pol_generator.getPolicy());
    }
  }

  const auto avoidance_policy = rmpcpp::PolicyValue<3>::sum(all_policies);

  rmpcpp::PolicyValue<3> scaled_avoidance = {avoidance_policy.f_,
                                             avoidance_policy.A_};
  //  std::cout << scaled_avoidance.f_.transpose() << std::endl;
  //  std::cout << scaled_avoidance.A_ << std::endl;
  return scaled_avoidance;
}
}  // namespace waverider
