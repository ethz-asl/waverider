#ifndef WAVERIDER_POLICY_TUNING_H_
#define WAVERIDER_POLICY_TUNING_H_

namespace waverider {
struct PolicyTuning {
  float r = 0.1;
  float c = 0.1;
  float eta_rep = 80;   // n
  float nu_rep = 1;     // v
  float eta_damp = 14;  // n
  float nu_damp = 1;    // v
  bool enable_damper = true;
  bool enable_repulsor = true;
};
}  // namespace waverider

#endif  // WAVERIDER_POLICY_TUNING_H_
