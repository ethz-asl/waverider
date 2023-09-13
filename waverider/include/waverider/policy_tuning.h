#ifndef WAVERIDER_POLICY_TUNING_H_
#define WAVERIDER_POLICY_TUNING_H_

namespace waverider {
struct PolicyTuning {
  float r = 1.3;
  float c = 0.2;
  float eta_rep = 88;   // n
  float nu_rep = 1.4;    // v
  float eta_damp = 140;  // n
  float nu_damp = 1.2;     // v
  bool enable_damper = true;
  bool enable_repulsor = true;
};
}  // namespace waverider

#endif  // WAVERIDER_POLICY_TUNING_H_
