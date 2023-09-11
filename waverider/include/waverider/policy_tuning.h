#ifndef WAVERIDER_POLICY_TUNING_H_
#define WAVERIDER_POLICY_TUNING_H_

namespace waverider {
struct PolicyTuning {
  float r = 1.3;
  float c = 0.05;
  float eta_rep = 3.4;   // n
  float nu_rep = 0.5;    // v
  float eta_damp = 2.0;  // n
  float nu_damp = 1;     // v
  bool enable_damper = true;
  bool enable_repulsor = true;
};
}  // namespace waverider

#endif  // WAVERIDER_POLICY_TUNING_H_
