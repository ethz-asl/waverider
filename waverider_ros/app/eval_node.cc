#include <gflags/gflags.h>
#include <wavemap_ros/wavemap_server.h>

#include "waverider_eval/waverider_evaluator.h"

int main(int argc, char** argv) {
  // Register with ROS
  ros::init(argc, argv, "waverider_eval");

  // Setup GLOG and register a failure signal handler that prints the callstack
  // if the program is killed (e.g. SIGSEGV)
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  // create the evaluator
  waverider::WaveriderEvaluatorConfig cfg;
  cfg.hans = "hans";
  waverider::WaveriderEvaluator evaluator(cfg);

  evaluator.loadMap("/home/mpantic/Work/waverider/meps/newer_college_math_5cm.wvmp");
  evaluator.plan({0,0,0}, {10,4,1});

  return 0;
}
