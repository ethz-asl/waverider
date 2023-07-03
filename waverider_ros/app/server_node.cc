#include <gflags/gflags.h>

#include "waverider_ros/waverider_server.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "waverider_server");

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  waverider::WaveriderServer waverider_server(nh, nh_private);

  ros::spin();
  return 0;
}
