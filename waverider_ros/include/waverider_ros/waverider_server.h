#ifndef WAVERIDER_ROS_WAVERIDER_SERVER_H_
#define WAVERIDER_ROS_WAVERIDER_SERVER_H_

#include <ros/ros.h>
#include <wavemap_ros/wavemap_server.h>

namespace waverider {
class WaveriderServer : public wavemap::WavemapServer {
  using wavemap::WavemapServer::WavemapServer;
};
}  // namespace waverider

#endif  // WAVERIDER_ROS_WAVERIDER_SERVER_H_
