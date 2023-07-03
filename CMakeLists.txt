cmake_minimum_required(VERSION 3.0.2)
project(waverider)

# NOTE: This CMakeLists file is not used by ROS and catkin, it only exists to
#       make it easy to load and edit all packages simultaneously in IDEs.

# ROS interfaces and tooling
add_subdirectory(waverider_msgs)
add_subdirectory(waverider_ros)
