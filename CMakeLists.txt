cmake_minimum_required(VERSION 3.0.2)
project(waverider)

# NOTE: This CMakeLists file is not used by ROS and catkin, it only exists to
#       make it easy to load and edit all packages simultaneously in IDEs.

# Include the catkin workspace's include dir, e.g. for ros msg definitions
execute_process(COMMAND catkin locate --devel
        OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE CATKIN_WS_DEVEL_PATH)
include_directories(${CATKIN_WS_DEVEL_PATH}/include)

# ROS interfaces and tooling
add_subdirectory(waverider)
add_subdirectory(waverider_ros)
add_subdirectory(chomp_ros)
