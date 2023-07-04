cmake_minimum_required(VERSION 3.0.2)
project(waverider)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# Compiler definitions and options
add_wavemap_compile_definitions_and_options()

# Libraries
cs_add_library(${PROJECT_NAME}
        src/obstacle_filter.cc
        src/parallelized_policy.cc
        src/waverider_policy.cc)

# Export
cs_install()
cs_export()
