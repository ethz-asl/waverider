cmake_minimum_required(VERSION 3.0.2)
project(chomp_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# Compiler definitions and options
add_wavemap_compile_definitions_and_options()

# Libraries
cs_add_library(${PROJECT_NAME}
    src/chomp_optimizer.cc)

# Binaries
cs_add_executable(chomp_node app/chomp_node.cc)
target_link_libraries(chomp_node ${PROJECT_NAME})

# Export
cs_install()
cs_export()
