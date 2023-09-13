cmake_minimum_required(VERSION 3.0.2)
project(waverider_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# Compiler definitions and options
add_wavemap_compile_definitions_and_options()

# Libraries
cs_add_library(${PROJECT_NAME}
        src/policy_visuals.cc
        src/waverider_server.cc
        src/waverider_evaluator.cc)

# Binaries
cs_add_executable(waverider_server_node app/server_node.cc)
target_link_libraries(waverider_server_node ${PROJECT_NAME})

cs_add_executable(waverider_eval app/eval_node.cc)
target_link_libraries(waverider_eval ${PROJECT_NAME})


# Export
cs_install()
cs_export()
