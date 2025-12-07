cmake_minimum_required(VERSION 3.10)
project(On-Manifold-IMU-Preintegration)

set(CMAKE_CXX_STANDARD 20)

find_package(GTSAM CONFIG REQUIRED)

# List all example source files
set(EXAMPLE_SOURCES
    cpp_examples/factor_graphs_and_gtsam_tutorial/0_Modeling_Robot_Motion.cpp
    cpp_examples/factor_graphs_and_gtsam_tutorial/1_Robot_Localization.cpp
    cpp_examples/factor_graphs_and_gtsam_tutorial/2_Pose_SLAM.cpp
)

# Loop to create executables
foreach(src ${EXAMPLE_SOURCES})
    get_filename_component(exec_name ${src} NAME_WE)

    add_executable(${exec_name} ${src})
    target_link_libraries(${exec_name} PRIVATE gtsam)
endforeach()