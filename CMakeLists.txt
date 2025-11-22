cmake_minimum_required(VERSION 3.10)
project(On-Manifold-IMU-Preintegration)

set(CMAKE_CXX_STANDARD 20)

find_package(GTSAM CONFIG REQUIRED)

add_executable(Modeling_Robot_Motion "cpp_examples/factor_graphs_and_gtsam_tutorial/Modeling_Robot_Motion.cpp")
target_link_libraries(Modeling_Robot_Motion PRIVATE gtsam)

add_executable(Robot_Localization "cpp_examples/factor_graphs_and_gtsam_tutorial/Robot_Localization.cpp")
target_link_libraries(Robot_Localization PRIVATE gtsam)