cmake_minimum_required(VERSION 3.20)
project(autopark_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(
  autopark_run
  src/main.cpp
  src/geometry_lane.cpp
  src/connector_transfer.cpp
  src/forward_overshoot.cpp
  src/reverse_bezier.cpp
  src/viz_svg.cpp
  src/axis_fillet.cpp
  src/bicycle_sim.cpp
  src/parking_simulate.cpp
)
target_include_directories(
  autopark_run
  PRIVATE
    include
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/nlohmann_json/single_include"
)
