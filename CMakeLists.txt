cmake_minimum_required(VERSION 3.10)
project(fcl_urdf_loader)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find required packages
find_package(fcl REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(assimp REQUIRED)

# Include directories
include_directories(
  ${fcl_INCLUDE_DIRS}
  ${urdfdom_INCLUDE_DIRS}
)

# ----- main.cpp (Self-Collision Checker) -----
add_executable(main main.cpp)
target_link_libraries(main
  fcl
  urdfdom_model
  urdfdom_world
  urdfdom_sensor
  console_bridge
)

# ----- enc_coll.cpp (Environment Collision Checker) -----
add_executable(env_coll env_coll.cpp)
target_link_libraries(env_coll
  fcl
  urdfdom_model
  urdfdom_world
  urdfdom_sensor
  console_bridge
  assimp
)
