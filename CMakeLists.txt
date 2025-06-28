cmake_minimum_required(VERSION 3.10)
project(fcl_urdf_loader)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ------------------------------------
# 🔍 Find Required Packages
# ------------------------------------
find_package(fcl REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(Open3D REQUIRED)
find_package(Eigen3 REQUIRED)  # ✅ Recommended for Eigen
find_package(assimp REQUIRED)  # Optional if loading mesh later

# ------------------------------------
# 📂 Include Directories (optional in modern CMake if targets are used)
# ------------------------------------
include_directories(
  ${fcl_INCLUDE_DIRS}
  ${urdfdom_INCLUDE_DIRS}
  ${Open3D_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# ------------------------------------
# 🧠 main.cpp (Self-Collision Checker)
# ------------------------------------
add_executable(main main.cpp)
target_link_libraries(main
  fcl
  urdfdom_model
  urdfdom_world
  urdfdom_sensor
  console_bridge
  Eigen3::Eigen
)

# ------------------------------------
# 🌍 env_coll.cpp (URDF Loader + FCL + Open3D Visualizer)
# ------------------------------------
add_executable(env_coll env_coll.cpp)

target_link_libraries(env_coll
  fcl
  urdfdom_model
  urdfdom_world
  urdfdom_sensor
  console_bridge
  Open3D::Open3D
  assimp
  Eigen3::Eigen
)

add_executable(open3d_env env_open3d.cpp)

target_link_libraries(open3d_env
  fcl
  urdfdom_model
  urdfdom_world
  urdfdom_sensor
  console_bridge
  Open3D::Open3D
  assimp
  Eigen3::Eigen
)
