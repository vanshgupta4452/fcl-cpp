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
find_package(Eigen3 REQUIRED)
find_package(assimp REQUIRED)
find_package(orocos_kdl REQUIRED)

# Optional: If you want to use NLOPT (TRAC-IK requires it)
find_library(NLOPT_LIB nlopt REQUIRED)

# ✅ TRAC-IK from source
add_subdirectory("/home/vansh/ros2_ws/src/trac_ik/trac_ik_lib" trac_ik_lib_build)

# ------------------------------------
# 📂 Include Directories
# ------------------------------------
include_directories(
  ${PROJECT_SOURCE_DIR}/include                         # For local headers like kdl_parser
  ${fcl_INCLUDE_DIRS}
  ${Open3D_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  /usr/include/urdf                                     # Only if model.h exists
  /usr/include/urdfdom_headers
  /home/vansh/ros2_ws/src/trac_ik/trac_ik_lib/include   # For trac_ik headers
)

# ------------------------------------
# 🧠 Executables
# ------------------------------------
add_executable(main main.cpp)
target_link_libraries(main
  fcl
  urdfdom_model
  urdfdom_world
  urdfdom_sensor
  console_bridge
  Eigen3::Eigen
  Open3D::Open3D
)

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

add_executable(self_open3d self_open3d.cpp)
target_link_libraries(self_open3d
  fcl
  urdfdom_model
  urdfdom_world
  urdfdom_sensor
  console_bridge
  Open3D::Open3D
  Eigen3::Eigen
)

# ------------------------------------
# ➕ tracik.cpp (TRAC-IK + KDL)
# ------------------------------------
add_executable(tracik tracik.cpp)

target_link_libraries(tracik
  trac_ik_lib          # ✅ Name of the target inside trac_ik_lib's CMakeLists
  orocos-kdl
  urdfdom_model
  urdfdom_world
  urdfdom_sensor
  console_bridge
  ${NLOPT_LIB}         # Needed by trac_ik
  Eigen3::Eigen
)
