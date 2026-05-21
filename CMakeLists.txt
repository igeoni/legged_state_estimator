cmake_minimum_required(VERSION 3.8)
project(legged_state_estimator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pinocchio REQUIRED)

# GTSAM (installed to /usr/local)
find_package(GTSAM REQUIRED HINTS /usr/local/lib/cmake)

# ---------------------------------------------------------------------------
# contact_collector_lib (shared between collector executable and tests)
# ---------------------------------------------------------------------------

set(CONTACT_COLLECTOR_SOURCES
  src/kinematics/g1_kinematics.cpp
  src/data_collector/npy_writer.cpp
  src/data_collector/data_buffer.cpp
  src/data_collector/contact_data_collector.cpp
)

add_library(contact_collector_lib STATIC ${CONTACT_COLLECTOR_SOURCES})

target_include_directories(contact_collector_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(contact_collector_lib
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  Eigen3
)

target_link_libraries(contact_collector_lib
  pinocchio::pinocchio
)

# ---------------------------------------------------------------------------
# contact_data_collector executable
# ---------------------------------------------------------------------------

add_executable(contact_data_collector
  src/data_collector/contact_data_collector_main.cpp
)

target_link_libraries(contact_data_collector contact_collector_lib)

ament_target_dependencies(contact_data_collector
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  Eigen3
)

# ---------------------------------------------------------------------------
# fk_visualizer executable
# ---------------------------------------------------------------------------

add_executable(fk_visualizer
  src/visualizer/fk_visualizer.cpp
  src/visualizer/fk_visualizer_main.cpp
  src/kinematics/g1_kinematics.cpp
)

target_include_directories(fk_visualizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(fk_visualizer
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
  Eigen3
)

target_link_libraries(fk_visualizer
  pinocchio::pinocchio
)

# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# humanoid_state_estimator executable
#
# Split into two static libs to avoid the Boost-serialization conflict
# between pinocchio (ROS2 humble) and GTSAM when compiled in the same TU:
#
#   humanoid_fk_lib   — pinocchio only (g1_kinematics + g1_fk_provider)
#   humanoid_est_lib  — GTSAM + ROS2 (estimator + node)
# ---------------------------------------------------------------------------

# FK lib: pinocchio TU — must NOT link gtsam
add_library(humanoid_fk_lib STATIC
  src/kinematics/g1_kinematics.cpp
  src/kinematics/g1_fk_provider.cpp
)

target_include_directories(humanoid_fk_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(humanoid_fk_lib
  Eigen3
)

target_link_libraries(humanoid_fk_lib
  pinocchio::pinocchio
)

# Estimator lib: GTSAM + ROS2 TU — must NOT include pinocchio headers
add_library(humanoid_est_lib STATIC
  src/state_estimator/humanoid_estimator.cpp
  src/state_estimator/humanoid_estimator_node.cpp
)

target_include_directories(humanoid_est_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/local/include
)

ament_target_dependencies(humanoid_est_lib
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  Eigen3
)

target_link_libraries(humanoid_est_lib
  gtsam
)

add_executable(humanoid_state_estimator
  src/state_estimator/humanoid_estimator_main.cpp
)

target_link_libraries(humanoid_state_estimator
  humanoid_est_lib
  humanoid_fk_lib
)

# ---------------------------------------------------------------------------

install(TARGETS contact_data_collector fk_visualizer humanoid_state_estimator
        humanoid_fk_lib humanoid_est_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY description/
  DESTINATION share/${PROJECT_NAME}/description
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
