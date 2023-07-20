cmake_minimum_required(VERSION 3.5)
project(robot_centric_slam)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
  #add_compile_options(-Wextra -Wpedantic)
endif()

SET(CMAKE_CXX_FLAGS "-O2 -g ${CMAKE_CXX_FLAGS}")
add_compile_definitions(G2O_USE_VENDORED_CERES)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(vox_nav_slam_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(g2o REQUIRED
                  COMPONENTS
                  g2o::core
                  g2o::types_slam3d
                  g2o::solver_eigen
                  )
find_package(OpenMP)
find_package(fast_gicp REQUIRED)
find_package(CUDA REQUIRED)
find_package(GTSAM REQUIRED)

option(BUILD_VGICP_CUDA "Build GPU-powered VGICP" ON)
add_definitions(-BUILD_VGICP_CUDA)
add_definitions(${PCL_DEFINITIONS})

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

set(dependencies
    rclcpp 
    rclcpp_components 
    tf2_ros 
    tf2_eigen
    std_srvs
    tf2_geometry_msgs
    tf2_sensor_msgs
    pcl_conversions
    geometry_msgs 
    sensor_msgs
    nav_msgs
    visualization_msgs
    vox_nav_slam_msgs)

add_library(graph_based_slam_component SHARED src/graph_based_slam_component.cpp)
ament_target_dependencies(graph_based_slam_component ${dependencies})
target_compile_definitions(graph_based_slam_component PRIVATE "GS_GBS_BUILDING_DLL")

add_library(fast_gicp_scan_matcher_component SHARED src/fast_gicp_scan_matcher_component.cpp)
ament_target_dependencies(fast_gicp_scan_matcher_component ${dependencies})
target_compile_definitions(fast_gicp_scan_matcher_component PRIVATE "GS_GBS_BUILDING_DLL")

add_library(gtsam_component SHARED src/gtsam_component.cpp)
ament_target_dependencies(gtsam_component ${dependencies})
target_compile_definitions(gtsam_component PRIVATE "GS_GBS_BUILDING_DLL")

add_executable(vox_nav_slam_node src/vox_nav_slam_node.cpp)
ament_target_dependencies(vox_nav_slam_node ${dependencies})
include_directories(
    include
    ${PCL_INCLUDE_DIRS}
    ${fast_gicp_INCLUDE_DIRS}
)
target_link_libraries(vox_nav_slam_node
                      graph_based_slam_component 
                      fast_gicp_scan_matcher_component
                      gtsam_component
                      ${PCL_LIBRARIES}
                      ${fast_gicp_LIBRARIES}
                      gtsam
                      gtsam_unstable
                      g2o::core
                      g2o::types_slam3d
                      g2o::solver_eigen)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(ros2bag_analyzer src/ros2bag_analyzer.cpp)
ament_target_dependencies(ros2bag_analyzer ${dependencies})
include_directories(include)
target_link_libraries(ros2bag_analyzer ${PCL_LIBRARIES})
link_directories(${PCL_LIBRARY_DIRS})

rclcpp_components_register_nodes(graph_based_slam_component 
  "graphslam::GraphBasedSlamComponent")
rclcpp_components_register_nodes(fast_gicp_scan_matcher_component 
  "vox_nav_slam::FastGICPScanMatcher")
rclcpp_components_register_nodes(gtsam_component 
  "vox_nav_slam::GTSAMComponent")

ament_export_libraries(graph_based_slam_component)
ament_export_libraries(fast_gicp_scan_matcher_component)
ament_export_libraries(gtsam_component)

install(
  DIRECTORY "include/"
  DESTINATION include)
install(
  DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME})
install(TARGETS
  graph_based_slam_component
  fast_gicp_scan_matcher_component
  gtsam_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS 
  vox_nav_slam_node
  ros2bag_analyzer
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_package()
