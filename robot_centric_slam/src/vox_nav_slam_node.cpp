#include "robot_centric_slam/graph_based_slam_component.h"
#include "robot_centric_slam/scan_matcher_component.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  rclcpp::executors::MultiThreadedExecutor exec;

  auto scanmatcher = std::make_shared<vox_nav_slam::FastGICPScanMatcher>(options);
  exec.add_node(scanmatcher);
  // auto graphbasedslam = std::make_shared<graphslam::GraphBasedSlamComponent>(options);
  // exec.add_node(graphbasedslam);

  exec.spin();
  rclcpp::shutdown();

  return 0;
}
