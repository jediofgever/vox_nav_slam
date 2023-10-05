#include "robot_centric_slam/fast_gicp_optim_component.h"
#include "robot_centric_slam/fast_gicp_scan_matcher_component.hpp"
#include "robot_centric_slam/gtsam_component.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  rclcpp::executors::MultiThreadedExecutor exec;

  auto scanmatcher = std::make_shared<vox_nav_slam::FastGICPScanMatcher>(options);
  exec.add_node(scanmatcher);
  // auto optim = std::make_shared<vox_nav_slam::FastGICPOptimComponent>(options);
  // exec.add_node(optim);
  //  auto gtsam = std::make_shared<vox_nav_slam::GTSAMComponent>(options);
  //  exec.add_node(gtsam);

  exec.spin();
  rclcpp::shutdown();

  return 0;
}
