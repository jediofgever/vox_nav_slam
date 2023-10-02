// Copyright (c) 2022 Fetullah Atas, Norwegian University of Life Sciences
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef VOX_NAV_SLAM__SCAN_MATCHER__FAST_GICP_CLIENT_HPP_
#define VOX_NAV_SLAM__SCAN_MATCHER__FAST_GICP_CLIENT_HPP_

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/registration/registration.h>
#include <pcl/filters/voxel_grid.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/create_timer_ros.h>
#include <pcl/filters/model_outlier_removal.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_gicp_st.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <fast_gicp/ndt/ndt_cuda.hpp>
#include <fast_gicp/gicp/fast_vgicp_cuda.hpp>

#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Key.h>
#include <gtsam/geometry/Pose3.h>

#include <vox_nav_slam_msgs/msg/map_array.hpp>
#include <vox_nav_slam_msgs/msg/sub_map.hpp>

#include <Eigen/Core>
#include <queue>
#include <vector>
#include <string>
#include <memory>
#include <mutex>

namespace vox_nav_slam
{

struct ICPParameters
{
  float x_bound = 40.0;
  float y_bound = 40.0;
  float z_bound = 5.0;
  float downsample_voxel_size = 0.2;
  float map_voxel_size = 0.1;
  int max_icp_iter = 30;
  float max_correspondence_distance = 3.0;
  std::string method{ "VGICP" };  //'VGICP' # OTHER OPTIONS' GICP, VGICP, VGICP_CUDA, NDT_CUDA
  int num_threads = 4;
  int max_num_targeted_clouds = 20;
  float min_dist_to_update_map = 0.5;
  bool publish_tf = true;
  bool debug = false;
};

/**
 * @brief Given a
 *
 */
class FastGICPScanMatcher : public rclcpp::Node
{
public:
  FastGICPScanMatcher(const rclcpp::NodeOptions& options);

  ~FastGICPScanMatcher();

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom);

  void initializeMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,  // NOLINT
                     const std_msgs::msg::Header& header);

  pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr createRegistration(std::string method, int num_threads,
                                                                            double voxel_resolution = 0.2);

  void performRegistration(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,  // NOLINT
                           const std_msgs::msg::Header& header);

  void insertScantoMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                       const Eigen::Matrix4f& icp_estimate,                  // NOLINT
                       const geometry_msgs::msg::PoseStamped& current_pose,  // NOLINT
                       const std_msgs::msg::Header& header);

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_pub_;
  rclcpp::Publisher<vox_nav_slam_msgs::msg::MapArray>::SharedPtr sub_maps_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  // tf buffer to get access to transfroms
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr targeted_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_msg_;
  geometry_msgs::msg::PoseStamped::SharedPtr current_pose_;
  nav_msgs::msg::Path::SharedPtr path_;
  vox_nav_slam_msgs::msg::MapArray::SharedPtr sub_maps_;

  // Traverasblity estimation realted variables
  std_msgs::msg::Bool::SharedPtr episode_end_;

  ICPParameters icp_params_;

  pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr reg_;

  bool is_first_cloud_received_{ false };
  std::shared_ptr<std::mutex> mutex_;
  std::shared_ptr<std::thread> icp_thread_;
  std::packaged_task<void()> icp_task_;
  std::shared_ptr<std::future<void>> icp_future_;
  Eigen::Matrix4f previous_odom_mat_{ Eigen::Matrix4f::Identity() };
  Eigen::Matrix4f current_map_origin_{ Eigen::Matrix4f::Identity() };
  Eigen::Matrix4f previous_icp_mat_{ Eigen::Matrix4f::Identity() };
  Eigen::Vector3f previous_position_{ Eigen::Vector3f::Zero() };
  float translation_from_previous_{ 0.0 };
  float accumulated_translation_{ 0.0 };
  bool is_map_updated_{ false };
  bool is_odom_updated_{ false };
  bool mapping_flag_{ false };
};

}  // namespace vox_nav_slam

#endif  // VOX_NAV_SLAM__SCAN_MATCHER__FAST_GICP_CLIENT_HPP_
