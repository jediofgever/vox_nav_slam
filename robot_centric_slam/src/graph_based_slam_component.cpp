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

#include "robot_centric_slam/graph_based_slam_component.h"

namespace graphslam
{
GraphBasedSlamComponent::GraphBasedSlamComponent(const rclcpp::NodeOptions& options)
  : rclcpp::Node("graph_slam_node", options)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  map_array_msg_ = std::make_shared<vox_nav_slam_msgs::msg::MapArray>();

  // Define parameters
  declare_parameter("x_bound", icp_params_.x_bound);
  declare_parameter("y_bound", icp_params_.y_bound);
  declare_parameter("z_bound", icp_params_.z_bound);
  declare_parameter("downsample_voxel_size", icp_params_.downsample_voxel_size);
  declare_parameter("map_voxel_size", icp_params_.map_voxel_size);
  declare_parameter("max_icp_iter", icp_params_.max_icp_iter);
  declare_parameter("max_correspondence_distance", icp_params_.max_correspondence_distance);
  declare_parameter("method", icp_params_.method);
  declare_parameter("num_threads", icp_params_.num_threads);
  declare_parameter("max_num_targeted_clouds", icp_params_.max_num_targeted_clouds);
  declare_parameter("min_dist_to_update_map", icp_params_.min_dist_to_update_map);
  declare_parameter("publish_tf", icp_params_.publish_tf);
  declare_parameter("num_adjacent_pose_constraints", icp_params_.num_adjacent_pose_constraints);
  declare_parameter("debug", icp_params_.debug);

  get_parameter("x_bound", icp_params_.x_bound);
  get_parameter("y_bound", icp_params_.y_bound);
  get_parameter("z_bound", icp_params_.z_bound);
  get_parameter("downsample_voxel_size", icp_params_.downsample_voxel_size);
  get_parameter("max_icp_iter", icp_params_.max_icp_iter);
  get_parameter("map_voxel_size", icp_params_.map_voxel_size);
  get_parameter("max_correspondence_distance", icp_params_.max_correspondence_distance);
  get_parameter("method", icp_params_.method);
  get_parameter("num_threads", icp_params_.num_threads);
  get_parameter("max_num_targeted_clouds", icp_params_.max_num_targeted_clouds);
  get_parameter("min_dist_to_update_map", icp_params_.min_dist_to_update_map);
  get_parameter("publish_tf", icp_params_.publish_tf);
  get_parameter("num_adjacent_pose_constraints", icp_params_.num_adjacent_pose_constraints);
  get_parameter("debug", icp_params_.debug);

  // Init registration with given parameters
  registration_ = createRegistration(icp_params_.method, icp_params_.num_threads);
  if (!registration_)
  {
    RCLCPP_ERROR(get_logger(), "Failed to create registration");
    rclcpp::shutdown();
  }
  registration_->setMaximumIterations(icp_params_.max_icp_iter);
  registration_->setMaxCorrespondenceDistance(icp_params_.max_correspondence_distance);

  map_array_sub_ = create_subscription<vox_nav_slam_msgs::msg::MapArray>(
      "sub_maps", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      std::bind(&GraphBasedSlamComponent::mapArrayCallback, this, std::placeholders::_1));
  modified_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(  // NOLINT
      "modified_map", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  modified_map_array_pub_ = create_publisher<vox_nav_slam_msgs::msg::MapArray>(  // NOLINT
      "modified_map_array", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  modified_path_pub_ = create_publisher<nav_msgs::msg::Path>(  // NOLINT
      "modified_path", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  icp_pose_array_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(  // NOLINT
      "icp_pose_array", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  odom_pose_array_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(  // NOLINT
      "odom_pose_array", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  icp_uncertainty_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(  // NOLINT
      "icp_uncertainty_marker", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

  // Print parameters
  RCLCPP_INFO_STREAM(get_logger(), "x_bound: " << icp_params_.x_bound);
  RCLCPP_INFO_STREAM(get_logger(), "y_bound: " << icp_params_.y_bound);
  RCLCPP_INFO_STREAM(get_logger(), "z_bound: " << icp_params_.z_bound);
  RCLCPP_INFO_STREAM(get_logger(), "downsample_voxel_size: " << icp_params_.downsample_voxel_size);
  RCLCPP_INFO_STREAM(get_logger(), "map_voxel_size: " << icp_params_.map_voxel_size);
  RCLCPP_INFO_STREAM(get_logger(), "max_icp_iter: " << icp_params_.max_icp_iter);
  RCLCPP_INFO_STREAM(get_logger(), "max_correspondence_distance: " << icp_params_.max_correspondence_distance);
  RCLCPP_INFO_STREAM(get_logger(), "method: " << icp_params_.method);
  RCLCPP_INFO_STREAM(get_logger(), "num_threads: " << icp_params_.num_threads);
  RCLCPP_INFO_STREAM(get_logger(), "max_num_targeted_clouds: " << icp_params_.max_num_targeted_clouds);
  RCLCPP_INFO_STREAM(get_logger(), "num_adjacent_pose_constraints: " << icp_params_.num_adjacent_pose_constraints);
  RCLCPP_INFO_STREAM(get_logger(), "debug: " << icp_params_.debug);

  RCLCPP_INFO(get_logger(), "Creating...");
}

void GraphBasedSlamComponent::mapArrayCallback(const vox_nav_slam_msgs::msg::MapArray::ConstSharedPtr msg_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  map_array_msg_ = std::make_shared<vox_nav_slam_msgs::msg::MapArray>(*msg_ptr);

  if (!initial_map_array_received_ && map_array_msg_->submaps.size() > 0)
  {
    current_pose_.pose = map_array_msg_->submaps.back().odometry.pose.pose;
  }

  initial_map_array_received_ = true;
  is_map_array_updated_ = true;

  icpThread();
}

void GraphBasedSlamComponent::icpThread()
{
  // Optimize submaps in the loop until the node is shutdown
  if (map_array_msg_->submaps.size() <= icp_params_.num_adjacent_pose_constraints)
  {
    return;
  }

  // Optimize the subgraph and get refined transforms
  // this function also publishes uncertainty markers
  std::vector<Eigen::Matrix4f> refined_transforms;
  optimizeSubmapGraph(refined_transforms);

  // Publish modified path which is residing in the refined_transforms
  nav_msgs::msg::Path modified_path;
  modified_path.header = map_array_msg_->header;
  for (auto&& i : refined_transforms)
  {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = map_array_msg_->header;
    Eigen::Vector3f position = i.block<3, 1>(0, 3).cast<float>();
    Eigen::Matrix3f rot_mat = i.block<3, 3>(0, 0).cast<float>();
    Eigen::Quaternionf quat_eig(rot_mat);
    geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig.cast<double>());
    pose_stamped.pose.position.x = position.x();
    pose_stamped.pose.position.y = position.y();
    pose_stamped.pose.position.z = position.z();
    pose_stamped.pose.orientation = quat_msg;
    modified_path.poses.push_back(pose_stamped);
  }

  modified_path_pub_->publish(modified_path);

  // Publish modified map
  sensor_msgs::msg::PointCloud2 modified_map;
  pcl::PointCloud<pcl::PointXYZI>::Ptr modified_map_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  int num_submaps = map_array_msg_->submaps.size();
  for (int i = 0; i < icp_params_.max_num_targeted_clouds - 1; i++)
  {
    if (num_submaps - 1 - i < 0)
    {
      continue;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(map_array_msg_->submaps[num_submaps - 1 - i].cloud, *tmp_ptr);
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    Eigen::Affine3d submap_affine;
    tf2::fromMsg(map_array_msg_->submaps[num_submaps - 1 - i].pose, submap_affine);
    pcl::transformPointCloud(*tmp_ptr, *transformed_tmp_ptr, submap_affine.matrix());
    *modified_map_ptr += *transformed_tmp_ptr;

    if (transformed_tmp_ptr->size() == 0)
    {
      RCLCPP_WARN(get_logger(), "Submap %d is empty", num_submaps - 1 - i);
    }
  }
  pcl::toROSMsg(*modified_map_ptr, modified_map);
  modified_map.header = map_array_msg_->header;
  modified_map_pub_->publish(modified_map);
}

void GraphBasedSlamComponent::optimizeSubmapGraph(std::vector<Eigen::Matrix4f>& refined_transforms)
{
  auto start = std::chrono::high_resolution_clock::now();

  if (map_array_msg_->submaps.size() <= 1)
  {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;

  for (int i = 1; i < map_array_msg_->submaps.size(); i++)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(map_array_msg_->submaps[i].cloud, *source_cloud);
    Eigen::Affine3d current_pose_homogenous = Eigen::Affine3d::Identity();
    tf2::fromMsg(map_array_msg_->submaps[i].odometry.pose.pose, current_pose_homogenous);
    registration_->setInputSource(source_cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(map_array_msg_->submaps.front().cloud, *target_cloud);
    Eigen::Affine3d prev_pose_homogenous = Eigen::Affine3d::Identity();
    registration_->setInputTarget(target_cloud);

    // Calculate relative transform between current odom and previous odom
    Eigen::Affine3d curr_odom = Eigen::Affine3d::Identity();
    Eigen::Affine3d init_odom = Eigen::Affine3d::Identity();

    tf2::fromMsg(map_array_msg_->submaps[i].odometry.pose.pose, curr_odom);
    tf2::fromMsg(map_array_msg_->submaps.front().odometry.pose.pose, init_odom);

    auto odom_diff = (init_odom.inverse() * curr_odom).matrix().cast<float>();
    auto sim_trans = curr_odom.matrix().cast<float>() * odom_diff;

    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    registration_->align(*output_cloud, curr_odom.matrix().cast<float>());
    Eigen::Matrix4f final_transformation = registration_->getFinalTransformation();

    auto best_icp_measurement = final_transformation;

    // update the current pose with the best icp measurement
    Eigen::Affine3d best_icp_measurement_homogenous = Eigen::Affine3d::Identity();
    best_icp_measurement_homogenous.matrix() = best_icp_measurement.cast<double>();

    // update pose in map array
    map_array_msg_->submaps[i].pose = tf2::toMsg(current_pose_homogenous);
    // update odom in map array

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = current_pose_homogenous.matrix().block<3, 3>(0, 0).cast<float>();
    transform.block<3, 1>(0, 3) = current_pose_homogenous.matrix().block<3, 1>(0, 3).cast<float>();
    refined_transforms.push_back(transform);

    // Print the transformation between the two clouds
    std::cout << "Transformation between submap " << i << " and submap 0" << std::endl;
    std::cout << best_icp_measurement << std::endl;
  }

  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  RCLCPP_INFO_STREAM(get_logger(), "Cycle time: " << elapsed.count());
}

pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr
GraphBasedSlamComponent::createRegistration(std::string method,  // NOLINT
                                            int num_threads,     // NOLINT
                                            double voxel_resolution)
{
  if (method == "GICP")
  {
    auto gicp = pcl::make_shared<fast_gicp::FastGICP<pcl::PointXYZI, pcl::PointXYZI>>();
    gicp->setNumThreads(num_threads);
    return gicp;
  }
  else if (method == "VGICP")
  {
    auto vgicp = pcl::make_shared<fast_gicp::FastVGICP<pcl::PointXYZI, pcl::PointXYZI>>();
    vgicp->setNumThreads(num_threads);
    return vgicp;
  }
  else if (method == "VGICP_CUDA")
  {
    auto vgicp_cuda = pcl::make_shared<fast_gicp::FastVGICPCuda<pcl::PointXYZI, pcl::PointXYZI>>();
    vgicp_cuda->setResolution(voxel_resolution);
    vgicp_cuda->setNearestNeighborSearchMethod(fast_gicp::NearestNeighborMethod::GPU_RBF_KERNEL);
    return vgicp_cuda;
  }
  else if (method == "NDT_CUDA")
  {
    auto ndt = pcl::make_shared<fast_gicp::NDTCuda<pcl::PointXYZI, pcl::PointXYZI>>();
    ndt->setResolution(voxel_resolution);
    ndt->setDistanceMode(fast_gicp::NDTDistanceMode::P2D);
    ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT_RADIUS, 0.8);
    return ndt;
  }
  std::throw_with_nested(std::runtime_error("unknown registration method"));

  return nullptr;
}

}  // namespace graphslam

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(graphslam::GraphBasedSlamComponent)
