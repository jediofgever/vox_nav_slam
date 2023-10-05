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

#include "robot_centric_slam/fast_gicp_scan_matcher_component.hpp"
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <random>

namespace vox_nav_slam
{
FastGICPScanMatcher::FastGICPScanMatcher(const rclcpp::NodeOptions& options)  // NOLINT(modernize-pass-by-value)
  : Node("fast_gicp_scan_matcher", options)
{
  // Init all pointers
  temporal_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  latest_odom_msg_ = std::make_shared<nav_msgs::msg::Odometry>();
  current_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
  sub_maps_ = std::make_shared<vox_nav_slam_msgs::msg::MapArray>();
  path_ = std::make_shared<nav_msgs::msg::Path>();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  mutex_ = std::make_shared<std::mutex>();
  icp_thread_ = std::make_shared<std::thread>();
  icp_future_ = std::make_shared<std::future<void>>();

  // Init all pub and subs
  cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud_in", rclcpp::SensorDataQoS(), std::bind(&FastGICPScanMatcher::cloudCallback, this, std::placeholders::_1));
  odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom_in", rclcpp::SensorDataQoS(), std::bind(&FastGICPScanMatcher::odomCallback, this, std::placeholders::_1));
  map_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(  // NOLINT
      "map_cloud", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  sub_maps_pub_ = this->create_publisher<vox_nav_slam_msgs::msg::MapArray>(  // NOLINT
      "sub_maps", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(  // NOLINT
      "pose", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(  // NOLINT
      "path", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

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
  get_parameter("debug", icp_params_.debug);

  // Init registration with given parameters
  reg_ = createRegistration(icp_params_.method, icp_params_.num_threads);
  if (!reg_)
  {
    RCLCPP_ERROR(get_logger(), "Failed to create registration");
    rclcpp::shutdown();
  }
  reg_->setMaximumIterations(icp_params_.max_icp_iter);
  reg_->setMaxCorrespondenceDistance(icp_params_.max_correspondence_distance);
  reg_->setTransformationRotationEpsilon(0.78539816339);
  reg_->setEuclideanFitnessEpsilon(0.0000001);

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
  RCLCPP_INFO_STREAM(get_logger(), "debug: " << icp_params_.debug);

  RCLCPP_INFO(get_logger(), "Creating...");
}

FastGICPScanMatcher::~FastGICPScanMatcher()
{
  RCLCPP_INFO(get_logger(), "Destroying...");
}

void FastGICPScanMatcher::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  std::lock_guard<std::mutex> lock(*mutex_);
  latest_odom_msg_ = std::make_shared<nav_msgs::msg::Odometry>(*odom);
  is_odom_updated_ = true;
}

void FastGICPScanMatcher::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  std::lock_guard<std::mutex> lock(*mutex_);
  if (!is_odom_updated_)
  {
    RCLCPP_WARN(get_logger(),
                "Odometry is not updated yet!, currently this methods requires odometry to be published "
                "before the first pointcloud is received.");
    return;
  }

  sensor_msgs::msg::PointCloud2::SharedPtr output(new sensor_msgs::msg::PointCloud2);
  pcl_ros::transformPointCloud("base_link", *cloud, *output, *tf_buffer_);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*output, *cloud_pcl);
  // Crop cloud to ROI with crop box filter
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cropped(new pcl::PointCloud<pcl::PointXYZI>());

  // is this the first cloud?
  if (!is_first_cloud_received_)
  {
    is_first_cloud_received_ = true;
    Eigen::Affine3d current_map_origin = Eigen::Affine3d::Identity();
    // set the current map origin to the current odom pose
    // where we received the first cloud
    tf2::fromMsg(latest_odom_msg_->pose.pose, current_map_origin);
    current_map_origin_ = current_map_origin.matrix().cast<float>();
    initializeMap(cloud_pcl, cloud->header);
    return;
  }

  performRegistration(cloud_pcl, cloud->header);
}

void FastGICPScanMatcher::initializeMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,  // NOLINT
                                        const std_msgs::msg::Header& header)
{
  // First cloud, initialize map
  RCLCPP_INFO(get_logger(), "Initializing map");
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize(icp_params_.map_voxel_size,  // NOLINT
                         icp_params_.map_voxel_size,  // NOLINT
                         icp_params_.map_voxel_size);
  voxel_grid.filter(*cloud_downsampled);

  // crop box filter ROI around current pose
  pcl::CropBox<pcl::PointXYZI> crop_box_filter;
  crop_box_filter.setInputCloud(cloud_downsampled);
  crop_box_filter.setMin(Eigen::Vector4f(-icp_params_.x_bound, -icp_params_.y_bound, -icp_params_.z_bound, 1.0));
  crop_box_filter.setMax(Eigen::Vector4f(icp_params_.x_bound, icp_params_.y_bound, icp_params_.z_bound, 1.0));
  crop_box_filter.filter(*cloud_downsampled);

  // Transform cloud to recieved odom pose (external odmetry source)
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::transformPointCloud(*cloud_downsampled, *transformed_cloud_ptr, current_map_origin_);

  // This is the first cloud, so set it as the target cloud
  temporal_map_ = transformed_cloud_ptr;
  reg_->setInputTarget(transformed_cloud_ptr);

  // map array
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*cloud_downsampled, *cloud_msg_ptr);

  // push as the first submap
  vox_nav_slam_msgs::msg::SubMap submap;
  submap.header.frame_id = "odom";
  submap.header.stamp = now();
  submap.distance = 0;
  submap.pose = latest_odom_msg_->pose.pose;
  submap.optimized_pose = latest_odom_msg_->pose.pose;
  submap.cloud = *cloud_msg_ptr;
  submap.cloud.header.frame_id = "odom";
  submap.cloud.header.stamp = now();
  submap.odometry = *latest_odom_msg_;
  sub_maps_->header.frame_id = "odom";
  sub_maps_->header.stamp = now();
  sub_maps_->origin = current_pose_->pose;
  sub_maps_->submaps.push_back(submap);
  current_pose_->pose = latest_odom_msg_->pose.pose;
  map_cloud_pub_->publish(submap.cloud);
}

void FastGICPScanMatcher::performRegistration(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pcl,  // NOLINT
                                              const std_msgs::msg::Header& header)
{
  // Crop cloud to ROI with crop box filter
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cropped(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::CropBox<pcl::PointXYZI> crop_box_filter;
  crop_box_filter.setInputCloud(cloud_pcl);
  crop_box_filter.setMin(Eigen::Vector4f(-icp_params_.x_bound, -icp_params_.y_bound, -icp_params_.z_bound, 1.0));
  crop_box_filter.setMax(Eigen::Vector4f(icp_params_.x_bound, icp_params_.y_bound, icp_params_.z_bound, 1.0));
  crop_box_filter.filter(*cloud_cropped);

  // Downsample cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
  voxel_grid.setInputCloud(cloud_cropped);
  voxel_grid.setLeafSize(icp_params_.downsample_voxel_size,   // NOLINT
                         icp_params_.downsample_voxel_size,   // NOLINT
                         icp_params_.downsample_voxel_size);  // NOLINT
  voxel_grid.filter(*cloud_downsampled);

  // set the source cloud to current cloud
  reg_->setInputSource(cloud_downsampled);

  if (icp_future_->valid() && mapping_flag_)
  {
    // record the time taken by one cycle
    auto begin = std::chrono::steady_clock::now();

    auto status = icp_future_->wait_for(std::chrono::milliseconds(1));
    if (status == std::future_status::ready)
    {
      if (is_map_updated_ && temporal_map_->size() > 0)
      {
        is_map_updated_ = false;
        RCLCPP_INFO(get_logger(), "Targeted cloud size: %d", temporal_map_->size());

        // downsize targeted cloud
        pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
        voxel_grid.setInputCloud(temporal_map_);
        voxel_grid.setLeafSize(icp_params_.map_voxel_size,  // NOLINT
                               icp_params_.map_voxel_size,  // NOLINT
                               icp_params_.map_voxel_size);
        voxel_grid.filter(*temporal_map_);

        // print the size of the target cloud

        RCLCPP_INFO(get_logger(), "Targeted cloud size: %d", temporal_map_->size());

        reg_->setInputTarget(temporal_map_);
      }
      mapping_flag_ = false;
      icp_thread_->detach();
    }
  }

  // Calculate relative transform between current odom and previous odom
  Eigen::Affine3d latest_odom_homogenous = Eigen::Affine3d::Identity();
  tf2::fromMsg(latest_odom_msg_->pose.pose, latest_odom_homogenous);

  // Align clouds
  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  // calculate time elapsed for ICP alignment

  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  // max rotation epsilon is 45 degrees but use radians

  reg_->align(*output_cloud, latest_odom_homogenous.matrix().cast<float>());
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

  Eigen::Matrix4f final_transformation = reg_->getFinalTransformation();

  // Publish some visualizations to rviz
  Eigen::Vector3f position = final_transformation.block<3, 1>(0, 3).cast<float>();
  Eigen::Matrix3f rot_mat = final_transformation.block<3, 3>(0, 0).cast<float>();
  Eigen::Quaternionf quat_eig(rot_mat);
  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig.cast<double>());

  current_pose_->header.stamp = now();
  current_pose_->header.frame_id = "odom";
  current_pose_->pose.position.x = position.x();
  current_pose_->pose.position.y = position.y();
  current_pose_->pose.position.z = position.z();
  current_pose_->pose.orientation = quat_msg;

  pose_pub_->publish(*current_pose_);

  path_->poses.push_back(*current_pose_);
  path_->header.stamp = now();
  path_->header.frame_id = "odom";
  path_pub_->publish(*path_);

  // Publish tf
  if (icp_params_.publish_tf)
  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = now();
    transform_stamped.header.frame_id = "odom";
    transform_stamped.child_frame_id = "base_link";
    transform_stamped.transform.translation.x = position.x();
    transform_stamped.transform.translation.y = position.y();
    transform_stamped.transform.translation.z = position.z();
    transform_stamped.transform.rotation = quat_msg;
    broadcaster_->sendTransform(transform_stamped);
  }

  // Update map if translation is greater than threshold
  translation_from_previous_ = (position - previous_position_).norm();
  if (translation_from_previous_ >= icp_params_.min_dist_to_update_map && !mapping_flag_)
  {
    geometry_msgs::msg::PoseStamped current_pose_stamped;
    current_pose_stamped = *current_pose_;
    previous_position_ = position;  // NOLINT
    icp_task_ = std::packaged_task<void()>(std::bind(&FastGICPScanMatcher::insertScantoMap, this, cloud_downsampled,
                                                     output_cloud, final_transformation, current_pose_stamped,
                                                     header));  // NOLINT
    icp_future_ = std::make_shared<std::future<void>>(icp_task_.get_future());
    icp_thread_ = std::make_shared<std::thread>(std::move(std::ref(icp_task_)));
    mapping_flag_ = true;

    if (icp_params_.debug)
    {
      // Print elapsed time for ICP alignment
      int icp_alingment_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();

      RCLCPP_INFO(get_logger(), "ICP alignment took %d microseconds", icp_alingment_elapsed);
      RCLCPP_INFO(get_logger(), "Map size: %d", temporal_map_->size());
      // was alignment converged?
      if (reg_->hasConverged())
      {
        RCLCPP_INFO(get_logger(), "ICP has converged, score is %+.0e", reg_->getFitnessScore());
      }
      // print result transformation
      std::cout << "Transformation matrix:" << std::endl;
      std::cout << final_transformation << std::endl;

      // Raw initial guess
      std::cout << "Raw initial guess:" << std::endl;
      std::cout << "latest_odom_homogenous.matrix().cast<float>()" << std::endl;
      std::cout << latest_odom_homogenous.matrix().cast<float>() << std::endl;
    }
  }
}

void FastGICPScanMatcher::insertScantoMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                          const pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud,  // NOLINT
                                          const Eigen::Matrix4f& icp_estimate,                      // NOLINT
                                          const geometry_msgs::msg::PoseStamped& current_pose,
                                          const std_msgs::msg::Header& header)
{
  // Downsample cloud according to the voxel size of the map
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
  voxel_grid.setInputCloud(output_cloud);
  voxel_grid.setLeafSize(icp_params_.map_voxel_size,   // NOLINT
                         icp_params_.map_voxel_size,   // NOLINT
                         icp_params_.map_voxel_size);  // NOLINT
  voxel_grid.filter(*cloud_downsampled);

  temporal_map_->clear();

  // Add thge latest cloud to the targeted cloud
  *temporal_map_ += *cloud_downsampled;

  int num_submaps = sub_maps_->submaps.size();

  for (int i = 0; i < icp_params_.max_num_targeted_clouds - 1; i++)
  {
    if (num_submaps - 1 - i < 0)
    {
      continue;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(sub_maps_->submaps[num_submaps - 1 - i].cloud, *tmp_ptr);
    Eigen::Affine3d tmp_pose = Eigen::Affine3d::Identity();
    tf2::fromMsg(sub_maps_->submaps[num_submaps - 1 - i].pose, tmp_pose);
    Eigen::Matrix4f tmp_mat = tmp_pose.matrix().cast<float>();
    pcl::transformPointCloud(*tmp_ptr, *tmp_ptr, tmp_mat);

    *temporal_map_ += *tmp_ptr;

    if (tmp_ptr->size() == 0)
    {
      RCLCPP_WARN(get_logger(), "Submap %d is empty", num_submaps - 1 - i);
    }
  }

  sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*temporal_map_, *map_msg_ptr);
  map_msg_ptr->header.frame_id = "odom";
  map_msg_ptr->header.stamp = header.stamp;
  map_cloud_pub_->publish(*map_msg_ptr);

  // Use non-transformed cloud for submap
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*cloud, *cloud_msg_ptr);

  vox_nav_slam_msgs::msg::SubMap submap;
  submap.header.frame_id = "odom";
  submap.header.stamp = now();
  accumulated_translation_ += translation_from_previous_;
  submap.distance = accumulated_translation_;
  submap.pose = current_pose.pose;
  submap.optimized_pose = current_pose.pose;
  submap.cloud = *cloud_msg_ptr;
  submap.cloud.header.frame_id = "odom";
  submap.cloud.header.stamp = now();
  submap.odometry = *latest_odom_msg_;

  sub_maps_->header.stamp = now();
  sub_maps_->header.frame_id = "odom";
  sub_maps_->submaps.push_back(submap);

  // remove subclouds that are more than max_num_targeted_clouds
  if (num_submaps >= icp_params_.max_num_targeted_clouds)
  {
    sub_maps_->submaps.erase(sub_maps_->submaps.begin(),
                             sub_maps_->submaps.begin() + num_submaps - icp_params_.max_num_targeted_clouds + 1);
  }
  if (icp_params_.debug)
  {
    RCLCPP_INFO(get_logger(), "There is %d Targeted clouds.", sub_maps_->submaps.size());
  }

  // update the origin of the map
  sub_maps_->origin = sub_maps_->submaps[0].pose;
  sub_maps_pub_->publish(*sub_maps_);

  is_map_updated_ = true;
}

pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr FastGICPScanMatcher::createRegistration(std::string method,
                                                                                               int num_threads,
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
    vgicp_cuda->setNearestNeighborSearchMethod(fast_gicp::NearestNeighborMethod::GPU_BRUTEFORCE);

    return vgicp_cuda;
  }
  else if (method == "NDT_CUDA")
  {
    auto ndt = pcl::make_shared<fast_gicp::NDTCuda<pcl::PointXYZI, pcl::PointXYZI>>();
    ndt->setResolution(voxel_resolution);
    ndt->setDistanceMode(fast_gicp::NDTDistanceMode::P2D);
    ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT_RADIUS, voxel_resolution);
    return ndt;
  }
  else if (method == "PCL_NDT")
  {
    auto pcl_ndt = pcl::make_shared<pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>>();
    // Setting scale dependent NDT parameters
    // Setting minimum transformation difference for termination condition.
    pcl_ndt->setTransformationEpsilon(0.01);
    // Setting maximum step size for More-Thuente line search.
    pcl_ndt->setStepSize(0.1);
    // Setting Resolution of NDT grid structure (VoxelGridCovariance).
    pcl_ndt->setResolution(0.1);
    // Setting max number of registration iterations.
    pcl_ndt->setMaximumIterations(50);
    return pcl_ndt;
  }

  std::throw_with_nested(std::runtime_error("unknown registration method"));

  return nullptr;
}
}  // namespace vox_nav_slam

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(vox_nav_slam::FastGICPScanMatcher)
