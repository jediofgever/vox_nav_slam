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

namespace vox_nav_slam
{
FastGICPScanMatcher::FastGICPScanMatcher(const rclcpp::NodeOptions& options)  // NOLINT(modernize-pass-by-value)
  : Node("fast_gicp_scan_matcher", options)
{
  // Init all pointers
  targeted_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  source_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  latest_odom_msg_ = std::make_shared<nav_msgs::msg::Odometry>();
  current_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
  sub_maps_ = std::make_shared<vox_nav_slam_msgs::msg::MapArray>();
  path_ = std::make_shared<nav_msgs::msg::Path>();
  episode_end_ = std::make_shared<std_msgs::msg::Bool>();

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
  episode_end_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
      "end_of_episode", rclcpp::SensorDataQoS(),
      std::bind(&FastGICPScanMatcher::episodeEndCallback, this, std::placeholders::_1));
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

void FastGICPScanMatcher::episodeEndCallback(const std_msgs::msg::Bool::ConstSharedPtr msg)
{
  // Once episode ends, Reset the map, start everything from the beginning
  std::lock_guard<std::mutex> lock(*mutex_);
  if (msg->data)
  {
    RCLCPP_INFO(get_logger(), "Episode ended, resetting the map");
    // save the map to a file before resetting the map
    std::string map_file_name = "map.pcd";
    pcl::io::savePCDFileASCII(map_file_name, *targeted_cloud_);
    RCLCPP_INFO(get_logger(), "Map saved to %s", map_file_name.c_str());

    is_first_cloud_received_ = false;
    is_map_updated_ = false;
    mapping_flag_ = false;
    is_odom_updated_ = false;
    previous_odom_mat_ = Eigen::Matrix4f::Identity();
    accumulated_translation_ = 0;
    translation_from_previous_ = 0;
    current_map_origin_ = Eigen::Matrix4f::Identity();
    targeted_cloud_->clear();
    sub_maps_->submaps.clear();
    path_->poses.clear();
    path_->header.frame_id = "map";
    path_->header.stamp = now();
    sub_maps_->header.frame_id = "map";
    sub_maps_->header.stamp = now();
    sub_maps_->origin = geometry_msgs::msg::Pose();
    sub_maps_pub_->publish(*sub_maps_);
    path_pub_->publish(*path_);
  }
}

void FastGICPScanMatcher::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  std::lock_guard<std::mutex> lock(*mutex_);
  if (!is_odom_updated_)
  {
    RCLCPP_WARN(get_logger(), "Odometry is not updated yet");
    return;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*cloud, *cloud_pcl);
  // Crop cloud to ROI with crop box filter
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cropped(new pcl::PointCloud<pcl::PointXYZI>());

  // is this the first cloud?
  if (!is_first_cloud_received_)
  {
    is_first_cloud_received_ = true;
    Eigen::Affine3d current_map_origin = Eigen::Affine3d::Identity();
    tf2::fromMsg(latest_odom_msg_->pose.pose, current_map_origin);
    current_map_origin_ = current_map_origin.matrix().cast<float>();
    initializeMap(cloud_pcl, cloud->header);

    return;
  }

  pcl::CropBox<pcl::PointXYZI> crop_box_filter;
  crop_box_filter.setInputCloud(cloud_pcl);
  crop_box_filter.setMin(Eigen::Vector4f(-icp_params_.x_bound, -icp_params_.y_bound, -icp_params_.z_bound, 1.0));
  crop_box_filter.setMax(Eigen::Vector4f(icp_params_.x_bound, icp_params_.y_bound, icp_params_.z_bound, 1.0));
  crop_box_filter.filter(*cloud_pcl);

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

  current_pose_->pose = latest_odom_msg_->pose.pose;
  // crop box filter ROI around current pose
  pcl::CropBox<pcl::PointXYZI> crop_box_filter;
  crop_box_filter.setInputCloud(cloud_downsampled);
  crop_box_filter.setMin(Eigen::Vector4f(-icp_params_.x_bound, -icp_params_.y_bound, -icp_params_.z_bound, 1.0));
  crop_box_filter.setMax(Eigen::Vector4f(icp_params_.x_bound, icp_params_.y_bound, icp_params_.z_bound, 1.0));
  crop_box_filter.filter(*cloud_downsampled);

  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::transformPointCloud(*cloud_downsampled, *transformed_cloud_ptr, current_map_origin_);

  reg_->setInputTarget(transformed_cloud_ptr);

  // map
  sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*transformed_cloud_ptr, *map_msg_ptr);

  // map array
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*transformed_cloud_ptr, *cloud_msg_ptr);
  vox_nav_slam_msgs::msg::SubMap submap;
  submap.header.frame_id = "map";
  submap.header.stamp = now();
  submap.distance = 0;
  submap.pose = geometry_msgs::msg::Pose();
  submap.cloud = *cloud_msg_ptr;
  submap.cloud.header.frame_id = "map";
  submap.cloud.header.stamp = now();
  submap.odometry = *latest_odom_msg_;
  sub_maps_->header.frame_id = "map";
  sub_maps_->header.stamp = now();
  sub_maps_->origin = current_pose_->pose;
  sub_maps_->submaps.push_back(submap);

  map_cloud_pub_->publish(submap.cloud);
}

void FastGICPScanMatcher::performRegistration(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,  // NOLINT
                                              const std_msgs::msg::Header& header)
{
  if (icp_future_->valid() && mapping_flag_)
  {
    // record the time taken by one cycle
    auto begin = std::chrono::steady_clock::now();

    auto status = icp_future_->wait_for(std::chrono::milliseconds(1));
    if (status == std::future_status::ready)
    {
      if (is_map_updated_ && targeted_cloud_->size() > 0)
      {
        is_map_updated_ = false;
        RCLCPP_INFO(get_logger(), "Targeted cloud size: %d", targeted_cloud_->size());

        // crop the map to the ROI with crop box filter around current pose
        /*pcl::CropBox<pcl::PointXYZI> crop_box_filter;
        crop_box_filter.setInputCloud(targeted_cloud_);
        crop_box_filter.setMin(Eigen::Vector4f(-icp_params_.x_bound, -icp_params_.y_bound, -icp_params_.z_bound, 1.0));
        crop_box_filter.setMax(Eigen::Vector4f(icp_params_.x_bound, icp_params_.y_bound, icp_params_.z_bound, 1.0));
        crop_box_filter.filter(*targeted_cloud_);*/

        // downsize targeted cloud
        pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
        voxel_grid.setInputCloud(targeted_cloud_);
        voxel_grid.setLeafSize(icp_params_.map_voxel_size,  // NOLINT
                               icp_params_.map_voxel_size,  // NOLINT
                               icp_params_.map_voxel_size);
        voxel_grid.filter(*targeted_cloud_);

        // print the size of the target cloud

        RCLCPP_INFO(get_logger(), "Targeted cloud size: %d", targeted_cloud_->size());

        reg_->setInputTarget(targeted_cloud_);
      }
      mapping_flag_ = false;
      icp_thread_->detach();
    }
  }
  // Crop cloud to ROI with crop box filter
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cropped(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::CropBox<pcl::PointXYZI> crop_box_filter;
  crop_box_filter.setInputCloud(cloud);
  crop_box_filter.setMin(Eigen::Vector4f(-icp_params_.x_bound, -icp_params_.y_bound, -icp_params_.z_bound, 1.0));
  crop_box_filter.setMax(Eigen::Vector4f(icp_params_.x_bound, icp_params_.y_bound, icp_params_.z_bound, 1.0));
  crop_box_filter.filter(*cloud_cropped);

  // Downsample cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
  voxel_grid.setInputCloud(cloud_cropped);
  voxel_grid.setLeafSize(icp_params_.downsample_voxel_size,  // NOLINT
                         icp_params_.downsample_voxel_size,  // NOLINT
                         icp_params_.downsample_voxel_size);
  voxel_grid.filter(*cloud_downsampled);

  // set the source cloud to current cloud
  reg_->setInputSource(cloud_downsampled);

  // Calculate relative transform between current odom and previous odom
  Eigen::Affine3d curr_pose_homogenous = Eigen::Affine3d::Identity();
  Eigen::Affine3d latest_odom_homogenous = Eigen::Affine3d::Identity();
  Eigen::Matrix4f sim_trans = Eigen::Matrix4f::Identity();
  tf2::fromMsg(current_pose_->pose, curr_pose_homogenous);
  tf2::fromMsg(latest_odom_msg_->pose.pose, latest_odom_homogenous);

  sim_trans = curr_pose_homogenous.matrix().cast<float>();

  if (previous_odom_mat_ != Eigen::Matrix4f::Identity())
  {
    Eigen::Matrix4f odom_diff = previous_odom_mat_.inverse() * latest_odom_homogenous.matrix().cast<float>();
    sim_trans = sim_trans * odom_diff;
  }
  previous_odom_mat_ = latest_odom_homogenous.matrix().cast<float>();

  // Align clouds
  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  // calculate time elapsed for ICP alignment

  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  reg_->align(*output_cloud, sim_trans.cast<float>());
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

  Eigen::Matrix4f final_transformation = reg_->getFinalTransformation();

  // Publish some visualizations to rviz
  Eigen::Vector3f position = final_transformation.block<3, 1>(0, 3).cast<float>();
  Eigen::Matrix3f rot_mat = final_transformation.block<3, 3>(0, 0).cast<float>();
  Eigen::Quaternionf quat_eig(rot_mat);
  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig.cast<double>());

  current_pose_->header.stamp = now();
  current_pose_->header.frame_id = "map";
  current_pose_->pose.position.x = position.x();
  current_pose_->pose.position.y = position.y();
  current_pose_->pose.position.z = position.z();
  current_pose_->pose.orientation = quat_msg;

  pose_pub_->publish(*current_pose_);
  // eigen::matrix to geometry_msgs::msg::pose
  /*geometry_msgs::msg::PoseStamped current_map_origin_pose_msg;
  Eigen::Affine3d current_pose_homogenous = Eigen::Affine3d::Identity();
  current_pose_homogenous.matrix() = current_map_origin_.cast<double>();
  current_map_origin_pose_msg.pose = tf2::toMsg(current_pose_homogenous);
  current_map_origin_pose_msg.header.frame_id = "map";
  current_map_origin_pose_msg.header.stamp = now();
  pose_pub_->publish(current_map_origin_pose_msg);*/

  path_->poses.push_back(*current_pose_);
  path_->header.stamp = now();
  path_->header.frame_id = "map";
  path_pub_->publish(*path_);

  sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*targeted_cloud_, *map_msg_ptr);
  map_msg_ptr->header.frame_id = "map";
  map_msg_ptr->header.stamp = header.stamp;
  map_cloud_pub_->publish(*map_msg_ptr);

  // Publish tf
  if (icp_params_.publish_tf)
  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = now();
    transform_stamped.header.frame_id = "map";
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
    icp_task_ = std::packaged_task<void()>(std::bind(&FastGICPScanMatcher::insertScantoMap, this, cloud,
                                                     final_transformation, current_pose_stamped, header));  // NOLINT
    icp_future_ = std::make_shared<std::future<void>>(icp_task_.get_future());
    icp_thread_ = std::make_shared<std::thread>(std::move(std::ref(icp_task_)));
    mapping_flag_ = true;

    if (icp_params_.debug)
    {
      // Print elapsed time for ICP alignment
      int icp_alingment_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();

      RCLCPP_INFO(get_logger(), "ICP alignment took %d microseconds", icp_alingment_elapsed);
      RCLCPP_INFO(get_logger(), "Map size: %d", targeted_cloud_->size());
      // was alignment converged?
      if (reg_->hasConverged())
      {
        RCLCPP_INFO(get_logger(), "ICP has converged, score is %+.0e", reg_->getFitnessScore());
      }
      std::cout << "Current Pose homogenenous:" << std::endl;
      std::cout << curr_pose_homogenous.matrix() << std::endl;
      // Print the initial guess
      std::cout << "Initial guess:" << std::endl;
      std::cout << sim_trans << std::endl;
      // print result transformation
      std::cout << "Transformation matrix:" << std::endl;
      std::cout << final_transformation << std::endl;
    }
  }
}

void FastGICPScanMatcher::insertScantoMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                          const Eigen::Matrix4f& pose,  // NOLINT
                                          const geometry_msgs::msg::PoseStamped& current_pose,
                                          const std_msgs::msg::Header& header)
{
  // Downsample cloud according to the voxel size of the map
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize(icp_params_.map_voxel_size,  // NOLINT
                         icp_params_.map_voxel_size,  // NOLINT
                         icp_params_.map_voxel_size);
  voxel_grid.filter(*cloud_downsampled);

  // Transform cloud to the map frame
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::transformPointCloud(*cloud_downsampled, *cloud_transformed, pose);

  targeted_cloud_->clear();
  *targeted_cloud_ += *cloud_transformed;

  int num_submaps = sub_maps_->submaps.size();
  for (int i = 0; i < icp_params_.max_num_targeted_clouds - 1; i++)
  {
    if (num_submaps - 1 - i < 0)
    {
      continue;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(sub_maps_->submaps[num_submaps - 1 - i].cloud, *tmp_ptr);
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    Eigen::Affine3d submap_affine;
    tf2::fromMsg(sub_maps_->submaps[num_submaps - 1 - i].pose, submap_affine);
    pcl::transformPointCloud(*tmp_ptr, *transformed_tmp_ptr, submap_affine.matrix());
    *targeted_cloud_ += *transformed_tmp_ptr;

    if (transformed_tmp_ptr->size() == 0)
    {
      RCLCPP_WARN(get_logger(), "Submap %d is empty", num_submaps - 1 - i);
    }
  }

  sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*cloud_downsampled, *cloud_msg_ptr);

  vox_nav_slam_msgs::msg::SubMap submap;
  submap.header.frame_id = "map";
  submap.header.stamp = now();
  accumulated_translation_ += translation_from_previous_;
  submap.distance = accumulated_translation_;
  submap.pose = current_pose.pose;
  submap.cloud = *cloud_msg_ptr;
  submap.cloud.header.frame_id = "map";
  submap.cloud.header.stamp = now();
  submap.odometry = *latest_odom_msg_;
  sub_maps_->header.stamp = now();
  sub_maps_->header.frame_id = "map";
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

  sub_maps_->origin = sub_maps_->submaps[0].odometry.pose.pose;
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
}  // namespace vox_nav_slam

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(vox_nav_slam::FastGICPScanMatcher)
