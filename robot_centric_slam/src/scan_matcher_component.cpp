#include "robot_centric_slam/scan_matcher_component.hpp"

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
  get_parameter("debug", icp_params_.debug);

  set_parameter(rclcpp::Parameter("use_sim_time", true));

  // Init registration with given parameters
  reg_ = createRegistration(icp_params_.method, icp_params_.num_threads);
  if (!reg_)
  {
    RCLCPP_ERROR(get_logger(), "Failed to create registration");
    rclcpp::shutdown();
  }

  // Print parameters
  RCLCPP_INFO_STREAM(get_logger(), "x_bound " << icp_params_.x_bound);
  RCLCPP_INFO_STREAM(get_logger(), "y_bound " << icp_params_.y_bound);
  RCLCPP_INFO_STREAM(get_logger(), "z_bound " << icp_params_.z_bound);
  RCLCPP_INFO_STREAM(get_logger(), "downsample_voxel_size " << icp_params_.downsample_voxel_size);
  RCLCPP_INFO_STREAM(get_logger(), "map_voxel_size " << icp_params_.map_voxel_size);
  RCLCPP_INFO_STREAM(get_logger(), "max_icp_iter " << icp_params_.max_icp_iter);
  RCLCPP_INFO_STREAM(get_logger(), "max_correspondence_distance " << icp_params_.max_correspondence_distance);
  RCLCPP_INFO_STREAM(get_logger(), "method " << icp_params_.method);
  RCLCPP_INFO_STREAM(get_logger(), "num_threads " << icp_params_.num_threads);
  RCLCPP_INFO_STREAM(get_logger(), "max_num_targeted_clouds " << icp_params_.max_num_targeted_clouds);
  RCLCPP_INFO_STREAM(get_logger(), "debug " << icp_params_.debug);

  RCLCPP_INFO(get_logger(), "Creating...");
}

FastGICPScanMatcher::~FastGICPScanMatcher()
{
}

void FastGICPScanMatcher::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  std::lock_guard<std::mutex> lock(*mutex_);
  latest_odom_msg_ = std::make_shared<nav_msgs::msg::Odometry>(*odom);
}

void FastGICPScanMatcher::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  std::lock_guard<std::mutex> lock(*mutex_);
  // Convert to PCL
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*cloud, *cloud_pcl);

  // is this the first cloud?
  if (!is_first_cloud_received_)
  {
    // First cloud, initialize map
    RCLCPP_INFO(get_logger(), "Initializing map");
    is_first_cloud_received_ = true;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
    voxel_grid.setInputCloud(cloud_pcl);
    voxel_grid.setLeafSize(icp_params_.map_voxel_size,  // NOLINT
                           icp_params_.map_voxel_size,  // NOLINT
                           icp_params_.map_voxel_size);
    voxel_grid.filter(*cloud_downsampled);

    // Convert current pose to Eigen
    Eigen::Affine3d eig_affine = Eigen::Affine3d::Identity();
    tf2::fromMsg(current_pose_->pose, eig_affine);

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*cloud_downsampled, *transformed_cloud_ptr, eig_affine.matrix());
    reg_->setInputTarget(transformed_cloud_ptr);

    // map
    sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*transformed_cloud_ptr, *map_msg_ptr);

    // map array
    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*transformed_cloud_ptr, *cloud_msg_ptr);
    vox_nav_slam_msgs::msg::SubMap submap;
    submap.header = cloud->header;
    submap.distance = 0;
    submap.pose = current_pose_->pose;
    submap.cloud = *cloud_msg_ptr;
    sub_maps_->header = cloud->header;
    sub_maps_->submaps.push_back(submap);

    map_cloud_pub_->publish(submap.cloud);
  }

  performRegistration(cloud_pcl);
}

void FastGICPScanMatcher::performRegistration(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  if (icp_future_->valid() && mapping_flag_)
  {
    auto status = icp_future_->wait_for(std::chrono::milliseconds(1));
    if (status == std::future_status::ready)
    {
      if (is_map_updated_)
      {
        is_map_updated_ = false;

        // downsize targeted cloud
        pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
        voxel_grid.setInputCloud(targeted_cloud_);
        voxel_grid.setLeafSize(icp_params_.map_voxel_size,  // NOLINT
                               icp_params_.map_voxel_size,  // NOLINT
                               icp_params_.map_voxel_size);
        voxel_grid.filter(*targeted_cloud_);
        reg_->setInputTarget(targeted_cloud_);
      }
      mapping_flag_ = false;
      icp_thread_->detach();
    }
  }

  // Downsample cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize(icp_params_.downsample_voxel_size,  // NOLINT
                         icp_params_.downsample_voxel_size,  // NOLINT
                         icp_params_.downsample_voxel_size);
  voxel_grid.filter(*cloud_downsampled);
  reg_->setInputSource(cloud_downsampled);

  Eigen::Affine3d curr_pose_homogenous = Eigen::Affine3d::Identity();
  Eigen::Affine3d latest_odom_homogenous = Eigen::Affine3d::Identity();
  Eigen::Matrix4f sim_trans = Eigen::Matrix4f::Identity();

  tf2::fromMsg(current_pose_->pose, curr_pose_homogenous);
  tf2::fromMsg(latest_odom_msg_->pose.pose, latest_odom_homogenous);

  if (previous_odom_mat_ != Eigen::Matrix4f::Identity())
  {
    Eigen::Matrix4f odom_diff = previous_odom_mat_.inverse() * latest_odom_homogenous.matrix().cast<float>();
    sim_trans = curr_pose_homogenous.matrix().cast<float>() * odom_diff;
  }
  previous_odom_mat_ = latest_odom_homogenous.matrix().cast<float>();

  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  reg_->align(*output_cloud, sim_trans.cast<float>());

  Eigen::Matrix4f final_transformation = reg_->getFinalTransformation();

  Eigen::Vector3f position = final_transformation.block<3, 1>(0, 3).cast<float>();
  Eigen::Matrix3f rot_mat = final_transformation.block<3, 3>(0, 0).cast<float>();
  Eigen::Quaternionf quat_eig(rot_mat);
  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig.cast<double>());

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = now();
  transform_stamped.header.frame_id = "odom";
  transform_stamped.child_frame_id = "base_link";
  transform_stamped.transform.translation.x = position.x();
  transform_stamped.transform.translation.y = position.y();
  transform_stamped.transform.translation.z = position.z();
  transform_stamped.transform.rotation = quat_msg;
  // broadcaster_->sendTransform(transform_stamped);

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

  translation_from_previous_ = (position - previous_position_).norm();
  if (translation_from_previous_ >= icp_params_.min_dist_to_update_map && !mapping_flag_)
  {
    geometry_msgs::msg::PoseStamped current_pose_stamped;
    current_pose_stamped = *current_pose_;
    previous_position_ = position;  // NOLINT
    icp_task_ = std::packaged_task<void()>(std::bind(&FastGICPScanMatcher::insertScantoMap, this, cloud,
                                                     final_transformation, current_pose_stamped));  // NOLINT
    icp_future_ = std::make_shared<std::future<void>>(icp_task_.get_future());
    icp_thread_ = std::make_shared<std::thread>(std::move(std::ref(icp_task_)));
    mapping_flag_ = true;
  }
}

void FastGICPScanMatcher::insertScantoMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                          const Eigen::Matrix4f& pose,  // NOLINT
                                          const geometry_msgs::msg::PoseStamped& current_pose)
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
  }

  /* map array */
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*cloud_downsampled, *cloud_msg_ptr);

  vox_nav_slam_msgs::msg::SubMap submap;
  submap.header.frame_id = "odom";
  submap.header.stamp = current_pose.header.stamp;
  accumulated_translation_ += translation_from_previous_;
  submap.distance = accumulated_translation_;
  submap.pose = current_pose.pose;
  submap.cloud = *cloud_msg_ptr;
  submap.cloud.header.frame_id = "odom";
  sub_maps_->header.stamp = current_pose.header.stamp;
  sub_maps_->submaps.push_back(submap);
  sub_maps_pub_->publish(*sub_maps_);

  is_map_updated_ = true;

  publishMap();
}

void FastGICPScanMatcher::publishMap()
{
  // Accumulate submaps to create a map
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  for (auto& submap : sub_maps_->submaps)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr submap_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_submap_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(submap.cloud, *submap_cloud_ptr);

    Eigen::Affine3d affine;
    tf2::fromMsg(submap.pose, affine);
    pcl::transformPointCloud(*submap_cloud_ptr, *transformed_submap_cloud_ptr, affine.matrix());

    *map_ptr += *transformed_submap_cloud_ptr;
  }
  RCLCPP_INFO(this->get_logger(), "Map size: %d", map_ptr->size());

  sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*map_ptr, *map_msg_ptr);
  map_msg_ptr->header.frame_id = "odom";
  map_cloud_pub_->publish(*map_msg_ptr);
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
