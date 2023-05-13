#include "robot_centric_slam/gtsam_component.h"
#include <chrono>

using namespace std::chrono_literals;

namespace vox_nav_slam
{
GTSAMComponent::GTSAMComponent(const rclcpp::NodeOptions& options) : rclcpp::Node("graph_slam_node", options)
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

  voxelgrid_ = std::make_shared<pcl::VoxelGrid<pcl::PointXYZI>>();
  voxelgrid_->setLeafSize(icp_params_.downsample_voxel_size, icp_params_.downsample_voxel_size,
                          icp_params_.downsample_voxel_size);

  map_array_sub_ = create_subscription<vox_nav_slam_msgs::msg::MapArray>(
      "sub_maps", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      std::bind(&GTSAMComponent::mapArrayCallback, this, std::placeholders::_1));
  modified_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(  // NOLINT
      "modified_map", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  modified_map_array_pub_ = create_publisher<vox_nav_slam_msgs::msg::MapArray>(  // NOLINT
      "modified_map_array", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  modified_path_pub_ = create_publisher<nav_msgs::msg::Path>(  // NOLINT
      "modified_path", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

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

GTSAMComponent::~GTSAMComponent()
{
}

void GTSAMComponent::mapArrayCallback(const vox_nav_slam_msgs::msg::MapArray::ConstSharedPtr msg_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  map_array_msg_ = std::make_shared<vox_nav_slam_msgs::msg::MapArray>(*msg_ptr);
  initial_map_array_received_ = true;
  is_map_array_updated_ = true;

  doPoseAdjustment(*map_array_msg_);
}

void GTSAMComponent::doPoseAdjustment(vox_nav_slam_msgs::msg::MapArray& map_array_msg)
{
  RCLCPP_INFO(get_logger(), "Doing pose adjustment...");
  double lag = 2.0;

  // Create a fixed lag smoother
  // The Batch version uses Levenberg-Marquardt to perform the nonlinear optimization
  gtsam::BatchFixedLagSmoother smootherBatch(lag);
  // The Incremental version uses iSAM2 to perform the nonlinear optimization
  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.0;  // Set the relin threshold to zero such that the batch estimate is recovered
  parameters.relinearizeSkip = 1;         // Relinearize every time
  gtsam::IncrementalFixedLagSmoother smootherISAM2(lag, parameters);

  // Create containers to store the factors and linearization points that
  // will be sent to the smoothers
  gtsam::NonlinearFactorGraph newFactors;
  gtsam::Values newValues;
  gtsam::FixedLagSmoother::KeyTimestampMap newTimestamps;

  auto submap_size = map_array_msg.submaps.size();

  // Create a pose prior factor for the first submap
  gtsam::Pose3 first_pose = gtsam::Pose3(
      gtsam::Rot3::Quaternion(map_array_msg.submaps[0].pose.orientation.w, map_array_msg.submaps[0].pose.orientation.x,
                              map_array_msg.submaps[0].pose.orientation.y, map_array_msg.submaps[0].pose.orientation.z),
      gtsam::Point3(map_array_msg.submaps[0].pose.position.x, map_array_msg.submaps[0].pose.position.y,
                    map_array_msg.submaps[0].pose.position.z));
  gtsam::noiseModel::Diagonal::shared_ptr priorNoise =
      gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01).finished());
  newFactors.add(gtsam::PriorFactor<gtsam::Pose3>(0, first_pose, priorNoise));
  newTimestamps.insert(std::make_pair(0, 0.0));

  auto initial_time = map_array_msg.submaps[0].header.stamp.sec + map_array_msg.submaps[0].header.stamp.nanosec * 1e-9;

  for (size_t i = 1; i < submap_size; i++)
  {
    auto prev_submap = map_array_msg.submaps[i - 1];
    auto curr_submap = map_array_msg.submaps[i];

    // Create a new pose prior factor
    gtsam::Pose3 prev_pose = gtsam::Pose3(
        gtsam::Rot3::Quaternion(prev_submap.pose.orientation.w, prev_submap.pose.orientation.x,
                                prev_submap.pose.orientation.y, prev_submap.pose.orientation.z),
        gtsam::Point3(prev_submap.pose.position.x, prev_submap.pose.position.y, prev_submap.pose.position.z));

    gtsam::Pose3 curr_pose = gtsam::Pose3(
        gtsam::Rot3::Quaternion(curr_submap.pose.orientation.w, curr_submap.pose.orientation.x,
                                curr_submap.pose.orientation.y, curr_submap.pose.orientation.z),
        gtsam::Point3(curr_submap.pose.position.x, curr_submap.pose.position.y, curr_submap.pose.position.z));

    gtsam::noiseModel::Diagonal::shared_ptr priorNoise =
        gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01).finished());
        
    newFactors.add(gtsam::BetweenFactor<gtsam::Pose3>(i - 1, i, prev_pose.between(curr_pose), priorNoise));

    // Time stamp for the new factor
    auto curr_time = curr_submap.header.stamp.sec + curr_submap.header.stamp.nanosec * 1e-9;
    newTimestamps.insert(std::make_pair(i, curr_time - initial_time));
  }
}

pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr GTSAMComponent::createRegistration(std::string method,  // NOLINT
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

}  // namespace vox_nav_slam

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(vox_nav_slam::GTSAMComponent)
