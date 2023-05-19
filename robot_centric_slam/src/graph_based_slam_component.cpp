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

  icp_thread_ = std::make_shared<std::thread>(std::bind(&GraphBasedSlamComponent::icpThread, this));

  RCLCPP_INFO(get_logger(), "Creating...");
}

void GraphBasedSlamComponent::mapArrayCallback(const vox_nav_slam_msgs::msg::MapArray::ConstSharedPtr msg_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  map_array_msg_ = std::make_shared<vox_nav_slam_msgs::msg::MapArray>(*msg_ptr);
  initial_map_array_received_ = true;
  is_map_array_updated_ = true;
}

void GraphBasedSlamComponent::icpThread()
{
  // Optimize submaps in the loop until the node is shutdown
  while (rclcpp::ok())
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (map_array_msg_->submaps.size() <= icp_params_.num_adjacent_pose_constraints)
    {
      continue;
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
    for (int i = icp_params_.num_adjacent_pose_constraints; i < map_array_msg_->submaps.size(); i++)
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(map_array_msg_->submaps[i].cloud, *tmp_ptr);
      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>());
      Eigen::Affine3d submap_affine;
      tf2::fromMsg(map_array_msg_->submaps[i].pose, submap_affine);
      pcl::transformPointCloud(*tmp_ptr, *transformed_tmp_ptr, submap_affine.matrix());
      *modified_map_ptr += *transformed_tmp_ptr;
    }
    pcl::toROSMsg(*modified_map_ptr, modified_map);
    modified_map.header = map_array_msg_->header;
    modified_map_pub_->publish(modified_map);
  }
}

void GraphBasedSlamComponent::optimizeSubmapGraph(std::vector<Eigen::Matrix4f>& refined_transforms)
{
  // Optimize submaps
  // The accumulated submaps shall be greater than num_adjacent_pose_constraints
  // Publish spheres to indicate uncertainty
  visualization_msgs::msg::MarkerArray marker_array;
  for (int i = icp_params_.num_adjacent_pose_constraints; i < map_array_msg_->submaps.size(); i++)
  {
    Eigen::Affine3d curr_ith_cloud_transform;
    tf2::fromMsg(map_array_msg_->submaps[i].odometry.pose.pose, curr_ith_cloud_transform);
    pcl::PointCloud<pcl::PointXYZI>::Ptr curr_ith_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(map_array_msg_->submaps[i].cloud, *curr_ith_cloud);
    pcl::transformPointCloud(*curr_ith_cloud, *curr_ith_cloud, curr_ith_cloud_transform.matrix().cast<float>());
    // Set source pointcloud
    registration_->setInputSource(curr_ith_cloud);

    Eigen::Vector3d odom_position = curr_ith_cloud_transform.matrix().block<3, 1>(0, 3);
    Eigen::Matrix3d odom_rot_mat = curr_ith_cloud_transform.matrix().block<3, 3>(0, 0);
    Eigen::Vector3d odom_rpy = odom_rot_mat.cast<double>().eulerAngles(0, 1, 2);
    Eigen::VectorXd odom_vec_joined(odom_position.size() + odom_rpy.size());
    odom_vec_joined << odom_position, odom_rpy;

    // From a given submap, find the previous submaps (num_adjacent_pose_constraints) to be used for ICP
    // This process is done continuously
    std::vector<Eigen::VectorXd> icp_measurements_vec;
    std::vector<Eigen::Matrix4f> icp_measurements_mat;

    double min_icp_cost = std::numeric_limits<double>::max();
    int min_icp_cost_index = -1;
    int counter = 0;

    for (int j = i - icp_params_.num_adjacent_pose_constraints; j < i; j++)
    {
      Eigen::Affine3d prev_jth_transform;
      tf2::fromMsg(map_array_msg_->submaps[j].odometry.pose.pose, prev_jth_transform);

      Eigen::Matrix4f diff =
          prev_jth_transform.inverse().matrix().cast<float>() * curr_ith_cloud_transform.matrix().cast<float>();

      pcl::PointCloud<pcl::PointXYZI>::Ptr prev_jth_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(map_array_msg_->submaps[j].cloud, *prev_jth_cloud);
      pcl::transformPointCloud(*prev_jth_cloud, *prev_jth_cloud, prev_jth_transform.matrix().cast<float>());

      // Set target pointcloud
      registration_->setInputTarget(prev_jth_cloud);

      pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);

      registration_->align(*output_cloud, curr_ith_cloud_transform.matrix().cast<float>());
      Eigen::Matrix4f final_transformation = registration_->getFinalTransformation();

      Eigen::Vector3d icp_position = final_transformation.block<3, 1>(0, 3).cast<double>();
      Eigen::Matrix3d icp_rot_mat = final_transformation.block<3, 3>(0, 0).cast<double>();
      Eigen::Vector3d icp_rpy = icp_rot_mat.eulerAngles(0, 1, 2);

      Eigen::VectorXd icp_vec_joined(icp_position.size() + icp_rpy.size());
      icp_vec_joined << icp_position, icp_rpy;

      icp_measurements_vec.push_back(icp_vec_joined);
      icp_measurements_mat.push_back(final_transformation);

      min_icp_cost = std::min(min_icp_cost, registration_->getFitnessScore());
      if (min_icp_cost == registration_->getFitnessScore())
      {
        min_icp_cost_index = counter;
      }
      counter++;
    }

    Eigen::VectorXd mean = Eigen::VectorXd::Zero(icp_measurements_vec[0].size());
    for (int m = 0; m < icp_measurements_vec.size(); m++)
    {
      mean += icp_measurements_vec[m];
    }
    mean /= icp_measurements_vec.size();

    Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(icp_measurements_vec[0].size(), icp_measurements_vec[0].size());
    for (int m = 0; m < icp_measurements_vec.size(); m++)
    {
      covariance += (icp_measurements_vec[m] - mean) * (icp_measurements_vec[m] - mean).transpose();
    }
    covariance /= icp_measurements_vec.size();

    auto best_icp_measurement = icp_measurements_mat[min_icp_cost_index];

    // Publish some visualizations to rviz
    Eigen::Vector3f position = best_icp_measurement.block<3, 1>(0, 3).cast<float>();
    Eigen::Matrix3f rot_mat = best_icp_measurement.block<3, 3>(0, 0).cast<float>();
    Eigen::Quaternionf quat_eig(rot_mat);
    geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig.cast<double>());

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = rot_mat;
    transform.block<3, 1>(0, 3) = position;
    refined_transforms.push_back(transform);

    map_array_msg_->submaps[i].pose.position.x = position(0);
    map_array_msg_->submaps[i].pose.position.y = position(1);
    map_array_msg_->submaps[i].pose.position.z = position(2);
    map_array_msg_->submaps[i].pose.orientation = quat_msg;
    map_array_msg_->submaps[i].odometry.pose.covariance[0] = std::max(10.0 * covariance(0, 0), 0.05);
    map_array_msg_->submaps[i].odometry.pose.covariance[7] = std::max(10.0 * covariance(1, 1), 0.05);
    map_array_msg_->submaps[i].odometry.pose.covariance[14] = std::max(10.0 * covariance(2, 2), 0.05);
    map_array_msg_->submaps[i].odometry.pose.covariance[21] = std::max(10.0 * covariance(3, 3), 0.05);
    map_array_msg_->submaps[i].odometry.pose.covariance[28] = std::max(10.0 * covariance(4, 4), 0.05);
    map_array_msg_->submaps[i].odometry.pose.covariance[35] = std::max(10.0 * covariance(5, 5), 0.05);

    visualization_msgs::msg::Marker marker;

    // Publish uncertainty marker
    marker.header = map_array_msg_->header;
    marker.ns = "icp_uncertainty";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = map_array_msg_->submaps[i].pose;
    marker.scale.x = std::max(10.0 * covariance(0, 0), 0.05);
    marker.scale.y = std::max(10.0 * covariance(1, 1), 0.05);
    marker.scale.z = std::max(10.0 * covariance(2, 2), 0.05);
    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker_array.markers.push_back(marker);
  }
  icp_uncertainty_marker_pub_->publish(marker_array);
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
