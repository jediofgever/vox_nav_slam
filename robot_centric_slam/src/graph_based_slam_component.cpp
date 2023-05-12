#include "robot_centric_slam/graph_based_slam_component.h"
#include <chrono>

using namespace std::chrono_literals;

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

  voxelgrid_ = std::make_shared<pcl::VoxelGrid<pcl::PointXYZI>>();
  voxelgrid_->setLeafSize(icp_params_.downsample_voxel_size, icp_params_.downsample_voxel_size,
                          icp_params_.downsample_voxel_size);

  map_array_sub_ = create_subscription<vox_nav_slam_msgs::msg::MapArray>(
      "sub_maps", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      std::bind(&GraphBasedSlamComponent::mapArrayCallback, this, std::placeholders::_1));
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

void GraphBasedSlamComponent::mapArrayCallback(const vox_nav_slam_msgs::msg::MapArray::ConstSharedPtr msg_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  map_array_msg_ = std::make_shared<vox_nav_slam_msgs::msg::MapArray>(*msg_ptr);
  initial_map_array_received_ = true;
  is_map_array_updated_ = true;

  doPoseAdjustment(*map_array_msg_);
}

void GraphBasedSlamComponent::doPoseAdjustment(vox_nav_slam_msgs::msg::MapArray map_array_msg)
{
  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linear_solver =
      g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linear_solver)));

  optimizer.setAlgorithm(solver);

  int submaps_size = map_array_msg.submaps.size();
  Eigen::Matrix<double, 6, 6> info_mat = Eigen::Matrix<double, 6, 6>::Identity();
  for (int i = 0; i < submaps_size; i++)
  {
    Eigen::Affine3d affine;
    Eigen::fromMsg(map_array_msg.submaps[i].pose, affine);
    Eigen::Isometry3d pose(affine.matrix());

    g2o::VertexSE3* vertex_se3 = new g2o::VertexSE3();
    vertex_se3->setId(i);
    vertex_se3->setEstimate(pose);
    if (i == 0)
    {
      vertex_se3->setFixed(true);
    }
    optimizer.addVertex(vertex_se3);

    if (i > icp_params_.num_adjacent_pose_constraints)
    {
      for (int j = 0; j < icp_params_.num_adjacent_pose_constraints; j++)
      {
        Eigen::Affine3d pre_affine;
        Eigen::fromMsg(map_array_msg.submaps[i - icp_params_.num_adjacent_pose_constraints + j].pose, pre_affine);
        Eigen::Isometry3d pre_pose(pre_affine.matrix());
        Eigen::Isometry3d relative_pose = pre_pose.inverse() * pose;
        g2o::EdgeSE3* edge_se3 = new g2o::EdgeSE3();
        edge_se3->setMeasurement(relative_pose);
        edge_se3->setInformation(info_mat);
        edge_se3->vertices()[0] = optimizer.vertex(i - icp_params_.num_adjacent_pose_constraints + j);
        edge_se3->vertices()[1] = optimizer.vertex(i);
        optimizer.addEdge(edge_se3);
      }
    }
  }
  /* loop edge */
  /*for (auto loop_edge : loop_edges_)
  {
    g2o::EdgeSE3* edge_se3 = new g2o::EdgeSE3();
    edge_se3->setMeasurement(loop_edge.relative_pose);
    edge_se3->setInformation(info_mat);
    edge_se3->vertices()[0] = optimizer.vertex(loop_edge.pair_id.first);
    edge_se3->vertices()[1] = optimizer.vertex(loop_edge.pair_id.second);
    optimizer.addEdge(edge_se3);
  }*/

  optimizer.initializeOptimization();
  optimizer.optimize(10);
  optimizer.save("pose_graph.g2o");

  /* modified_map publish */
  std::cout << "modified_map publish" << std::endl;
  vox_nav_slam_msgs::msg::MapArray modified_map_array_msg;
  modified_map_array_msg.header = map_array_msg.header;
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  for (int i = 0; i < submaps_size; i++)
  {
    g2o::VertexSE3* vertex_se3 = static_cast<g2o::VertexSE3*>(optimizer.vertex(i));
    Eigen::Affine3d se3 = vertex_se3->estimate();
    geometry_msgs::msg::Pose pose = tf2::toMsg(se3);

    /* map */
    Eigen::Affine3d previous_affine;
    tf2::fromMsg(map_array_msg.submaps[i].pose, previous_affine);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(map_array_msg.submaps[i].cloud, *cloud_ptr);

    pcl::transformPointCloud(*cloud_ptr, *transformed_cloud_ptr, se3.matrix().cast<float>());
    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*transformed_cloud_ptr, *cloud_msg_ptr);
    *map_ptr += *transformed_cloud_ptr;

    /* submap */
    vox_nav_slam_msgs::msg::SubMap submap;
    submap.header = map_array_msg.submaps[i].header;
    submap.pose = pose;
    submap.cloud = *cloud_msg_ptr;
    modified_map_array_msg.submaps.push_back(submap);

    /* path */
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = submap.header;
    pose_stamped.pose = submap.pose;
    path.poses.push_back(pose_stamped);
  }

  modified_map_array_pub_->publish(modified_map_array_msg);
  modified_path_pub_->publish(path);

  sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*map_ptr, *map_msg_ptr);
  map_msg_ptr->header.frame_id = "map";
  modified_map_pub_->publish(*map_msg_ptr);
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
