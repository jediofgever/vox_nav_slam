// Source: https://github.com/rsasaki0109/lidarslam_ros2

#ifndef GS_GBS_COMPONENT_H_INCLUDED
#define GS_GBS_COMPONENT_H_INCLUDED

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define GS_GBS_EXPORT __attribute__((dllexport))
#define GS_GBS_IMPORT __attribute__((dllimport))
#else
#define GS_GBS_EXPORT __declspec(dllexport)
#define GS_GBS_IMPORT __declspec(dllimport)
#endif
#ifdef GS_GBS_BUILDING_DLL
#define GS_GBS_PUBLIC GS_GBS_EXPORT
#else
#define GS_GBS_PUBLIC GS_GBS_IMPORT
#endif
#define GS_GBS_PUBLIC_TYPE GS_GBS_PUBLIC
#define GS_GBS_LOCAL
#else
#define GS_GBS_EXPORT __attribute__((visibility("default")))
#define GS_GBS_IMPORT
#if __GNUC__ >= 4
#define GS_GBS_PUBLIC __attribute__((visibility("default")))
#define GS_GBS_LOCAL __attribute__((visibility("hidden")))
#else
#define GS_GBS_PUBLIC
#define GS_GBS_LOCAL
#endif
#define GS_GBS_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

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
#include <visualization_msgs/msg/marker_array.hpp>

#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_gicp_st.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <fast_gicp/ndt/ndt_cuda.hpp>
#include <fast_gicp/gicp/fast_vgicp_cuda.hpp>

#include <vox_nav_slam_msgs/msg/map_array.hpp>
#include <vox_nav_slam_msgs/msg/sub_map.hpp>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/slam3d/parameter_se3_offset.h>

#include <Eigen/Core>
#include <queue>
#include <vector>
#include <string>
#include <memory>
#include <mutex>

namespace graphslam
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
  int num_adjacent_pose_constraints = 5;
  bool debug = false;
};

class GraphBasedSlamComponent : public rclcpp::Node
{
public:
  GS_GBS_PUBLIC
  explicit GraphBasedSlamComponent(const rclcpp::NodeOptions& options);

  void mapArrayCallback(const vox_nav_slam_msgs::msg::MapArray::ConstSharedPtr msg);

  pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr createRegistration(std::string method, int num_threads,
                                                                            double voxel_resolution = 0.2);

  void icpThread();

  void optimizeSubmapGraph(std::vector<Eigen::Matrix4f>& refined_transforms);

private:
  std::mutex mutex_;

  // tf buffer to get access to transfroms
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  std::shared_ptr<pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>> registration_;
  std::shared_ptr<pcl::VoxelGrid<pcl::PointXYZI>> voxelgrid_;

  std::shared_ptr<std::thread> icp_thread_;

  vox_nav_slam_msgs::msg::MapArray::SharedPtr map_array_msg_;
  rclcpp::Subscription<vox_nav_slam_msgs::msg::MapArray>::SharedPtr map_array_sub_;
  rclcpp::Publisher<vox_nav_slam_msgs::msg::MapArray>::SharedPtr modified_map_array_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr modified_path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr modified_map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr icp_uncertainty_marker_pub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr icp_pose_array_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr odom_pose_array_pub_;

  // ICP parameters
  ICPParameters icp_params_;

  bool initial_map_array_received_{ false };
  bool is_map_array_updated_{ false };
};
}  // namespace graphslam

#endif  // GS_GBS_COMPONENT_H_INCLUDED
