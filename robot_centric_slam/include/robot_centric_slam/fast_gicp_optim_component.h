
#ifndef VOX_NAV_SLAM___ROBOT_CENTRIC_SLAM__FAST_GICP_OPTIM_COMPONENT
#define VOX_NAV_SLAM___ROBOT_CENTRIC_SLAM__FAST_GICP_OPTIM_COMPONENT

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

#include <Eigen/Core>
#include <queue>
#include <vector>
#include <string>
#include <memory>
#include <mutex>

#include <cmath>
#include <cstdlib>
#include <limits>
#include <vector>

struct ICPParametersOptim
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

namespace vox_nav_slam
{

template <typename fp_type>
fp_type minkowski_distance(const fp_type& p, const std::vector<fp_type>& ds)
{
  // ds contains d[i] = v[i] - w[i] for two vectors v and w
  fp_type ex = 0.0;
  fp_type min_d = std::numeric_limits<fp_type>::infinity();
  fp_type max_d = -std::numeric_limits<fp_type>::infinity();
  for (int i = 0; i < ds.size(); ++i)
  {
    fp_type d = std::fabs(ds[i]);
    ex += std::pow(d, p);
    min_d = std::min(min_d, d);
    max_d = std::max(max_d, d);
  }

  return std::isnan(ex)                         ? ex :
         !std::isnormal(ex) && std::signbit(p)  ? min_d :
         !std::isnormal(ex) && !std::signbit(p) ? max_d :
                                                  std::pow(ex, 1.0 / p);
}

class FastGICPOptimComponent : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Fast G I C P Optim Component object
   *
   * @param options
   */
  GS_GBS_PUBLIC
  explicit FastGICPOptimComponent(const rclcpp::NodeOptions& options);

  /**
   * @brief Destroy the Fast G I C P Optim Component object
   *
   */

  void mapArrayCallback(const vox_nav_slam_msgs::msg::MapArray::ConstSharedPtr msg);

  pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr createRegistration(std::string method, int num_threads,
                                                                            double voxel_resolution = 0.2);

  void icpThread();

  void optimizeSubmapGraph(std::vector<Eigen::Matrix4f>& refined_transforms);

  void
  publishUncertaintyMarkers(const std::vector<std::pair<Eigen::Matrix4f, Eigen::Matrix4f>>& initial_guess_vs_refined);

private:
  geometry_msgs::msg::PoseStamped current_pose_;

  std::mutex mutex_;

  // tf buffer to get access to transfroms
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  std::shared_ptr<pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>> registration_;
  std::shared_ptr<pcl::VoxelGrid<pcl::PointXYZI>> voxelgrid_;

  vox_nav_slam_msgs::msg::MapArray::SharedPtr map_array_msg_;
  rclcpp::Subscription<vox_nav_slam_msgs::msg::MapArray>::SharedPtr map_array_sub_;
  rclcpp::Publisher<vox_nav_slam_msgs::msg::MapArray>::SharedPtr modified_map_array_pub_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr modified_path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr modified_map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr icp_uncertainty_marker_pub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr icp_pose_array_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr odom_pose_array_pub_;

  std::shared_ptr<std::thread> icp_thread_;

  // ICP parameters
  ICPParametersOptim icp_params_;

  bool initial_map_array_received_{ false };
  bool is_map_array_updated_{ false };

  Eigen::Matrix4f previous_odom_mat_{ Eigen::Matrix4f::Identity() };
};
}  // namespace vox_nav_slam

#endif  // VOX_NAV_SLAM___ROBOT_CENTRIC_SLAM__FAST_GICP_OPTIM_COMPONENT
