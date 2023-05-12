
#ifndef GS_GTSAM_COMPONENT_H_INCLUDED
#define GS_GTSAM_COMPONENT_H_INCLUDED

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

#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_gicp_st.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <fast_gicp/ndt/ndt_cuda.hpp>
#include <fast_gicp/gicp/fast_vgicp_cuda.hpp>

#include <vox_nav_slam_msgs/msg/map_array.hpp>
#include <vox_nav_slam_msgs/msg/sub_map.hpp>

#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Key.h>
#include <gtsam/geometry/Pose3.h>

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
  int num_adjacent_pose_constraints = 5;
  bool debug = false;
};

class GTSAMComponent : public rclcpp::Node
{
public:
  GTSAMComponent(const rclcpp::NodeOptions& options);
  ~GTSAMComponent();

  void mapArrayCallback(const vox_nav_slam_msgs::msg::MapArray::ConstSharedPtr msg);
  void doPoseAdjustment(vox_nav_slam_msgs::msg::MapArray map_array_msg);

  pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr createRegistration(std::string method, int num_threads,
                                                                            double voxel_resolution = 0.2);

private:
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

  // ICP parameters
  ICPParameters icp_params_;

  bool initial_map_array_received_{ false };
  bool is_map_array_updated_{ false };

  struct LoopEdge
  {
    std::pair<int, int> pair_id;
    Eigen::Isometry3d relative_pose;
  };
  std::vector<LoopEdge> loop_edges_;
};
}  // namespace vox_nav_slam

#endif  // GS_GTSAM_COMPONENT_H_INCLUDED
