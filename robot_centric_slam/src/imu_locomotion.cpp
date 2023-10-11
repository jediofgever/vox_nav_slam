
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

#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/create_timer_ros.h>
#include <pcl/filters/model_outlier_removal.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vision_msgs/msg/detection3_d.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

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

#include <queue>

class LocomotionAnalyzer : public rclcpp::Node
{
public:
  LocomotionAnalyzer();
  ~LocomotionAnalyzer();

  void mapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  void takeSnapshotCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                            const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                            const std::shared_ptr<std_srvs::srv::SetBool::Response> response, const int i);

  std::tuple<Eigen::Matrix3f, Eigen::Matrix3f, Eigen::Matrix3f, Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f>
  calculateIMUCovariance();

  void publishCovarianceMarkers(
      std::tuple<Eigen::Matrix3f, Eigen::Matrix3f, Eigen::Matrix3f, Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f>
          imu_covariance,
      geometry_msgs::msg::Transform pose);

  double clip(double n, double lower, double upper)
  {
    return std::max(lower, std::min(n, upper));
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr imu_covariance_pub_;

  // publish a footprint box around the robot
  rclcpp::Publisher<vision_msgs::msg::Detection3D>::SharedPtr footprint_box_pub_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr map_;

  std::vector<sensor_msgs::msg::Imu::SharedPtr> imu_buffer_;
  sensor_msgs::msg::Imu::SharedPtr latest_imu_msg_;
  visualization_msgs::msg::MarkerArray locomotion_error_markers_;

  // Lat N seconds to consider
  int look_back_poses_{ 50 };
  float dist_threshold_{ 0.01 };

  // latest transform from map to base_link
  geometry_msgs::msg::TransformStamped map_to_base_link_;

  // tf buffer to get access to transfroms
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool take_snapshot_non_traversable_{ false };
  bool take_snapshot_traversable_{ false };

  // service server for taking a snapshot of point cloud
  std::vector<rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr> take_snapshot_srv_;

  std::mutex mutex_;
};

LocomotionAnalyzer::LocomotionAnalyzer() : Node("locomotion_analyzer")
{
  RCLCPP_INFO(this->get_logger(), "locomotion_analyzer node has been initialized.");

  map_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

  // use a reliable qos profile to make sure that the message is delivered
  rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

  // use sensor qos profile to make sure that the message is delivered
  rclcpp::QoS sensor_qos_profile = rclcpp::SensorDataQoS();

  // map subscriber
  map_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/agv_map", sensor_qos_profile, std::bind(&LocomotionAnalyzer::mapCallback, this, std::placeholders::_1));

  // odom subscriber
  imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/agv_imu", sensor_qos_profile, std::bind(&LocomotionAnalyzer::imuCallback, this, std::placeholders::_1));

  marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(  // NOLINT
      "cova_marker_array", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

  // footprint box publisher
  footprint_box_pub_ = this->create_publisher<vision_msgs::msg::Detection3D>(  // NOLINT
      "footprint_box", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

  // imu covariance publisher
  imu_covariance_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(  // NOLINT
      "imu_info", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

  // from 0 to 5, there is 6 services
  for (size_t i = 0; i < 6; i++)
  {
    take_snapshot_srv_.push_back(this->create_service<std_srvs::srv::SetBool>(
        "take_snapshot_" + std::to_string(i),
        std::bind(&LocomotionAnalyzer::takeSnapshotCallback, this, std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3, i)));
  }

  // TF listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  latest_imu_msg_ = sensor_msgs::msg::Imu::SharedPtr(new sensor_msgs::msg::Imu());

  declare_parameter("look_back_poses", 50);
  get_parameter("look_back_poses", look_back_poses_);

  declare_parameter("dist_threshold", 0.01);
  get_parameter("dist_threshold", dist_threshold_);
}

LocomotionAnalyzer::~LocomotionAnalyzer()
{
}

std::tuple<Eigen::Matrix3f, Eigen::Matrix3f, Eigen::Matrix3f, Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f>
LocomotionAnalyzer::calculateIMUCovariance()
{
  Eigen::MatrixXf rpy_matrix(3, imu_buffer_.size());
  Eigen::MatrixXf acc_matrix(3, imu_buffer_.size());
  Eigen::MatrixXf gyro_matrix(3, imu_buffer_.size());

  for (std::size_t j = 0; j < imu_buffer_.size(); ++j)
  {
    // Calculate roll, pitch, yaw from imu
    Eigen::Quaternionf q(imu_buffer_[j]->orientation.w, imu_buffer_[j]->orientation.x, imu_buffer_[j]->orientation.y,
                         imu_buffer_[j]->orientation.z);
    Eigen::Vector3f rpy = q.toRotationMatrix().eulerAngles(0, 1, 2);
    rpy_matrix(0, j) = rpy(0);
    rpy_matrix(1, j) = rpy(1);
    rpy_matrix(2, j) = rpy(2);

    // Calculate acceleration from imu
    Eigen::Vector3f acc(imu_buffer_[j]->linear_acceleration.x, imu_buffer_[j]->linear_acceleration.y,
                        imu_buffer_[j]->linear_acceleration.z);
    acc_matrix(0, j) = acc(0);
    acc_matrix(1, j) = acc(1);
    acc_matrix(2, j) = acc(2);

    // Calculate angular velocity from imu
    Eigen::Vector3f gyro(imu_buffer_[j]->angular_velocity.x, imu_buffer_[j]->angular_velocity.y,
                         imu_buffer_[j]->angular_velocity.z);
    gyro_matrix(0, j) = gyro(0);
    gyro_matrix(1, j) = gyro(1);
    gyro_matrix(2, j) = gyro(2);
  }

  Eigen::Vector3f rpy_mean = rpy_matrix.rowwise().mean();
  Eigen::Vector3f acc_mean = acc_matrix.rowwise().mean();
  Eigen::Vector3f gyro_mean = gyro_matrix.rowwise().mean();

  // Transpose in place
  rpy_matrix.transposeInPlace();
  acc_matrix.transposeInPlace();
  gyro_matrix.transposeInPlace();

  Eigen::MatrixXf rpy_centered = rpy_matrix.rowwise() - rpy_matrix.colwise().mean();
  Eigen::MatrixXf acc_centered = acc_matrix.rowwise() - acc_matrix.colwise().mean();
  Eigen::MatrixXf gyro_centered = gyro_matrix.rowwise() - gyro_matrix.colwise().mean();

  Eigen::Matrix3f rpy_covariance = (rpy_centered.adjoint() * rpy_centered) / float(rpy_matrix.rows() - 1);
  Eigen::Matrix3f acc_covariance = (acc_centered.adjoint() * acc_centered) / float(acc_matrix.rows() - 1);
  Eigen::Matrix3f gyro_covariance = (gyro_centered.adjoint() * gyro_centered) / float(gyro_matrix.rows() - 1);

  // Normilize the meanrpys from -pi to pi range to 0 to 1 range
  rpy_mean(0) = (rpy_mean(0) + M_PI) / (2 * M_PI);
  rpy_mean(1) = (rpy_mean(1) + M_PI) / (2 * M_PI);
  rpy_mean(2) = (rpy_mean(2) + M_PI) / (2 * M_PI);

  // Return a tuple covarinces and means
  return std::make_tuple(rpy_covariance, acc_covariance, gyro_covariance, rpy_mean, acc_mean, gyro_mean);
}

void LocomotionAnalyzer::publishCovarianceMarkers(
    std::tuple<Eigen::Matrix3f, Eigen::Matrix3f, Eigen::Matrix3f,  // NOLINT
               Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f>
        imu_covariance,
    geometry_msgs::msg::Transform pose)
{
  // get the rpy_covariance, acc_covariance, gyro_covariance, rpy_mean, acc_mean, gyro_mean
  Eigen::Matrix3f rpy_covariance = std::get<0>(imu_covariance);
  Eigen::Matrix3f acc_covariance = std::get<1>(imu_covariance);
  Eigen::Matrix3f gyro_covariance = std::get<2>(imu_covariance);
  Eigen::Vector3f rpy_mean = std::get<3>(imu_covariance);
  Eigen::Vector3f acc_mean = std::get<4>(imu_covariance);
  Eigen::Vector3f gyro_mean = std::get<5>(imu_covariance);

  // Publish acc covariace with green ellipses use all 3 rows and columns of the acc_covariance matrix
  // RPY COVARINCE MARKER
  visualization_msgs::msg::Marker gyro_covariance_marker;
  gyro_covariance_marker.header.frame_id = "odom";
  gyro_covariance_marker.header.stamp = this->now();
  gyro_covariance_marker.ns = "gyro_covariance";
  gyro_covariance_marker.id = locomotion_error_markers_.markers.size();
  gyro_covariance_marker.type = visualization_msgs::msg::Marker::SPHERE;
  gyro_covariance_marker.action = visualization_msgs::msg::Marker::ADD;
  gyro_covariance_marker.lifetime = rclcpp::Duration(0, 0);
  gyro_covariance_marker.pose.position.x = pose.translation.x;
  gyro_covariance_marker.pose.position.y = pose.translation.y;
  gyro_covariance_marker.pose.position.z = pose.translation.z;
  float min_val = 0.01;
  float max_val = 1.0;
  gyro_covariance_marker.scale.x = clip(gyro_covariance(0, 0), min_val, max_val);
  gyro_covariance_marker.scale.y = clip(gyro_covariance(1, 1), min_val, max_val);
  // gyro_covariance_marker.scale.z = clip(gyro_covariance(2, 2), min_val, max_val);
  // yaw doesnt matter
  gyro_covariance_marker.scale.z = min_val;
  gyro_covariance_marker.color.a = 0.9;
  gyro_covariance_marker.color.r = 1.0;
  gyro_covariance_marker.color.g = 0.0;
  gyro_covariance_marker.color.b = 0.0;
  locomotion_error_markers_.markers.push_back(gyro_covariance_marker);

  // ACC COVARINCE MARKER
  visualization_msgs::msg::Marker acc_covariance_marker;
  acc_covariance_marker.header.frame_id = "odom";
  acc_covariance_marker.header.stamp = this->now();
  acc_covariance_marker.ns = "acc_covariance";
  acc_covariance_marker.id = locomotion_error_markers_.markers.size();
  acc_covariance_marker.type = visualization_msgs::msg::Marker::SPHERE;
  acc_covariance_marker.action = visualization_msgs::msg::Marker::ADD;
  acc_covariance_marker.lifetime = rclcpp::Duration(0, 0);
  acc_covariance_marker.pose.position.x = pose.translation.x;
  acc_covariance_marker.pose.position.y = pose.translation.y;
  acc_covariance_marker.pose.position.z = pose.translation.z;

  // clip the covariance values to min and max values
  acc_covariance_marker.scale.x = clip(acc_covariance(0, 0), min_val, max_val);
  acc_covariance_marker.scale.y = clip(acc_covariance(1, 1), min_val, max_val);
  acc_covariance_marker.scale.z = clip(acc_covariance(2, 2), min_val, max_val);

  acc_covariance_marker.color.a = 0.9;
  acc_covariance_marker.color.r = 0.0;
  acc_covariance_marker.color.g = 1.0;
  acc_covariance_marker.color.b = 0.0;
  locomotion_error_markers_.markers.push_back(acc_covariance_marker);

  // also publish the footprint box as a detection3d message
  vision_msgs::msg::Detection3D footprint_box;
  footprint_box.header.frame_id = "base_link";
  footprint_box.header.stamp = this->now();
  footprint_box.results = { vision_msgs::msg::ObjectHypothesisWithPose() };
  footprint_box.bbox.center.position.x = 0.0;
  footprint_box.bbox.center.position.y = 0.0;
  footprint_box.bbox.center.position.z = 0.0;
  footprint_box.bbox.size.x = 2.0;
  footprint_box.bbox.size.y = 1.0;
  footprint_box.bbox.size.z = 5.0;
  footprint_box.id = "footprint_box";
  footprint_box_pub_->publish(footprint_box);

  // Publish the loco error markers
  marker_array_pub_->publish(locomotion_error_markers_);
}

void LocomotionAnalyzer::takeSnapshotCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                              const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                              const std::shared_ptr<std_srvs::srv::SetBool::Response> response,
                                              const int i)
{
  // Lets print i to see which service is called
  RCLCPP_INFO(this->get_logger(), "take_snapshot_%i service has been called.", i);

  double locomotion_error = 1.0 / 5.0 * i;

  // print the locomotion error
  RCLCPP_INFO(this->get_logger(), "locomotion_error: %f", locomotion_error);

  // crop the map to the ROI with crop box filter around current pose
  pcl::CropBox<pcl::PointXYZI> crop_box_filter;

  // Transform the map to base_link frame
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  // get map -> base_link transform with tf_buffer
  geometry_msgs::msg::TransformStamped transformStamped;
  try
  {
    transformStamped = tf_buffer_->lookupTransform("base_link", "odom", rclcpp::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    return;
  }

  // transform the map to base_link frame
  // if map is empty, then return
  if (map_->points.size() == 0)
  {
    RCLCPP_WARN(get_logger(), "Map is empty, not saving the map");
    return;
  }

  pcl_ros::transformPointCloud(*map_, *transformed_cloud_ptr, transformStamped);
  Eigen::Vector4f min_point(-1.0, -0.5, -2.5, 1.0);
  Eigen::Vector4f max_point(+1.0, +0.5, +2.5, 1.0);
  crop_box_filter.setInputCloud(transformed_cloud_ptr);
  crop_box_filter.setMin(min_point);
  crop_box_filter.setMax(max_point);
  crop_box_filter.setNegative(false);
  crop_box_filter.filter(*transformed_cloud_ptr);

  if (transformed_cloud_ptr->points.size() < 10)
  {
    RCLCPP_WARN(get_logger(), "Map size is too small, not saving the map");
    return;
  }

  // Save the map to a file
  time_t curr_time;
  tm* curr_tm;
  char time_string[100];
  time(&curr_time);
  curr_tm = localtime(&curr_time);
  strftime(time_string, 50, "%T", curr_tm);

  // REMOVE SPECIAL CHARACTERS FROM TIME STRING
  std::string time_string_str(time_string);
  std::replace(time_string_str.begin(), time_string_str.end(), ':', '_');
  std::replace(time_string_str.begin(), time_string_str.end(), ' ', '_');

  // round the locomotion error to 2 decimal places
  std::stringstream ss;
  ss << std::fixed << std::setprecision(2) << locomotion_error;
  std::string locomotion_error_str = ss.str();
  locomotion_error_str.erase(remove(locomotion_error_str.begin(), locomotion_error_str.end(), '.'),
                             locomotion_error_str.end());  // remove . from string

  std::string map_file_name = "/home/atas/RESEARCH/traversablity_estimation_net/data_imu/" + time_string_str + "_" +
                              locomotion_error_str + ".pcd";
  pcl::io::savePCDFileASCII(map_file_name, *transformed_cloud_ptr);
  RCLCPP_INFO(get_logger(), "Map saved to %s", map_file_name.c_str());

  // calculate imu covariance
  // The result is a tuple of 3 matrices and 3 vectors in the following order
  // (rpy_covariance, acc_covariance, gyro_covariance, rpy_mean, acc_mean, gyro_mean)
  auto imu_covariance = calculateIMUCovariance();

  // Flatten the matrices to vectors and concatenate them
  Eigen::Map<Eigen::VectorXf> v1(std::get<0>(imu_covariance).data(),
                                 std::get<0>(imu_covariance).size());  // size = 9 // RPY
  Eigen::Map<Eigen::VectorXf> v2(std::get<1>(imu_covariance).data(),
                                 std::get<1>(imu_covariance).size());  // size = 9 // ACC
  Eigen::Map<Eigen::VectorXf> v3(std::get<2>(imu_covariance).data(),
                                 std::get<2>(imu_covariance).size());  // size = 9 // GYRO

  // Concatenate the vectors to get a single vector of size 36
  Eigen::VectorXf imu_covariance_vector(36);
  imu_covariance_vector << v1, v2, v3, std::get<3>(imu_covariance), std::get<4>(imu_covariance),
      std::get<5>(imu_covariance);

  // set the precision to 5 decimal places and remove the trailing zeros from the imu_covariance_vector
  std::stringstream ss2;
  ss2 << std::fixed << std::setprecision(5) << imu_covariance_vector;
  std::string imu_covariance_vector_str = ss2.str();

  // save the imu covariance vector to a file
  // Print the vector
  // log the format of the vector
  // rpy_covariance, acc_covariance, gyro_covariance, rpy_mean, acc_mean, gyro_mean
  // std::cout << "The vector includes the following elements: " << std::endl;
  // std::cout << "rpy_covariance: "
  //           << "acc_covariance: "
  //           << "gyro_covariance: "
  //           << "rpy_mean: "
  //           << "acc_mean: "
  //           << "gyro_mean: " << std::endl;
  // std::cout << "imu_covariance_vector: " << imu_covariance_vector_str << std::endl;

  // Lets create another vector with acc_covariance, and xyzw of the orientation quaternion
  // with 9 + 4 = 13 elements
  Eigen::VectorXf imu_covariance_vector_13(13);
  Eigen::Vector4f orientation_quaternion(latest_imu_msg_->orientation.x, latest_imu_msg_->orientation.y,
                                         latest_imu_msg_->orientation.z, latest_imu_msg_->orientation.w);
  imu_covariance_vector_13 << v2, orientation_quaternion;

  // print the imu_covariance_vector_13
  std::cout << "imu_covariance_vector_13: " << imu_covariance_vector_13 << std::endl;

  // make a string from imu_covariance_vector_13
  std::stringstream ss3;
  ss3 << std::fixed << std::setprecision(5) << imu_covariance_vector_13;
  std::string imu_covariance_vector_13_str = ss3.str();

  // write this with same name but with .txt extension
  std::string imu_covariance_vector_file_name = map_file_name.substr(0, map_file_name.size() - 4) + ".txt";
  std::ofstream myfile;
  myfile.open(imu_covariance_vector_file_name);
  myfile << imu_covariance_vector_13_str;
  myfile.close();

  // Publish the covariance markers to map -> base_link frame
  publishCovarianceMarkers(imu_covariance, transformStamped.transform);
}

void LocomotionAnalyzer::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_imu_msg_ = std::make_shared<sensor_msgs::msg::Imu>(*msg);

  // get transform from odom to base_link
  geometry_msgs::msg::TransformStamped transform_stamped;
  try
  {
    transform_stamped = tf_buffer_->lookupTransform("base_link", "odom", tf2::TimePointZero);
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
    return;
  }

  // if difference between current and previous transform is less than 0.1 meter, then return
  double dist =
      std::sqrt(std::pow(map_to_base_link_.transform.translation.x - transform_stamped.transform.translation.x, 2) +
                std::pow(map_to_base_link_.transform.translation.y - transform_stamped.transform.translation.y, 2) +
                std::pow(map_to_base_link_.transform.translation.z - transform_stamped.transform.translation.z, 2));
  if (dist < dist_threshold_)
  {
    return;
  }

  // update the latest transform
  map_to_base_link_ = transform_stamped;

  // add the latest imu message to the buffer
  imu_buffer_.push_back(msg);

  // if buffer size is greater than look_back_poses_, then pop the oldest element
  if (imu_buffer_.size() > look_back_poses_)
  {
    // remove the oldest element
    imu_buffer_.erase(imu_buffer_.begin());
  }

  // Get current pose in odom frame from tf_buffer
  geometry_msgs::msg::TransformStamped transform_stamped_map;
  try
  {
    transform_stamped_map = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
    return;
  }

  // get IMU covariance and publish it
  auto imu_covariance = calculateIMUCovariance();
  publishCovarianceMarkers(imu_covariance, transform_stamped_map.transform);

  // In case we are inferrring the we need to also publish the floatarray message for IMU
  // covariance and orientation quaternion

  Eigen::VectorXf imu_covariance_vector_13(13);
  Eigen::Map<Eigen::VectorXf> v2(std::get<1>(imu_covariance).data(),
                                 std::get<1>(imu_covariance).size());  // size = 9 // ACC
  Eigen::Vector4f orientation_quaternion(latest_imu_msg_->orientation.x, latest_imu_msg_->orientation.y,
                                         latest_imu_msg_->orientation.z, latest_imu_msg_->orientation.w);
  imu_covariance_vector_13 << v2, orientation_quaternion;

  // crete float vector from imu_covariance_vector_13
  std::vector<float> imu_covariance_vector_13_float_vector(
      imu_covariance_vector_13.data(), imu_covariance_vector_13.data() + imu_covariance_vector_13.size());

  // Now publish the imu_covariance_vector_13 as a float32multiarray
  std_msgs::msg::Float32MultiArray imu_covariance_vector_13_msg;
  imu_covariance_vector_13_msg.data = imu_covariance_vector_13_float_vector;

  imu_covariance_pub_->publish(imu_covariance_vector_13_msg);
}

void LocomotionAnalyzer::mapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // RCLCPP_INFO(this->get_logger(), "Received map message with timestamp %i.", msg->header.stamp.sec);
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
  map_ = cloud;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LocomotionAnalyzer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}