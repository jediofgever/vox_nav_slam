
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
#include <std_msgs/msg/bool.hpp>
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

  void currPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void mapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr curr_pose_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr targeted_cloud_;
  geometry_msgs::msg::PoseStamped curr_pose_;
  geometry_msgs::msg::PoseStamped::SharedPtr last_map_save_pose_;

  std::queue<nav_msgs::msg::Odometry::SharedPtr> odom_buffer_;
  float current_locomotion_error_{ 0.0 };
  visualization_msgs::msg::MarkerArray locomotion_error_markers_;
  int spehere_marker_id_{ 0 };
};

LocomotionAnalyzer::LocomotionAnalyzer() : Node("locomotion_analyzer")
{
  RCLCPP_INFO(this->get_logger(), "locomotion_analyzer node has been initialized.");

  last_map_save_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
  targeted_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

  // use a reliable qos profile to make sure that the message is delivered
  rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

  // use sensor qos profile to make sure that the message is delivered
  rclcpp::QoS sensor_qos_profile = rclcpp::SensorDataQoS();

  // current pose subscriber
  curr_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/agv_curr_pose", sensor_qos_profile,
      std::bind(&LocomotionAnalyzer::currPoseCallback, this, std::placeholders::_1));

  // map subscriber
  map_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/agv_map", sensor_qos_profile, std::bind(&LocomotionAnalyzer::mapCallback, this, std::placeholders::_1));

  // odom subscriber
  odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/agv_odom", sensor_qos_profile, std::bind(&LocomotionAnalyzer::odomCallback, this, std::placeholders::_1));

  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(  // NOLINT
      "locomotion_error_marker", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(  // NOLINT
      "locomotion_error_marker_array", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
}

LocomotionAnalyzer::~LocomotionAnalyzer()
{
}

void LocomotionAnalyzer::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received odometry message with timestamp %i.", msg->header.stamp.sec);
  auto w1 = 2.0;
  auto w2 = 2.0;
  auto r = 0.135;
  auto time_frame = 3.0;

  // push this odom to the queue
  odom_buffer_.push(std::make_shared<nav_msgs::msg::Odometry>(*msg));

  // pop the odom from the queue if the message is older than 4 seconds
  while (odom_buffer_.size() > 0)
  {
    auto oldest_odom = odom_buffer_.front();
    auto newest_odom = odom_buffer_.back();
    if (newest_odom->header.stamp.sec - oldest_odom->header.stamp.sec > time_frame)
    {
      odom_buffer_.pop();
    }
    else
    {
      // break;
      RCLCPP_WARN(get_logger(), "Odom buffer not include 4 seconds period yet");
      break;
    }
  }

  // Calculate Locomotion error based on the odom buffer
  // use ((r * w1) + (r * w2)) / 2 to calculate the distance traveled
  // r = wheel radius, w1 = angular velocity of left wheel, w2 = angular velocity of right wheel
  auto distance = ((r * w1) + (r * w2)) / 2;
  auto nominal_distance = distance * time_frame;  // multiply by time_frame to get the distance traveled in 4 seconds

  // calculate actual distance traveled
  auto odom1 = odom_buffer_.front();
  auto odom2 = odom_buffer_.back();
  auto x1 = odom1->pose.pose.position.x;
  auto y1 = odom1->pose.pose.position.y;
  auto x2 = odom2->pose.pose.position.x;
  auto y2 = odom2->pose.pose.position.y;
  auto actual_distance = std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));

  // calculate locomotion error
  auto locomotion_error = nominal_distance - actual_distance;

  // Publish this error as RVIZ marker
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = now();
  marker.ns = "locomotion_error";
  marker.id = spehere_marker_id_;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.lifetime = rclcpp::Duration(rclcpp::Duration::from_seconds(time_frame));
  marker.pose.position = curr_pose_.pose.position;
  marker.pose.position.z += 1.0;
  marker.scale.z = 0.5;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  // Yellow color
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  std::stringstream ss;
  ss << "Locomotion Error: " << locomotion_error;
  marker.text = ss.str();
  marker_pub_->publish(marker);
  current_locomotion_error_ = locomotion_error;

  visualization_msgs::msg::Marker sphere_marker;
  sphere_marker.header.frame_id = "map";
  sphere_marker.header.stamp = now();
  sphere_marker.ns = "locomotion_error_sphere";
  sphere_marker.id = spehere_marker_id_;
  sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
  sphere_marker.action = visualization_msgs::msg::Marker::ADD;
  sphere_marker.lifetime = rclcpp::Duration(rclcpp::Duration::from_seconds(time_frame));
  sphere_marker.pose.position.x = curr_pose_.pose.position.x;
  sphere_marker.pose.position.y = curr_pose_.pose.position.y;
  sphere_marker.pose.position.z = curr_pose_.pose.position.z;
  sphere_marker.scale.z = 0.5;
  sphere_marker.scale.x = 0.5;
  sphere_marker.scale.y = 0.5;
  // color based on locomotion error
  if (locomotion_error < 0.1)
  {
    // Green color
    sphere_marker.color.a = 1.0;
    sphere_marker.color.r = 0.0;
    sphere_marker.color.g = 1.0;
    sphere_marker.color.b = 0.0;
  }
  else if (locomotion_error < 0.2)
  {
    // Yellow color
    sphere_marker.color.a = 1.0;
    sphere_marker.color.r = 1.0;
    sphere_marker.color.g = 1.0;
    sphere_marker.color.b = 0.0;
  }
  else
  {
    // Red color
    sphere_marker.color.a = 1.0;
    sphere_marker.color.r = 1.0;
    sphere_marker.color.g = 0.0;
    sphere_marker.color.b = 0.0;
  }
  locomotion_error_markers_.markers.push_back(sphere_marker);
  locomotion_error_markers_.markers.push_back(marker);
  marker_array_pub_->publish(locomotion_error_markers_);
  spehere_marker_id_++;

  // crop the map around the base_link and save it to disk together with locomotion error
  // Transform the map to base_link frame
  if (targeted_cloud_->points.size() > 0)
  {
    auto x_dist = last_map_save_pose_->pose.position.x - curr_pose_.pose.position.x;
    auto y_dist = last_map_save_pose_->pose.position.y - curr_pose_.pose.position.y;
    auto dist = std::sqrt(std::pow(x_dist, 2) + std::pow(y_dist, 2));

    if (dist > 0.5)
    {
      // convert current pose to Eigen
      Eigen::Affine3d current_map_origin = Eigen::Affine3d::Identity();
      tf2::fromMsg(curr_pose_.pose, current_map_origin);
      Eigen::Matrix4f current_map_origin_mat = current_map_origin.matrix().cast<float>();

      // center the map to current pose
      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::transformPointCloud(*targeted_cloud_, *transformed_cloud_ptr, current_map_origin_mat.inverse());

      // crop the map to the ROI with crop box filter around current pose
      pcl::CropBox<pcl::PointXYZI> crop_box_filter;
      crop_box_filter.setInputCloud(transformed_cloud_ptr);
      Eigen::Vector4f min_point(0.0, -0.5, -2.5, 1.0);
      Eigen::Vector4f max_point(2.5, 0.5, 2.5, 1.0);
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

      std::string map_file_name = "/home/atas/unity_data/data/" + time_string_str + "_" + locomotion_error_str + ".pcd";
      pcl::io::savePCDFileASCII(map_file_name, *transformed_cloud_ptr);
      RCLCPP_INFO(get_logger(), "Map saved to %s", map_file_name.c_str());

      last_map_save_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>(curr_pose_);
    }
  }
}

void LocomotionAnalyzer::currPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received current pose message with timestamp %i.", msg->header.stamp.sec);
  curr_pose_ = *msg;
}

void LocomotionAnalyzer::mapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received map message with timestamp %i.", msg->header.stamp.sec);
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
  targeted_cloud_ = cloud;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LocomotionAnalyzer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}