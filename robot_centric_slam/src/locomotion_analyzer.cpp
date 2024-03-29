
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
  void takeSnapshotNonTravCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                   const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                   const std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void takeSnapshotCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                            const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                            const std::shared_ptr<std_srvs::srv::SetBool::Response> response);

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr curr_pose_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;

  // service server for taking a snapshot of point cloud
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr take_snapshot_non_traversable_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr take_snapshot_traversable_srv_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr targeted_cloud_;
  geometry_msgs::msg::PoseStamped curr_pose_;
  geometry_msgs::msg::PoseStamped::SharedPtr last_map_save_pose_;

  std::queue<nav_msgs::msg::Odometry::SharedPtr> odom_buffer_;
  float current_locomotion_error_{ 0.0 };
  visualization_msgs::msg::MarkerArray locomotion_error_markers_;
  int spehere_marker_id_{ 0 };
  std::queue<nav_msgs::msg::Odometry::SharedPtr> cmd_vel_buffer_;

  // Lat N seconds to consider
  float time_frame_{ 4.0 };
  // Mean nominal velocity in the last  N seconds
  float w1_{ 2.0 };
  float w2_{ 2.0 };

  // tf buffer to get access to transfroms
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  sensor_msgs::msg::Imu::SharedPtr imu_msg_;
  bool imu_received_{ false };

  bool take_snapshot_non_traversable_{ false };
  bool take_snapshot_traversable_{ false };
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

  // cmd_vel subscriber
  cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/agv_cmd_vel", sensor_qos_profile,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {  // Keep only messages in the last N seconds
        auto fake_odom = std::make_shared<nav_msgs::msg::Odometry>();
        fake_odom->header.stamp = this->now();
        fake_odom->twist.twist.linear.x = msg->linear.x;
        fake_odom->twist.twist.linear.y = msg->linear.y;
        fake_odom->twist.twist.angular.z = msg->angular.z;
        cmd_vel_buffer_.push(fake_odom);

        // pop the odom from the queue if the message is older than 4 seconds
        while (cmd_vel_buffer_.size() > 0)
        {
          auto oldest_odom = cmd_vel_buffer_.front();
          auto newest_odom = cmd_vel_buffer_.back();
          if (newest_odom->header.stamp.sec - oldest_odom->header.stamp.sec > time_frame_)
          {
            cmd_vel_buffer_.pop();
          }
          else
          {
            // break;
            break;
          }
        }

        auto tmp_q = cmd_vel_buffer_;  // copy the original queue to the temporary queue
        // calculate the mean velocity in the last N seconds
        float mean_vel_x = 0.0;
        while (!tmp_q.empty())
        {
          mean_vel_x += tmp_q.front()->twist.twist.linear.x;
          tmp_q.pop();
        }
        mean_vel_x /= cmd_vel_buffer_.size();
        w1_ = std::abs(mean_vel_x);
        w2_ = std::abs(mean_vel_x);
      });

  // imu subscriber
  imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>("/agv_imu", sensor_qos_profile,
                                                                     [this](sensor_msgs::msg::Imu::SharedPtr msg) {
                                                                       if (!imu_received_)
                                                                       {
                                                                         imu_msg_ = msg;
                                                                         imu_received_ = true;
                                                                       }
                                                                     });

  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(  // NOLINT
      "locomotion_error_marker", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(  // NOLINT
      "locomotion_error_marker_array", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

  take_snapshot_non_traversable_srv_ = this->create_service<std_srvs::srv::SetBool>(
      "take_snapshot_non_traversable", std::bind(&LocomotionAnalyzer::takeSnapshotNonTravCallback, this,
                                                 std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  take_snapshot_traversable_srv_ = this->create_service<std_srvs::srv::SetBool>(
      "take_snapshot_traversable", std::bind(&LocomotionAnalyzer::takeSnapshotCallback, this, std::placeholders::_1,
                                             std::placeholders::_2, std::placeholders::_3));

  // TF listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

LocomotionAnalyzer::~LocomotionAnalyzer()
{
}

void LocomotionAnalyzer::takeSnapshotCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                              const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                              const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  (void)request_header;
  (void)request;
  (void)response;
  take_snapshot_traversable_ = true;
}

void LocomotionAnalyzer::takeSnapshotNonTravCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                                     const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                                     const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  (void)request_header;
  (void)request;
  (void)response;

  take_snapshot_non_traversable_ = true;
}

void LocomotionAnalyzer::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // RCLCPP_INFO(this->get_logger(), "Received odometry message with timestamp %i.", msg->header.stamp.sec);

  auto wheel_diameter = 0.185;

  // push this odom to the queue
  odom_buffer_.push(std::make_shared<nav_msgs::msg::Odometry>(*msg));

  // pop the odom from the queue if the message is older than 4 seconds
  while (odom_buffer_.size() > 0)
  {
    auto oldest_odom = odom_buffer_.front();
    auto newest_odom = odom_buffer_.back();
    if (newest_odom->header.stamp.sec - oldest_odom->header.stamp.sec > time_frame_)
    {
      odom_buffer_.pop();
    }
    else
    {
      break;
    }
  }

  // Calculate Locomotion error based on the odom buffer
  // use ((r * w1) + (r * w2)) / 2 to calculate the distance traveled
  // r = wheel diameter, w1 = angular velocity of left wheel, w2 = angular velocity of right wheel
  auto distance = ((wheel_diameter * w1_) + (wheel_diameter * w2_)) / 2;
  auto nominal_distance = distance * time_frame_;  // multiply by time_frame to get the distance traveled in 4 seconds
  nominal_distance = std::abs(nominal_distance);

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
  marker.lifetime = rclcpp::Duration(rclcpp::Duration::from_seconds(10 * time_frame_));
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
  ss << "W1_: " << w1_ << " W2_: " << w2_;
  marker.text = ss.str();
  current_locomotion_error_ = locomotion_error;

  visualization_msgs::msg::Marker sphere_marker;
  sphere_marker.header.frame_id = "map";
  sphere_marker.header.stamp = now();
  sphere_marker.ns = "locomotion_error_sphere";
  sphere_marker.id = spehere_marker_id_;
  sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
  sphere_marker.action = visualization_msgs::msg::Marker::ADD;
  sphere_marker.lifetime = rclcpp::Duration(rclcpp::Duration::from_seconds(10 * time_frame_));
  sphere_marker.pose.position.x = curr_pose_.pose.position.x;
  sphere_marker.pose.position.y = curr_pose_.pose.position.y;
  sphere_marker.pose.position.z = curr_pose_.pose.position.z;
  sphere_marker.scale.z = 0.5;
  sphere_marker.scale.x = 0.5;
  sphere_marker.scale.y = 0.5;

  // crop the map around the base_link and save it to disk together with locomotion error
  // Transform the map to base_link frame
  if (targeted_cloud_->points.size() > 0)
  {
    auto x_dist = last_map_save_pose_->pose.position.x - curr_pose_.pose.position.x;
    auto y_dist = last_map_save_pose_->pose.position.y - curr_pose_.pose.position.y;
    auto dist = std::sqrt(std::pow(x_dist, 2) + std::pow(y_dist, 2));

    if (take_snapshot_non_traversable_ || take_snapshot_traversable_)
    {
      if (take_snapshot_non_traversable_)
      {
        // Manual snapshots are taken when the robot is stuck, the locomotion error is set to 1.0
        RCLCPP_INFO(get_logger(), "Taking a snapshot non traversable area");
        current_locomotion_error_ = 1.0;
        locomotion_error = 1.0;
      }
      else
      {
        RCLCPP_INFO(get_logger(), "Taking a snapshot of traversable area");
        current_locomotion_error_ = 0.0;
        locomotion_error = 0.0;
      }
      take_snapshot_non_traversable_ = false;
      take_snapshot_traversable_ = false;

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
      // crop the map to the ROI with crop box filter around current pose
      pcl::CropBox<pcl::PointXYZI> crop_box_filter;

      // Transform the map to base_link frame
      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

      // get map -> base_link transform with tf_buffer
      geometry_msgs::msg::TransformStamped transformStamped;
      try
      {
        transformStamped = tf_buffer_->lookupTransform("base_link", "map", rclcpp::Time(0));
      }
      catch (tf2::TransformException& ex)
      {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        return;
      }

      // transform the map to base_link frame
      pcl_ros::transformPointCloud(*targeted_cloud_, *transformed_cloud_ptr, transformStamped);

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

      std::string map_file_name = "/home/atas/unity_data/data/" + time_string_str + "_" + locomotion_error_str + ".pcd";
      pcl::io::savePCDFileASCII(map_file_name, *transformed_cloud_ptr);
      RCLCPP_INFO(get_logger(), "Map saved to %s", map_file_name.c_str());

      last_map_save_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>(curr_pose_);

      locomotion_error_markers_.markers.push_back(sphere_marker);
      locomotion_error_markers_.markers.push_back(marker);
      spehere_marker_id_++;
      marker_array_pub_->publish(locomotion_error_markers_);
      marker_pub_->publish(marker);

      if (locomotion_error_markers_.markers.size() > 10)

      {
        // clear the markers
        locomotion_error_markers_.markers.clear();
      }
    }
  }
}

void LocomotionAnalyzer::currPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // RCLCPP_INFO(this->get_logger(), "Received current pose message with timestamp %i.", msg->header.stamp.sec);
  curr_pose_ = *msg;
}

void LocomotionAnalyzer::mapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // RCLCPP_INFO(this->get_logger(), "Received map message with timestamp %i.", msg->header.stamp.sec);
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