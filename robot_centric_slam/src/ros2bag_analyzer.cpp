

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
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
class Ros2BagAnalyzer : public rclcpp::Node
{
public:
  Ros2BagAnalyzer();
  ~Ros2BagAnalyzer();

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:
  // gnss pose subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

  // cloud subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;

  // publish a bool message to indicate the end of the episode
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr end_of_episode_publisher_;

  // latest gnss pose
  nav_msgs::msg::Odometry latest_odom_;

  bool is_first_pose = true;

  // publish the gnss pose and the cloud with the same timestamp
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
};

Ros2BagAnalyzer::Ros2BagAnalyzer() : Node("ros2bag_analyzer")
{
  RCLCPP_INFO(this->get_logger(), "ros2bag_analyzer node has been initialized.");

  // use a reliable qos profile to make sure that the message is delivered
  rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

  // use sensor qos profile to make sure that the message is delivered
  rclcpp::QoS sensor_qos_profile = rclcpp::SensorDataQoS();

  // gnss pose subscriber
  odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/agv_odom", sensor_qos_profile, std::bind(&Ros2BagAnalyzer::odomCallback, this, std::placeholders::_1));

  // cloud subscriber
  cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/agv_cloud", sensor_qos_profile, std::bind(&Ros2BagAnalyzer::cloudCallback, this, std::placeholders::_1));

  // publish a bool message to indicate the end of the episode
  end_of_episode_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/end_of_episode", qos_profile);

  // publish the gnss pose and the cloud with the same timestamp
  odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/episode_odom", sensor_qos_profile);
  cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/episode_cloud", sensor_qos_profile);
}

Ros2BagAnalyzer::~Ros2BagAnalyzer()
{
}

void Ros2BagAnalyzer::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (is_first_pose)
  {
    latest_odom_ = *msg;
    is_first_pose = false;
    return;
  }

  // check the distance between the latest gnss pose and the current gnss pose
  double distance = std::sqrt(std::pow(msg->pose.pose.position.x - latest_odom_.pose.pose.position.x, 2) +
                              std::pow(msg->pose.pose.position.y - latest_odom_.pose.pose.position.y, 2) +
                              std::pow(msg->pose.pose.position.z - latest_odom_.pose.pose.position.z, 2));

  if (distance > 5.0)
  {
    // msg and latest_gnss_pose_ are not of the same episode
    RCLCPP_INFO(this->get_logger(), "end of episode");
    // Print current pose and previous pose
    RCLCPP_INFO(this->get_logger(), "current pose: %f, %f, %f", msg->pose.pose.position.x, msg->pose.pose.position.y,
                msg->pose.pose.position.z);
    RCLCPP_INFO(this->get_logger(), "previous pose: %f, %f, %f", latest_odom_.pose.pose.position.x,
                latest_odom_.pose.pose.position.y, latest_odom_.pose.pose.position.z);
    // publish a bool message to indicate the end of the episode
    std_msgs::msg::Bool end_of_episode_msg;
    end_of_episode_msg.data = true;
    end_of_episode_publisher_->publish(end_of_episode_msg);
  }
  latest_odom_ = *msg;

  odom_publisher_->publish(latest_odom_);
}

void Ros2BagAnalyzer::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // check if there is a huge jump in the gnss pose, which indicates the end of the episode
  cloud_publisher_->publish(*msg);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ros2BagAnalyzer>());
  rclcpp::shutdown();
  return 0;
}