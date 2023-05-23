// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#ifndef CONVERGED_RADIUS_TABLE_SEARCH__CONVERGED_RADIUS_TABLE_SEARCH_HPP_
#define CONVERGED_RADIUS_TABLE_SEARCH__CONVERGED_RADIUS_TABLE_SEARCH_HPP_

#include <fstream>
#include <map>
#include <queue>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

namespace converged_radius_table_search {

class ConvergedRadiusTableSearchNode : public rclcpp::Node {
public:
  ConvergedRadiusTableSearchNode();

protected:
  void initPubSub();
  void setParam();
  void getParam();
  void ekfPoseCb(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void pointcloudCb(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void readCsv();

  geometry_msgs::msg::PoseStamped
  findClosestPoseStamped(std_msgs::msg::Header target);
  void eraseEkfPoseQueue();

  double findClosestPoseAndIdentifyConvergenceRadius(
      geometry_msgs::msg::PoseStamped target_pose);

  void publishCloud();

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      sub_ekf_pose_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      sub_pointcloud_;

  std::queue<geometry_msgs::msg::PoseStamped> ekf_pose_queue_;

  std::shared_ptr<std::ifstream> file_;
  std::string csv_path_;
  std::vector<std::map<std::string, double>> csv_tabele_;
};

} // namespace converged_radius_table_search

#endif // CONVERGED_RADIUS_TABLE_SEARCH__CONVERGED_RADIUS_TABLE_SEARCH_HPP_
