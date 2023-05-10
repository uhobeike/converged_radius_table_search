// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#ifndef CONVERGED_RADIUS_TABLE_SEARCH__CONVERGED_RADIUS_TABLE_SEARCH_HPP_
#define CONVERGED_RADIUS_TABLE_SEARCH__CONVERGED_RADIUS_TABLE_SEARCH_HPP_

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
  void initSubcription();
  void ekfPoseCb(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void pointcloudCb(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  rclcpp::Time findClosestTimestamp(std_msgs::msg::Header target);
  void eraseEkfPoseQueue();

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      sub_ekf_pose_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      sub_pointcloud_;

  std::queue<geometry_msgs::msg::PoseStamped> ekf_pose_queue_;
};

} // namespace converged_radius_table_search

#endif // CONVERGED_RADIUS_TABLE_SEARCH__CONVERGED_RADIUS_TABLE_SEARCH_HPP_
