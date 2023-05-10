// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include "converged_radius_table_search/converged_radius_table_search.hpp"

namespace converged_radius_table_search {
ConvergedRadiusTableSearchNode::ConvergedRadiusTableSearchNode()
    : Node("converged_radius_table_search_node") {
  initSubcription();
}

void ConvergedRadiusTableSearchNode::initSubcription() {
  sub_ekf_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/localization/pose_twist_fusion_filter/pose", 1,
      std::bind(&ConvergedRadiusTableSearchNode::ekfPoseCb, this,
                std::placeholders::_1));

  sub_pointcloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/localization/util/downsample/pointcloud", 1,
      std::bind(&ConvergedRadiusTableSearchNode::pointcloudCb, this,
                std::placeholders::_1));
}

void ConvergedRadiusTableSearchNode::ekfPoseCb(
    geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
  ekf_pose_queue_.push(*msg);
}

void ConvergedRadiusTableSearchNode::pointcloudCb(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  findClosestTimestamp(msg->header);
}

rclcpp::Time ConvergedRadiusTableSearchNode::findClosestTimestamp(
    std_msgs::msg::Header target) {
  rclcpp::Time target_time(target.stamp);

  double duration_sec_old = INFINITY;
  while (!ekf_pose_queue_.empty()) {
    rclcpp::Time ekf_pose_time(ekf_pose_queue_.front().header.stamp);
    auto duration_sec_latest = (target_time - ekf_pose_time).seconds();

    if (duration_sec_old != INFINITY) {
      if (duration_sec_old > duration_sec_latest) {
        return rclcpp::Time(ekf_pose_queue_.front().header.stamp);
        break;
      } else
        ekf_pose_queue_.pop();
    }
    duration_sec_old = duration_sec_latest;
  }

  eraseEkfPoseQueue();

  return rclcpp::Time();
}

void ConvergedRadiusTableSearchNode::eraseEkfPoseQueue() {
  ekf_pose_queue_ = std::queue<geometry_msgs::msg::PoseStamped>();
}

} // namespace converged_radius_table_search

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<
          converged_radius_table_search::ConvergedRadiusTableSearchNode>());
  rclcpp::shutdown();
  return 0;
}