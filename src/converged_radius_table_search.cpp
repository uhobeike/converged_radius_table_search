// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include <Eigen/Dense>

#include "converged_radius_table_search/converged_radius_table_search.hpp"

namespace converged_radius_table_search {
ConvergedRadiusTableSearchNode::ConvergedRadiusTableSearchNode()
    : Node("converged_radius_table_search_node") {
  initSubcription();
  readCsv();
}

void ConvergedRadiusTableSearchNode::initSubcription() {
  sub_ekf_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/localization/pose_twist_fusion_filter/pose", 1,
      std::bind(&ConvergedRadiusTableSearchNode::ekfPoseCb, this,
                std::placeholders::_1));

  sub_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/localization/util/downsample/pointcloud", 1,
      std::bind(&ConvergedRadiusTableSearchNode::pointcloudCb, this,
                std::placeholders::_1));
}

void ConvergedRadiusTableSearchNode::setParam() {
  this->declare_parameter("csv_tabele_path", "");
}

void ConvergedRadiusTableSearchNode::getParam() {
  csv_path_ = this->get_parameter("csv_tabele_path").as_string();
}

void ConvergedRadiusTableSearchNode::ekfPoseCb(
    geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
  ekf_pose_queue_.push(*msg);
}

void ConvergedRadiusTableSearchNode::pointcloudCb(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  auto pose = findClosestPoseStamped(msg->header);
  auto converge_radius = findClosestPoseAndIdentifyConvergenceRadius(pose);

  RCLCPP_INFO(get_logger(), "Converge Radius: %f", converge_radius);
}

void ConvergedRadiusTableSearchNode::readCsv() {
  file_ = std::make_shared<std::ifstream>(csv_path_);

  if (!file_->is_open())
    RCLCPP_ERROR(get_logger(), "Could not open file: %s\n", csv_path_.c_str());

  std::string line, field_str;
  while (std::getline(*file_, line)) {
    std::stringstream ss(line);
    std::vector<std::string> field_vec;
    while (std::getline(ss, field_str, ',')) {
      field_vec.push_back(field_str);
    }

    std::map<std::string, double> field_map;
    field_map.insert(std::make_pair("pose_x", std::stod(field_vec[0])));
    field_map.insert(std::make_pair("pose_y", std::stod(field_vec[0])));
    field_map.insert(
        std::make_pair("converge_radius", std::stod(field_vec[0])));

    csv_tabele_.push_back(field_map);
  }

  file_->close();
}

geometry_msgs::msg::PoseStamped
ConvergedRadiusTableSearchNode::findClosestPoseStamped(
    std_msgs::msg::Header target) {
  rclcpp::Time target_time(target.stamp);

  double duration_sec_old = INFINITY;
  while (!ekf_pose_queue_.empty()) {
    rclcpp::Time ekf_pose_time(ekf_pose_queue_.front().header.stamp);
    auto duration_sec_latest = (target_time - ekf_pose_time).seconds();

    if (duration_sec_old != INFINITY) {
      if (duration_sec_old > duration_sec_latest) {
        eraseEkfPoseQueue();
        return ekf_pose_queue_.front();
      } else
        ekf_pose_queue_.pop();
    }
    duration_sec_old = duration_sec_latest;
  }
  return geometry_msgs::msg::PoseStamped();
}

void ConvergedRadiusTableSearchNode::eraseEkfPoseQueue() {
  ekf_pose_queue_ = std::queue<geometry_msgs::msg::PoseStamped>();
}

double
ConvergedRadiusTableSearchNode::findClosestPoseAndIdentifyConvergenceRadius(
    geometry_msgs::msg::PoseStamped target_pose) {
  Eigen::Vector2d target_pose_mat(target_pose.pose.position.x,
                                  target_pose.pose.position.y);

  std::vector<double> l2_norm_vec;
  for (auto data : csv_tabele_) {
    Eigen::Vector2d reference_mat(data["pose_x"], data["pose_y"]);
    l2_norm_vec.push_back((target_pose_mat - reference_mat).norm());
  }
  auto min_element_iter =
      std::min_element(l2_norm_vec.begin(), l2_norm_vec.end());
  int min_index = std::distance(l2_norm_vec.begin(), min_element_iter);
  return csv_tabele_[min_index]["converge_radius"];
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