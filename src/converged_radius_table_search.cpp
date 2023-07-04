// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include <Eigen/Dense>
#include <chrono>
#include <pcl/filters/random_sample.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "converged_radius_table_search/converged_radius_table_search.hpp"

namespace converged_radius_table_search {
ConvergedRadiusTableSearchNode::ConvergedRadiusTableSearchNode()
    : Node("converged_radius_table_search_node") {
  initPubSub();
  setParam();
  getParam();
  readCsv();
}

void ConvergedRadiusTableSearchNode::initPubSub() {
  pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/localization/pose_estimator/random_pointcloud",
      rclcpp::SensorDataQoS());

  sub_ekf_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/localization/pose_twist_fusion_filter/pose", 1,
      std::bind(&ConvergedRadiusTableSearchNode::ekfPoseCb, this,
                std::placeholders::_1));

  sub_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/localization/util/downsample/pointcloud",
      rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&ConvergedRadiusTableSearchNode::pointcloudCb, this,
                std::placeholders::_1));
}

void ConvergedRadiusTableSearchNode::setParam() {
  // clang-format off
  this->declare_parameter("csv_tabele_path", "/home/tatsuhiroikebe/rosbag/rosbag-replay-simulation/converged-radius-table-search-1/converged_radius_table.csv");
  // clang-format on
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

  auto start = std::chrono::high_resolution_clock::now();

  auto pose = findClosestPoseStamped(msg->header);
  auto converge_radius = findClosestPoseAndIdentifyConvergenceRadius(pose);

  sensor_msgs::msg::PointCloud2 output_msg;

  if (converge_radius >= 0.5) {
    pcl::PCLPointCloud2 pcl_cloud;
    pcl_conversions::toPCL(*msg, pcl_cloud);
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> temp_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_cloud, *temp_cloud);

    pcl::RandomSample<pcl::PointXYZ> random_sample;
    random_sample.setInputCloud(temp_cloud);
    random_sample.setSample(750);
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_filtered(
        new pcl::PointCloud<pcl::PointXYZ>);
    random_sample.filter(*cloud_filtered);

    pcl::PCLPointCloud2 pcl_cloud_filtered;
    pcl::toPCLPointCloud2(*cloud_filtered, pcl_cloud_filtered);
    pcl_conversions::fromPCL(pcl_cloud_filtered, output_msg);
    output_msg.header.frame_id = msg->header.frame_id;

    pub_cloud_->publish(output_msg);
  }

  pub_cloud_->publish(*msg);

  auto end = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double, std::milli> elapsed = end - start;

  RCLCPP_INFO(get_logger(), "Exe time: %f ms", elapsed.count());
  RCLCPP_INFO(get_logger(), "Converge Radius: %f", converge_radius);
  std::cout << "\n";
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
    field_map.insert(std::make_pair("pose_y", std::stod(field_vec[1])));
    field_map.insert(
        std::make_pair("converge_radius", std::stod(field_vec[2])));

    csv_tabele_.push_back(field_map);
  }

  file_->close();
}

geometry_msgs::msg::PoseStamped
ConvergedRadiusTableSearchNode::findClosestPoseStamped(
    std_msgs::msg::Header target) {
  auto target_time = target.stamp.nanosec;
  // RCLCPP_INFO(get_logger(), "findClosestPoseStamped start");

  auto copy_ekf_pose_queue = ekf_pose_queue_;

  double duration_sec_old = 0;
  while (!ekf_pose_queue_.empty()) {
    auto ekf_pose_time = ekf_pose_queue_.front().header.stamp.nanosec;
    auto duration_sec_latest = fabs(target_time - ekf_pose_time);
    if (duration_sec_old != 0) {
      if (duration_sec_old < duration_sec_latest) {
        eraseEkfPoseQueue();
        return ekf_pose_queue_.front();
      } else {
        ekf_pose_queue_.pop();
        return copy_ekf_pose_queue.back();
      }
    }
    duration_sec_old = duration_sec_latest;
  }

  // RCLCPP_INFO(get_logger(), "findClosestPoseStamped end");
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

  RCLCPP_INFO(get_logger(), "Index: %d", min_index);

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