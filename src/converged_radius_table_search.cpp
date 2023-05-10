// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include "converged_radius_table_search/converged_radius_table_search.hpp"

namespace converged_radius_table_search {
ConvergedRadiusTableSearchNode::ConvergedRadiusTableSearchNode()
    : Node("converged_radius_table_search_node") {}
} // namespace converged_radius_table_search

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<
          converged_radius_table_search::ConvergedRadiusTableSearchNode>());
  rclcpp::shutdown();
  return 0;
}