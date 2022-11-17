#include <rclcpp/rclcpp.hpp>
#include <robot_localization_ranges/ros_filter_ranges.h>
#include <robot_localization_ranges/ekf.h>

#include <robot_localization/ros_filter_types.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <string>
#include <memory>
#include <vector>

using namespace robot_localization_ranges;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.arguments({"ekf_filter_node"});
  auto filter{std::make_shared<RosFilterRanges>(options)};
  //auto filter{std::make_shared<robot_localization::RosFilter<Ekf>>(options)};
  //auto filter{std::make_shared<robot_localization::RosFilter<robot_localization::Ekf>>(options)};
  filter->initialize();
  rclcpp::spin(filter->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
