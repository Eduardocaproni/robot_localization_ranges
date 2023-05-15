#include <rclcpp/rclcpp.hpp>
#include <robot_localization_ranges/ros_filter_ranges.h>

using namespace robot_localization_ranges;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.arguments({"ekf_filter_node"});
  auto filter{std::make_shared<RosFilterRanges>(options)};
  filter->initialize();
  rclcpp::spin(filter->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
