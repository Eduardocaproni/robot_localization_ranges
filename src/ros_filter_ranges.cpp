#include <robot_localization_ranges/ros_filter_ranges.h>

using namespace robot_localization_ranges;

RosFilterRanges::RosFilterRanges(const rclcpp::NodeOptions & options)
  : robot_localization::RosFilter<Ekf>(options)
{


}
