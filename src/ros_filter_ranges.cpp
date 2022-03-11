#include <robot_localization_ranges/ros_filter_ranges.h>

using namespace robot_localization_ranges;

template <class Filter>
RosFilterRanges<Filter>::RosFilterRanges(const rclcpp::NodeOptions & options)
  : robot_localization::RosFilter<Filter>(options)
{


}
