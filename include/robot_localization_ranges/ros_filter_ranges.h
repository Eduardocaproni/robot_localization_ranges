#ifndef ROBOT_LOCALIZATION_RANGES_ROS_FILTER_RANGES_H
#define ROBOT_LOCALIZATION_RANGES_ROS_FILTER_RANGES_H

#include <robot_localization/ros_filter.hpp>
#include <robot_localization_ranges/ekf.h>

namespace robot_localization_ranges
{

class RosFilterRanges : public robot_localization::RosFilter<Ekf>
{
public:
  explicit RosFilterRanges(const rclcpp::NodeOptions & options);
  
};

}

#endif // ROBOT_LOCALIZATION_RANGES_ROS_FILTER_RANGES_H
