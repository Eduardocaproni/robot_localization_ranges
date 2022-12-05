#ifndef ROBOT_LOCALIZATION_RANGES_ROS_FILTER_RANGES_H
#define ROBOT_LOCALIZATION_RANGES_ROS_FILTER_RANGES_H

#include <robot_localization/ros_filter.hpp>
#include <robot_localization_ranges/ekf.h>

namespace robot_localization_ranges
{

/// this class is a sub-class of the classical EKF node
/// it should parse the range-related parameters
/// also should handle range measurements and their impact on the underlying EKF

class RosFilterRanges : public robot_localization::RosFilter<robot_localization::Ekf>
{
public:
  explicit RosFilterRanges(const rclcpp::NodeOptions & options);
  
  ~RosFilterRanges() = default;



protected:
  
};

}

#endif // ROBOT_LOCALIZATION_RANGES_ROS_FILTER_RANGES_H
