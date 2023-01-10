#ifndef ROBOT_LOCALIZATION_RANGES_ROS_FILTER_RANGES_H
#define ROBOT_LOCALIZATION_RANGES_ROS_FILTER_RANGES_H
#include <anchor_msgs/msg/range_with_covariance.hpp>

#include <robot_localization/ros_filter.hpp>
#include <robot_localization_ranges/ekf.h>

namespace robot_localization_ranges
{

struct Anchor
{
  double x,y,z;
  anchor_msgs::msg::RangeWithCovariance::SharedPtr latest_reading;
};
/// this class is a sub-class of the classical EKF node
/// it should parse the range-related parameters
/// also should handle range measurements and their impact on the underlying EKF

class RosFilterRanges : public robot_localization::RosFilter<robot_localization::Ekf>
{
public:
  explicit RosFilterRanges(const rclcpp::NodeOptions & options);
  
  ~RosFilterRanges() = default;

  std::map<std::string, Anchor> beacons;

  void anchorCallback(const anchor_msgs::msg::RangeWithCovariance::SharedPtr msg);

protected:

  void createSubscribers();

  void updateBaseAndRanges()
  {
    periodicUpdate();

    // add update form ranges
    // learn how to read messages from the anchors
    // learn how to compose them with the periodic update

  };

  rclcpp::Subscription<anchor_msgs::msg::RangeWithCovariance>::SharedPtr
    range_sub_;
  
};

}

#endif // ROBOT_LOCALIZATION_RANGES_ROS_FILTER_RANGES_H
