#ifndef ROBOT_LOCALIZATION_RANGES_ROS_FILTER_RANGES_H
#define ROBOT_LOCALIZATION_RANGES_ROS_FILTER_RANGES_H
#include <anchor_msgs/msg/range_with_covariance.hpp>

#include <robot_localization/ros_filter.hpp>
#include <robot_localization_ranges/ekf.h>

// #include "robot_localization/ekf.hpp"
// #include "robot_localization/ukf.hpp"

// #include "robot_localization/filter_base.hpp"
// #include "robot_localization/filter_common.hpp"

using anchor_msgs::msg::RangeWithCovariance;

namespace robot_localization_ranges
{

struct Anchor
{
  double x,y,z;
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

  void anchorCallback(const RangeWithCovariance::SharedPtr msg);

protected:

  void createSubscribers();

  inline void updateAll()
  {
    periodicUpdate();
    rangeUpdate();
  }

  void rangeUpdate();
  
  bool MahalanobisThreshold(const Eigen::VectorXd & innovation,
                            const Eigen::MatrixXd & innovation_covariance, const double mahalanobis_dist);

  // std::unique_ptr<FilterBase> filter_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<RangeWithCovariance>::SharedPtr range_sub_;
  std::vector<RangeWithCovariance> ranges;
  int update_size_ = 3; //x, y, z
  int measurement_size_ = 1;   //d
  float mahalanobis_dist_ = 1.6449; //sqrt 90% chi2 inverse n = 1
  
  int estimate_x = false;
  int estimate_y = false;
  int estimate_z = false;
  
  const Eigen::MatrixXd identity_ =  Eigen::MatrixXd::Identity(update_size_,update_size_ );
};

}

#endif // ROBOT_LOCALIZATION_RANGES_ROS_FILTER_RANGES_H
