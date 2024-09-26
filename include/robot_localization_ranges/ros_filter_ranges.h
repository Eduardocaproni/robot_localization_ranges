#ifndef ROBOT_LOCALIZATION_RANGES_ROS_FILTER_RANGES_H
#define ROBOT_LOCALIZATION_RANGES_ROS_FILTER_RANGES_H

#include <map_simulator/msg/range.hpp>

#include <robot_localization/ros_filter.hpp>
#include <robot_localization/ekf.hpp>

// #include "robot_localization/ekf.hpp"
// #include "robot_localization/ukf.hpp"

// #include "robot_localization/filter_base.hpp"
// #include "robot_localization/filter_common.hpp"

using map_simulator::msg::Range;

namespace robot_localization_ranges
{

struct Anchor
{
  double x,y,z;
};

/// this class inherits from the classical EKF node
/// it parses the range-related parameters
/// also handles range measurements and their impact on the underlying EKF

class RosFilterRanges : public robot_localization::RosFilter<robot_localization::Ekf>
{
public:
  explicit RosFilterRanges(const rclcpp::NodeOptions & options);

  ~RosFilterRanges() = default;

  std::map<std::string, Anchor> beacons;

protected:

  void createSubscribers();

  inline void updateAll()
  {
    periodicUpdate();
    rangeUpdate();
  }

  void rangeUpdate();
  
  inline static bool MahalanobisThreshold(const Eigen::VectorXd & innovation,
                            const Eigen::MatrixXd & innovation_covariance)
  {
    const auto squared_mahalanobis{innovation.dot(innovation_covariance * innovation)};
    const auto threshold{mahalanobis_dist_ * mahalanobis_dist_};
    return squared_mahalanobis < threshold;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<rclcpp::Subscription<Range>::SharedPtr> range_sub_;
  std::vector<Range> ranges;
  int update_size_ = 3; //x, y, z
  int measurement_size_ = 1;   //d
  constexpr static auto mahalanobis_dist_{1.6449}; //sqrt 90% chi2 inverse n = 1
  
  int estimate_x = false;
  int estimate_y = false;
  int estimate_z = false;
  
  const Eigen::MatrixXd identity_ =  Eigen::MatrixXd::Identity(update_size_,update_size_ );
};

}

#endif // ROBOT_LOCALIZATION_RANGES_ROS_FILTER_RANGES_H
