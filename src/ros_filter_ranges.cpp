#include <robot_localization_ranges/ros_filter_ranges.h>
#include <rclcpp/rclcpp.hpp>
#include <anchor_msgs/msg/range_with_covariance.hpp>
#include <iostream>
#include <random>

#include <robot_localization/ekf.hpp>

using anchor_msgs::msg::RangeWithCovariance;
using namespace robot_localization_ranges;

void RosFilterRanges::anchorCallback(const RangeWithCovariance::SharedPtr msg){
  ranges.push_back(*msg);
}

RosFilterRanges::RosFilterRanges(const rclcpp::NodeOptions & options)
  : robot_localization::RosFilter<robot_localization::Ekf>(options)
{
  std::vector<std::string> range_topics;

  size_t range_info{0};
  while(true)
  {
    const auto param = "range" + std::to_string(range_info);
    const auto topic = declare_parameter<std::string>(param, "");

    if(topic.empty())
      break;

    range_topics.push_back(topic);

    const auto anchors{declare_parameter<std::vector<std::string>>(param + "_anchors", std::vector<std::string>())};
    for(auto &anchor: anchors)
    {
      const auto x{declare_parameter<double>(param + "_" + anchor + ".pose.x",{})};
      const auto y{declare_parameter<double>(param + "_" + anchor + ".pose.y",{})};
      const auto z{declare_parameter<double>(param + "_" + anchor + ".pose.z",{})};
      beacons[anchor] = {x,y,z};
    }
    range_info++;
  }

  for(const auto &topic: range_topics)
  {
    range_sub_.push_back(this->create_subscription<RangeWithCovariance>(
                           topic, rclcpp::SensorDataQoS(),
                           std::bind(&RosFilterRanges::anchorCallback, this, std::placeholders::_1)));
  }

  const std::chrono::duration<double> timespan{1.0 / frequency_};

  timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(timespan),
        std::bind(&RosFilterRanges::updateAll, this));
}

void RosFilterRanges::rangeUpdate()
{
  // It's not possible to read the parameter on the initialization
  // a simple workaround was reading it everytime the rangeUpdate
  // function is executed. However we should take a look at it.
  //  auto mode = get_parameter("pose0_config").as_bool_array();
  //  estimate_x = int(mode[0]);
  //  estimate_y = int(mode[1]);
  //  estimate_z = int(mode[2]);
  //  RCLCPP_INFO(this->get_logger(), "Estimate_x: %b Estimate_y: %b Estimate_z: %b", estimate_x, estimate_y, estimate_z);
  //  RCLCPP_INFO(this->get_logger(), "Ranges size: %i", ranges.size());

  for(const auto &range: ranges)
  {
    // Handle nan and inf values in measurements
    if (std::isnan(range.range) || std::isinf(range.range) || range.range < 0)
      continue;

    // beacon frame
    if(!this->tf_buffer_->canTransform(map_frame_id_, range.header.frame_id, tf2::TimePointZero))
      continue;
    
    // beacon XYZ position in world frame
    const auto beacon = tf_buffer_->lookupTransform(map_frame_id_, range.header.frame_id, tf2::TimePointZero).transform.translation;

    //getting actual state
    auto x_hat = getFilter().getState();
    auto x_hat_covariance = getFilter().getEstimateErrorCovariance();

    //----------------Extended Kalman Filter--------------------
    // Now set up the relevant matrices
    Eigen::VectorXd state_subset(update_size_);       // x (in most literature)
    Eigen::VectorXd measurement_subset(measurement_size_);  // y
    Eigen::MatrixXd measurement_covariance_subset(measurement_size_, measurement_size_);  // R
    Eigen::MatrixXd C(measurement_size_, update_size_);  // C
    Eigen::MatrixXd kalman_gain_subset(update_size_, measurement_size_ );          // K
    Eigen::VectorXd innovation_subset(measurement_size_);  // y - Cx
    Eigen::MatrixXd P_subset(update_size_, update_size_ ); //P

    state_subset.setZero();
    measurement_subset.setZero();
    measurement_covariance_subset.setZero();
    C.setZero();
    kalman_gain_subset.setZero();
    innovation_subset.setZero();
    P_subset.setZero();


    // Now build the sub-matrices from the full-sized matrices
    measurement_subset(0) = range.range;

    // Handle negative (read: bad) covariances in the measurement. Rather
    // than exclude the range measurement or make up a covariance, just take
    // the absolute value.
    // also ensure a minimum value for the covariance
    measurement_covariance_subset(0, 0) = std::max(1e-9, std::abs(range.covariance));

    for(int i = 0; i< update_size_;i++)
    {
      state_subset(i) = x_hat(i);
      for(int j = 0; j< update_size_; j++)
        P_subset(i,j) = x_hat_covariance(i,j);
    }

    const auto dist = sqrt(estimate_x*(pow(x_hat(0)-beacon.x,2) +
                           estimate_y*pow(x_hat(1)-beacon.y,2)) +
                           estimate_z*pow(x_hat(2)-beacon.z,2));

    if (dist == 0){
      C(0) = 0;
      C(1) = 0;
      C(2) = 0;
    } else
    {
      C(0) = estimate_x*(x_hat(0) - beacon.x)/dist;
      C(1) = estimate_y*(x_hat(1) - beacon.y)/dist;
      C(2) = estimate_z*(x_hat(2) - beacon.z)/dist;
    }

    // (1) Compute the Kalman gain: K = (PC') / (CPC' + R)
    Eigen::MatrixXd pht =
        P_subset * C.transpose();
    Eigen::MatrixXd hphr_inverse =
        (C * pht + measurement_covariance_subset).inverse();
    kalman_gain_subset.noalias() = pht * hphr_inverse;
    Eigen::VectorXd y_hat(measurement_size_);
    y_hat(0)= dist;

    innovation_subset = (measurement_subset - y_hat);
    //    RCLCPP_INFO(this->get_logger(), "[Estimation] Innovation:");std::cout<<"trying :"<<innovation_subset<<std::endl;

    //(2) Check Mahalanobis distance between mapped measurement and state.
    if (MahalanobisThreshold(innovation_subset, hphr_inverse))
    {
      // (3) Apply the gain to the difference between the state and measurement: x
      // = x + K(y - y_hat)
      state_subset.noalias() += kalman_gain_subset * innovation_subset;

      // (4) Update the estimate error covariance using the Joseph form: (I -
      // KC)P(I - KC)' + KRK'
      Eigen::MatrixXd gain_residual = identity_;
      gain_residual.noalias() -= kalman_gain_subset * C;
      P_subset =
          gain_residual * P_subset * gain_residual.transpose();
      P_subset.noalias() += kalman_gain_subset *
          measurement_covariance_subset *
          kalman_gain_subset.transpose();

      //(5) Affect real state and covariance
      for(int i = 0; i<update_size_;i++)
      {
        x_hat(i) = state_subset(i);
        for(int j = 0; j<update_size_; j++)
          x_hat_covariance(i,j) = P_subset(i,j);
      }
      getFilter().setState(x_hat);
      getFilter().setEstimateErrorCovariance(x_hat_covariance);
    }
  }
  ranges.clear();
}
