#include <robot_localization_ranges/ros_filter_ranges.h>
#include <rclcpp/rclcpp.hpp>
#include <anchor_msgs/msg/range_with_covariance.hpp>
#include <iostream>

#include "robot_localization/ekf.hpp"
#include "robot_localization/ukf.hpp"

using anchor_msgs::msg::RangeWithCovariance;
using namespace robot_localization_ranges;

void RosFilterRanges::anchorCallback(const RangeWithCovariance::SharedPtr msg){
      std::string frame_id = msg->header.frame_id;
      ranges.push_back(*msg);
      RCLCPP_INFO(this->get_logger(), "[PushBack] Ranges size: %i", ranges.size());
}

RosFilterRanges::RosFilterRanges(const rclcpp::NodeOptions & options)
  : robot_localization::RosFilter<robot_localization::Ekf>(options)
{
  std::vector<std::string> range_topics;

  std::map<std::string, Anchor> _beacons;

  size_t range_info{0};
  int beacon_number;

  // declare_parameter("pose0_config", rclcpp::PARAMETER_BOOL_ARRAY);
  while(true)
  {
    const auto param = "range" + std::to_string(range_info);
    const auto topic = declare_parameter<std::string>(param, "");

    if(!topic.empty())
    {
      range_topics.push_back(topic);

      const auto anchors = declare_parameter<std::vector<std::string>>(param + "_anchors", {});
      beacon_number = anchors.size();
      std::cout << "got range topic: '" << topic << "' with "
                << anchors.size() << " anchors" << std::endl;
      for(auto &anchor: anchors)
      {
          const auto x = declare_parameter<double>(param + "_" + anchor + ".pose.x",{});
          const auto y = declare_parameter<double>(param + "_" + anchor + ".pose.y",{});
          const auto z = declare_parameter<double>(param + "_" + anchor + ".pose.z",{});
          _beacons[anchor] = Anchor{x,y,z};
      }

      range_info++;
    }
    else
    {
      //undeclare_parameter(param);
      break;
    }
    this->beacons = _beacons;
  }

  range_sub_ = this->create_subscription<RangeWithCovariance>(
        "/r2d2/ranges", 10,
        std::bind(&RosFilterRanges::anchorCallback, this, std::placeholders::_1));

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
  auto mode = get_parameter("pose0_config").as_bool_array();
  estimate_x = int(mode[0]);
  estimate_y = int(mode[1]);
  estimate_z = int(mode[2]);
  RCLCPP_INFO(this->get_logger(), "Estimate_x: %b Estimate_y: %b Estimate_z: %b", estimate_x, estimate_y, estimate_z);
  RCLCPP_INFO(this->get_logger(), "Ranges size: %i", ranges.size());

  for(const auto &range: ranges)
  {
    RCLCPP_INFO(this->get_logger(), "[Estimation] Starting");
    // beacon frame  
    if(!this->tf_buffer_->canTransform(map_frame_id_, range.header.frame_id, tf2::TimePointZero)){
      RCLCPP_INFO(this->get_logger(), "Transformation from %s to %s does not exists", map_frame_id_.c_str(), range.header.frame_id.c_str());
      continue;
    }
      
    RCLCPP_INFO(this->get_logger(), "Transformation from %s to %s exists", map_frame_id_.c_str(), range.header.frame_id.c_str());
    // beacon XYZ position in world frame
    const auto beacon = tf_buffer_->lookupTransform(map_frame_id_, range.header.frame_id, tf2::TimePointZero).transform.translation;

    //getting actual state
    auto x_hat = getFilter().getState();
    auto x_hat_covariance = getFilter().getEstimateErrorCovariance();

    // Handle nan and inf values in measurements
    if (std::isnan(range.range)) {
      std::cout << "Value was nan. Excluding from update.\n";
      continue;
    }
    if (std::isinf(range.range)) {
        std::cout <<"Value was inf. Excluding from update.\n";
        continue;
    }

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

    measurement_covariance_subset(0, 0) = range.covariance;
    // Handle negative (read: bad) covariances in the measurement. Rather
    // than exclude thrangee measurement or make up a covariance, just take
    // the absolute value.
    if (measurement_covariance_subset(0,0) < 0) {
        measurement_covariance_subset(0, 0) *=-1;
    }
    //If the measurement variance for a given variable is very
    //near 0 (as in e-50 or so) and the variance for that
    //variable in the covariance matrix is also near zero, then
    //the Kalman gain computation will blow up. Really, no
    //measurement can be completely without error, so add a small
    //amount in that case.
    if (measurement_covariance_subset(0, 0) < 1e-9) {
      measurement_covariance_subset(0, 0) = 1e-9;
    }

    for(int i = 0; i<update_size_;i++){
      state_subset(i) = x_hat(i);
      // RCLCPP_INFO(this->get_logger(), "[Estimation] X_hat dimension 0: %f", state_subset(0));
      for(int j = 0; j<update_size_; j++){
          P_subset(i,j) = x_hat_covariance(i,j);
          // RCLCPP_INFO(this->get_logger(), "[Estimation] X_hat covariance i: %f j: %f value: %f", i, j, x_hat_covariance(i,j));
      }
    }

    double dist = sqrt(estimate_x*(pow(x_hat(0)-beacon.x,2) + 
                        estimate_y*pow(x_hat(1)-beacon.y,2)) + 
                        estimate_z*pow(x_hat(2)-beacon.z,2));

    C(0) = estimate_x*(x_hat(0) - beacon.x)/dist;
    C(1) = estimate_y*(x_hat(1) - beacon.y)/dist;
    C(2) = estimate_z*(x_hat(2) - beacon.z)/dist;

    RCLCPP_INFO(this->get_logger(), "[Estimation] Z:"); RCLCPP_INFO(this->get_logger(), "%d", estimate_z*x_hat(2)); 
    RCLCPP_INFO(this->get_logger(), "[Estimation] Beacon Z:"); RCLCPP_INFO(this->get_logger(), "%d", estimate_z*beacon.z);
    // RCLCPP_INFO(this->get_logger(), "[Estimation] C2:"); RCLCPP_INFO(this->get_logger(), C(2));

    // (1) Compute the Kalman gain: K = (PC') / (CPC' + R) 
    Eigen::MatrixXd pht =
      P_subset * C.transpose();
    Eigen::MatrixXd hphr_inverse =
      (C * pht + measurement_covariance_subset).inverse();
    kalman_gain_subset.noalias() = pht * hphr_inverse;
    Eigen::VectorXd y_hat(measurement_size_);
    y_hat(0)= dist;

    innovation_subset = (measurement_subset - y_hat);
    RCLCPP_INFO(this->get_logger(), "[Estimation] Innovation:");std::cout<<"trying :"<<innovation_subset<<std::endl;

    //(2) Check Mahalanobis distance between mapped measurement and state.
    if (MahalanobisThreshold(
        innovation_subset, hphr_inverse,
        mahalanobis_dist_))
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
      for(int i = 0; i<update_size_;i++){
          x_hat(i) = state_subset(i);
          RCLCPP_INFO(this->get_logger(), "[Estimation] X_hat i: %d value: %f", i, state_subset(i));
      }
      for(int i = 0; i<update_size_;i++){
        for(int j = 0; j<update_size_; j++){
          x_hat_covariance(i,j) = P_subset(i,j);
        }
      }
      getFilter().setState(x_hat);
      getFilter().setEstimateErrorCovariance(x_hat_covariance);
    }else{
        std::cout<<"measure discarted by mahalonibis"<<std::endl;
    }
  }
  ranges.clear();
}


bool RosFilterRanges::MahalanobisThreshold(
  const Eigen::VectorXd & innovation,
  const Eigen::MatrixXd & innovation_covariance, const double mahalanobis_dist)
{
  double squared_mahalanobis =
    innovation.dot(innovation_covariance * innovation);
  double threshold = mahalanobis_dist * mahalanobis_dist;

  if (squared_mahalanobis >= threshold) 
    return false;

  return true;
}
