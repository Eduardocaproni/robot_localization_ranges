#include <robot_localization_ranges/ros_filter_ranges.h>
#include <rclcpp/rclcpp.hpp>
#include <anchor_msgs/msg/range_with_covariance.hpp>

using anchor_msgs::msg::RangeWithCovariance;
using namespace robot_localization_ranges;

// Questions:
// How do we consider the case when the vehicle is out of range for an anchor?

void RosFilterRanges::anchorCallback(const RangeWithCovariance::SharedPtr msg){
      std::string frame_id = msg->header.frame_id;
      Anchor anchor = beacons[frame_id];
      anchor.latest_reading = msg;
}

RosFilterRanges::RosFilterRanges(const rclcpp::NodeOptions & options)
  : robot_localization::RosFilter<robot_localization::Ekf>(options)
{
  std::vector<std::string> range_topics;

  std::map<std::string, Anchor> _beacons;

  size_t range_info{0};
  int beacon_number;
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
      std::cout << std::endl;

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
  timer_ = rclcpp::GenericTimer<rclcpp::VoidCallbackType>::make_shared(
        this->get_clock(), std::chrono::duration_cast<std::chrono::nanoseconds>(timespan),
        std::bind(&RosFilterRanges::updateAll, this),
        this->get_node_base_interface()->get_context());


  // end of scope
}

void RosFilterRanges::rangeUpdate()
{
  // for(const auto &range: ranges)
  // {
  //   // beacon frame
  //   if(!this->tf_buffer_->canTransform(world_frame_id_, range.header.frame_id, tf2::TimePointZero))
  //     continue;

  //   // beacon XYZ position in world frame
  //   const auto beacon = tf_buffer_->lookupTransform(world_frame_id_, range.header.frame_id, tf2::TimePointZero).transform.translation;

  //   //range.range
  //   //range.covariance

  //   //getFilter().getState()
  //   //getFilter().getEstimateErrorCovariance()

  //   //getFilter().setState()
  // }
}