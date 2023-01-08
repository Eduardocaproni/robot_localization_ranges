#include <robot_localization_ranges/ros_filter_ranges.h>
#include <rclcpp/rclcpp.hpp>
#include <anchor_msgs/msg/range_with_covariance.hpp>

using namespace robot_localization_ranges;

// Questions:
// How do we consider the case when the vehicle is out of range for an anchor?
class AnchorSubscriber : public rclcpp::Node
{
  public:

    AnchorSubscriber(std::string topic, int _anchor_number, auto _beacons) : Node("subscriber")
    {
      anchor_number = _anchor_number;
      this->beacons = _beacons;
      // Create a subscriber to the specified topic, using the RangeWithCovariance message type
      subscription_ = this->create_subscription<anchor_msgs::msg::RangeWithCovariance>(
        topic, 10,
        std::bind(&AnchorSubscriber::anchorCallback, this, std::placeholders::_1));
    }; // What should I do with the second argument

  private:
    int anchor_number;
    std::map<std::string, Anchor> beacons;

    void anchorCallback(const anchor_msgs::msg::RangeWithCovariance::SharedPtr msg){
      // std::string frame_id = msg->header.frame_id;
      // Anchor anchor = beacons[frame_id];
      // anchor.latest_reading = msg;
      // std::cout << "got message for the " << frame_id << std::endl;
    }

    rclcpp::Subscription<anchor_msgs::msg::RangeWithCovariance>::SharedPtr subscription_;
};

// RosFilterRanges::createSubscriber(){
// }

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
  AnchorSubscriber("/r2d2/ranges", beacon_number, this->beacons);
  const std::chrono::duration<double> timespan{1.0 / frequency_};
  timer_ = rclcpp::GenericTimer<rclcpp::VoidCallbackType>::make_shared(
        this->get_clock(), std::chrono::duration_cast<std::chrono::nanoseconds>(timespan),
        std::bind(&RosFilterRanges::updateBaseAndRanges, this),
        this->get_node_base_interface()->get_context());


  // end of scope
}
