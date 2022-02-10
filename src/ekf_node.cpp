#include <rclcpp/rclcpp.hpp>
#include <robot_localization_ranges/ekf.h>

namespace robot_localization_ranges
{

class EKFNode : public rclcpp::Node
{
public:
  EKFNode(rclcpp::NodeOptions options) : rclcpp::Node("ekf", options)
  {

  }

private:


};
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(robot_localization_ranges::EKFNode)
