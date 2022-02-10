#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <robot_localization/ekf.hpp>

namespace robot_localization_ranges
{
    
class EKFNode : public rclcpp::Node
{
public:
    EKFNode(rclcpp::NodeOptions options) : rclcpp::Node("ekf", options)
  {}

private:
    robot_localization::Ekf ekf;

};
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(robot_localization_ranges::EKFNode)
