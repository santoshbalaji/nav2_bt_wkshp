#include <string>
#include <memory>

#include "nav2_behavior_tree/plugins/action/feature_detection.hpp"

namespace nav2_behavior_tree
{

FeatureDetectionAction::FeatureDetectionAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::FeatureDetection>(xml_tag_name, action_name, conf)
{
  int duration;
  getInput("wait_duration", duration);
  if (duration <= 0) {
    RCLCPP_WARN(
      node_->get_logger(), "Wait duration is negative or zero "
      "(%i). Setting to positive.", duration);
    duration *= -1;
  }

  goal_.time.sec = duration;
}

void FeatureDetectionAction::on_tick()
{
  increment_recovery_count();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::FeatureDetectionAction>(name, "feature_detection", config);
    };

  factory.registerBuilder<nav2_behavior_tree::FeatureDetectionAction>("FeatureDetection", builder);
}
