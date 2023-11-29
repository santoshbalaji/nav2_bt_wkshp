
#include <string>
#include <memory>
#include <cmath>

#include "nav2_behavior_tree/plugins/action/aruco_tracking_action.hpp"

namespace nav2_behavior_tree
{

  ArucoTrackingAction::ArucoTrackingAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BtActionNode<nav2_msgs::action::ArucoTracking>(xml_tag_name, action_name, conf)
  {
    std::string aruco_tracking;
    getInput("aruco_tracking",aruco_tracking);
  }


  void ArucoTrackingAction::on_tick()
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
      return std::make_unique<nav2_behavior_tree::ArucoTrackingAction>(name, "aruco_tracking", config);
    };


  factory.registerBuilder<nav2_behavior_tree::ArucoTrackingAction>("ArucoTracking", builder);
}

