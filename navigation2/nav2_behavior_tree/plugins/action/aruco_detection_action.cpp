
#include <string>
#include <memory>
#include <cmath>

#include "nav2_behavior_tree/plugins/action/aruco_detection_action.hpp"

namespace nav2_behavior_tree
{


  ArucoDetectionAction::ArucoDetectionAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BtActionNode<nav2_msgs::action::ArucoDetection>(xml_tag_name, action_name, conf)
  {
    std::string aruco_detection;
    getInput("aruco_detection",aruco_detection);
  }


  void ArucoDetectionAction::on_tick()
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
      return std::make_unique<nav2_behavior_tree::ArucoDetectionAction>(name, "aruco_detection", config);
    };


  factory.registerBuilder<nav2_behavior_tree::ArucoDetectionAction>("ArucoDetection", builder);
}


