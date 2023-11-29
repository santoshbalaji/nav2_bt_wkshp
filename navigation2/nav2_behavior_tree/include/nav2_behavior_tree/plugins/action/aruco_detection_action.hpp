#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__ARUCO_DETECTION_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__ARUCO_DETECTION_ACTION_HPP_


#include <string>


#include "std_msgs/msg/string.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/aruco_detection.hpp"

namespace nav2_behavior_tree
{


class ArucoDetectionAction : public BtActionNode<nav2_msgs::action::ArucoDetection>
{
public:
  ArucoDetectionAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);


  void on_tick() override;


  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<std::string>("aruco_detection", "Nothing", "Aruco Detection")
      });
  }
};


}  // namespace nav2_behavior_tree


#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__ARUCO_DETECTION_ACTION_HPP_


