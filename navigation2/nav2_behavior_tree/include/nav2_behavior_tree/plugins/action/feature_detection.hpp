#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__FEATURE_DETECTION_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__FEATURE_DETECTION_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/feature_detection.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::FeatureDetection
 */
class FeatureDetectionAction : public BtActionNode<nav2_msgs::action::FeatureDetection>
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::FeatureDetectionAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  FeatureDetectionAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<int>("wait_duration", 1, "Wait time")
      });
  }
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__FEATURE_DETECTION_ACTION_HPP_
