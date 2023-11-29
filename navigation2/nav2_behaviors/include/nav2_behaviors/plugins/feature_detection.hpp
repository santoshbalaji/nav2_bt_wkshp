#ifndef NAV2_BEHAVIORS__PLUGINS__FEATURE_DETECTION_HPP_
#define NAV2_BEHAVIORS__PLUGINS__FEATURE_DETECTION_HPP_

#include <chrono>
#include <string>
#include <memory>

#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/feature_detection.hpp"

namespace nav2_behaviors
{
using FeatureDetectionAction = nav2_msgs::action::FeatureDetection;

/**
 * @class nav2_behaviors::FeatureDetection
 * @brief An action server behavior for waiting a fixed duration
 */
class FeatureDetection : public TimedBehavior<FeatureDetectionAction>
{
public:
  /**
   * @brief A constructor for nav2_behaviors::FeatureDetection
   */
  FeatureDetection();
  ~FeatureDetection();

  /**
   * @brief Initialization to run behavior
   * @param command Goal to execute
   * @return Status of behavior
   */
  Status onRun(const std::shared_ptr<const FeatureDetectionAction::Goal> command) override;

  /**
   * @brief Loop function to run behavior
   * @return Status of behavior
   */
  Status onCycleUpdate() override;

protected:
  rclcpp::Time wait_end_;
  FeatureDetectionAction::Feedback::SharedPtr feedback_;
};

}  // namespace nav2_behaviors

#endif  // NAV2_BEHAVIORS__PLUGINS__FEATURE_DETECTION_HPP_
