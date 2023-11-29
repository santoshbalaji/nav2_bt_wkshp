#include <chrono>
#include <memory>
#include <iostream>

#include "nav2_behaviors/plugins/feature_detection.hpp"

namespace nav2_behaviors
{

FeatureDetection::FeatureDetection()
: TimedBehavior<FeatureDetectionAction>(),
  feedback_(std::make_shared<FeatureDetectionAction::Feedback>())
{
}

FeatureDetection::~FeatureDetection() = default;

Status FeatureDetection::onRun(const std::shared_ptr<const FeatureDetectionAction::Goal> command)
{
  wait_end_ = node_.lock()->now() + rclcpp::Duration(command->time);
  return Status::SUCCEEDED;
}

Status FeatureDetection::onCycleUpdate()
{
  auto current_point = node_.lock()->now();
  auto time_left = wait_end_ - current_point;

  feedback_->time_left = time_left;
  action_server_->publish_feedback(feedback_);

  if (time_left.nanoseconds() > 0) {
    return Status::RUNNING;
  } else {
    return Status::SUCCEEDED;
  }
}

}  // namespace nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::FeatureDetection, nav2_core::Behavior)
