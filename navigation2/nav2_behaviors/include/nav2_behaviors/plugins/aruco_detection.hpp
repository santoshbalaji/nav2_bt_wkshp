#ifndef NAV2_BEHAVIORS__PLUGINS__ARUCO_DETECTION_HPP_
#define NAV2_BEHAVIORS__PLUGINS__ARUCO_DETECTION_HPP_


#include <chrono>
#include <string>
#include <memory>


#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/aruco_detection.hpp"
#include "std_msgs/msg/string.hpp"


namespace nav2_behaviors
{
	using ArucoDetectionAction = nav2_msgs::action::ArucoDetection;


	class ArucoDetection : public TimedBehavior<ArucoDetectionAction>
	{
	public:
	  ArucoDetection();
	  ~ArucoDetection();


	  Status onRun(const std::shared_ptr<const ArucoDetectionAction::Goal> command) override;


	  Status onCycleUpdate() override;


	  void onCleanup() override;


	  void onConfigure() override;


	protected:

	  // rclcpp::Publisher publisher_;
	  std::string message;
	  ArucoDetectionAction::Feedback::SharedPtr feedback_;

	};


}  // namespace nav2_behaviors


#endif //NAV2_BEHAVIOR_PLUGINS__ARUCO_DETECTION_HPP_
