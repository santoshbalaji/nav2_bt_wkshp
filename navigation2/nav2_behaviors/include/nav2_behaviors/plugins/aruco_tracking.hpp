#ifndef NAV2_BEHAVIORS__PLUGINS__ARUCO_TRACKING_HPP_
#define NAV2_BEHAVIORS__PLUGINS__ARUCO_TRACKING_HPP_


#include <chrono>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/aruco_tracking.hpp"
#include "std_msgs/msg/string.hpp"


namespace nav2_behaviors
{
	using ArucoTrackingAction = nav2_msgs::action::ArucoTracking;


	class ArucoTracking : public TimedBehavior<ArucoTrackingAction>
	{
	public:
	  ArucoTracking();
	  ~ArucoTracking();


	  Status onRun(const std::shared_ptr<const ArucoTrackingAction::Goal> command) override;


	  Status onCycleUpdate() override;


	  void onCleanup() override;


	  void onConfigure() override;


	protected:
	// rclcpp::Publisher publisher_;
	std::string message;
	ArucoTrackingAction::Feedback::SharedPtr feedback_;
	// rclcpp::Subscription<nav2_msgs::msg::ArucoMarkers>::SharedPtr aruco_marker_sub_;

    double min_rotational_vel_;
    double max_rotational_vel_;
    double rotational_acc_lim_;
    double cmd_yaw_;
    double prev_yaw_;
    double relative_yaw_;
    double simulate_ahead_time_;
	};


}  // namespace nav2_behaviors


#endif //NAV2_BEHAVIOR_PLUGINS__ARUCO_TRACKING_HPP_
