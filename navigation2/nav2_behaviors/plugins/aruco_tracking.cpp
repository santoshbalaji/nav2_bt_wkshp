#include <cmath>
#include <chrono>
#include <memory>
#include <functional>
#include <cstdlib>
#include <fstream>


#include "nav2_behaviors/plugins/aruco_tracking.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_msgs/msg/aruco_markers.hpp"

using namespace std::chrono_literals;
bool isEmptyMarkerList = false;
geometry_msgs::msg::PoseStamped aruco_marker_pose_;
rclcpp::Subscription<nav2_msgs::msg::ArucoMarkers>::SharedPtr aruco_marker_sub_;


namespace nav2_behaviors
{
void arucoMarkerVisibleCallback(const nav2_msgs::msg::ArucoMarkers msg);

ArucoTracking::ArucoTracking()
: TimedBehavior<ArucoTrackingAction>(),
  feedback_(std::make_shared<ArucoTrackingAction::Feedback>()),
  min_rotational_vel_(0.0),
  max_rotational_vel_(0.0),
  rotational_acc_lim_(0.0),
  cmd_yaw_(0.0),
  prev_yaw_(0.0),
  relative_yaw_(0.0),
  simulate_ahead_time_(0.0)
{
}


ArucoTracking::~ArucoTracking()
{
}

// Thread to create aruco detection node
Status aruco_recognition_launch_task(rclcpp::Logger logger_){
  int systemRet = system("ros2 launch ros2_aruco aruco_recognition.launch.py");
  if (systemRet == -1){
    RCLCPP_INFO(logger_, "aruco_recognition_launch_task: Success");
    return Status::FAILED;
  }
  RCLCPP_INFO(logger_, "aruco_recognition_launch_task: Failed");
  return Status::SUCCEEDED;
}

// Thread to create aruco marker subscription
Status aruco_marker_sub_task(rclcpp::Node::SharedPtr node){
  RCLCPP_INFO(node->get_logger(), "Arucomarker subscriber task created");
  // rclcpp::Subscription<nav2_msgs::msg::ArucoMarkers>::SharedPtr aruco_marker_sub_;
  //   // Initialize subscriber here
    
    // Callback function
    auto callback = [&node](const nav2_msgs::msg::ArucoMarkers::SharedPtr msg) {
        // Process messages here
        RCLCPP_INFO(node->get_logger(), "Arucomarker visible callback");
          if (msg->poses.empty()) {
                isEmptyMarkerList = true;
                 RCLCPP_INFO(node->get_logger(),"Marker List is empty");
            } else {
                isEmptyMarkerList = false;
                aruco_marker_pose_.pose = msg->poses[0];
                 RCLCPP_INFO(node->get_logger(),"Marker List is not empty");
            }
    };

    // Create subscription
    aruco_marker_sub_ = node->create_subscription<nav2_msgs::msg::ArucoMarkers>(
        "aruco_markers", 10, callback);

    rclcpp::spin(node); // Start spinning for the subscription
    return Status::SUCCEEDED;
}

void ArucoTracking::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  auto sub_node = rclcpp::Node::make_shared("sub_node");
  if (!sub_node) {
    throw std::runtime_error{"Failed to lock sub_node"};
  }

  // Thread to create aruco detection node
  // std::thread t1(aruco_recognition_launch_task, logger_);
  // t1.detach();
  
  // Thread to create a subscriber to aruco_markers
  std::thread t2(aruco_marker_sub_task, sub_node);
  t2.detach();
 
  nav2_util::declare_parameter_if_not_declared(
    node,
    "simulate_ahead_time", rclcpp::ParameterValue(2.0));
  node->get_parameter("simulate_ahead_time", simulate_ahead_time_);

  nav2_util::declare_parameter_if_not_declared(
    node,
    "max_rotational_vel", rclcpp::ParameterValue(1.0));
  node->get_parameter("max_rotational_vel", max_rotational_vel_);

  nav2_util::declare_parameter_if_not_declared(
    node,
    "min_rotational_vel", rclcpp::ParameterValue(0.4));
  node->get_parameter("min_rotational_vel", min_rotational_vel_);

  nav2_util::declare_parameter_if_not_declared(
    node,
    "rotational_acc_lim", rclcpp::ParameterValue(3.2));
  node->get_parameter("rotational_acc_lim", rotational_acc_lim_);

  //initialize to a large number
  aruco_marker_pose_.pose.position.z = std::numeric_limits<double>::quiet_NaN();
  aruco_marker_pose_.pose.position.x = std::numeric_limits<double>::quiet_NaN();

  rclcpp::QoS qos_reliable(10);
  qos_reliable.keep_last(10);
  qos_reliable.reliable();
  qos_reliable.durability_volatile();

  auto cmdvel_pub_ = node->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel", qos_reliable);
  
}


Status ArucoTracking::onRun([[maybe_unused]] const std::shared_ptr<const ArucoTrackingAction::Goal> command)
{
  return Status::SUCCEEDED;
}


Status ArucoTracking::onCycleUpdate()
{
  // Use Right Hand Rule and Consider Camera/Tag Coordinate Frame
  // to Retrive the right position coordinate for x and y translation
  RCLCPP_INFO(logger_, "onCycleUpdate");

  double trans_x = aruco_marker_pose_.pose.position.z;
  double trans_y = aruco_marker_pose_.pose.position.x;
  RCLCPP_INFO(logger_, "trans_x: %f and trans_y: %f", trans_x,trans_y);

  double hyp = std::sqrt(std::pow(trans_x,2.0) + std::pow(trans_y,2));
  double ang_rad_diff = std::asin(trans_y/hyp);

  // Convert radians to degrees if needed
  double ang_deg_diff = 0.0;
  ang_deg_diff = ang_rad_diff * 180.0 / M_PI; // Angle in degree
  RCLCPP_INFO(logger_, "angle degrees different %f", ang_deg_diff);
  double angle_thresh = 5; // in degrees 
  double dist_thresh = 0.20; // in metres

  // initialize velocity
  double linear_vel = 0.0;
  double angular_vel = 0.0;

  bool isWithinAngleThreshold = false;
  if (ang_deg_diff > -angle_thresh && ang_deg_diff < angle_thresh){
    isWithinAngleThreshold = true;
  }

  double linear_diff = trans_x;
  bool isWithinLinearDistThreshold = false;
  if (linear_diff < dist_thresh){
    isWithinLinearDistThreshold = true;
  }
  
  RCLCPP_INFO(logger_, "isWithinLinearDistThreshold %d", isWithinLinearDistThreshold);
  RCLCPP_INFO(logger_, "isWithinAngleThreshold %d", isWithinAngleThreshold);

  if (isWithinAngleThreshold && isWithinLinearDistThreshold){
    RCLCPP_INFO(logger_, "The robot is within the linear distance and angular threshold.");
    RCLCPP_INFO(logger_, "The robot has arrived at the destination.");
    RCLCPP_INFO(logger_, "The robot will stop moving.");

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = linear_vel;
    cmd_vel.angular.z = angular_vel;

    // Publish command velocity to the robot
    RCLCPP_INFO(logger_, "Publishing command velocity");
    vel_pub_->publish(std::move(cmd_vel));
    return Status::SUCCEEDED;
  }

  // Do not move if no ar_tag found
  if (isEmptyMarkerList){
    RCLCPP_INFO(logger_, "No aruco pose available, robot will not move.");
    angular_vel = 0.0;
    linear_vel = 0.0;
  }

  /*Compute velocity required to chase tag based on distance and orientation from tag
   Write Proportinal Controller equation to reduce angular error and translational error
   Tune to proportional gain to achieve better tracking */
  else{
    RCLCPP_INFO(logger_, "Aruco pose available, computing command velocity.");
    // angular_vel = -0.5 * std::atan2(trans_y, trans_x);
    // linear_vel = 0.3 * (std::sqrt(trans_x * trans_x) - 0.3);
    angular_vel = -3 * std::atan2(trans_y, trans_x);
    linear_vel = 1.5 * (std::sqrt(trans_x * trans_x) - 0.17);
  }

  RCLCPP_INFO(logger_, "cmd_vel_linear: %f", linear_vel);
  RCLCPP_INFO(logger_, "cmd_vel_angular: %f", angular_vel);


  // Assign Computed Linear and Angular Velocity
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear_vel;
  cmd_vel.angular.z = angular_vel;

  // Publish command velocity to the robot
  RCLCPP_INFO(logger_, "Publishing command velocity");
  vel_pub_->publish(std::move(cmd_vel));

  return Status::RUNNING;
}


void ArucoTracking::onCleanup()
{
  // Shutdown subscription(s)
    if (aruco_marker_sub_) {
        aruco_marker_sub_.reset(); // Reset the subscription
    }
}

}  // namespace nav2_behaviors


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::ArucoTracking, nav2_core::Behavior)
