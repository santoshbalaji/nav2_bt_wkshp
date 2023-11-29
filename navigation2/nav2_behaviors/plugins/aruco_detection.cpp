#include <cmath>
#include <chrono>
#include <memory>
#include <functional>
#include <cstdlib>
#include <fstream>
#include "opencv2/opencv.hpp"


// #include "aruco_detection.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
// #include "tf2/utils.h"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// #include "nav2_util/node_utils.hpp"
// #include "nav2_msgs/msg/aruco_markers.hpp"

#include "nav2_behaviors/plugins/aruco_detection.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav2_msgs/msg/aruco_markers.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/aruco.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "opencv2/core.hpp"

using namespace std::chrono_literals;
bool isEmptyMarkerList = false;
nav2_msgs::msg::ArucoMarkers markers;
geometry_msgs::msg::PoseStamped aruco_marker_pose_;
rclcpp::Subscription<nav2_msgs::msg::ArucoMarkers>::SharedPtr aruco_marker_sub_;
rclcpp::Publisher<nav2_msgs::msg::ArucoMarkers>::SharedPtr  aruco_marker_pub_;
auto sub_node = rclcpp::Node::make_shared("sub_node");
double trans_x = std::numeric_limits<double>::quiet_NaN();
double trans_y = std::numeric_limits<double>::quiet_NaN();

namespace nav2_behaviors
{

ArucoDetection::ArucoDetection()
: TimedBehavior<ArucoDetectionAction>(),
  feedback_(std::make_shared<ArucoDetectionAction::Feedback>())
{
}


ArucoDetection::~ArucoDetection()
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
        // RCLCPP_INFO(node->get_logger(), "Arucomarker visible callback");
          if (msg->poses.empty()) {
                isEmptyMarkerList = true;
                // RCLCPP_INFO(node->get_logger(),"Marker List is empty");
                trans_x = std::numeric_limits<double>::quiet_NaN();
                trans_y = std::numeric_limits<double>::quiet_NaN();
            } else {
                isEmptyMarkerList = false;
                markers = *msg;
                aruco_marker_pose_.pose = msg->poses[0];
                // RCLCPP_INFO(node->get_logger(),"Marker List is not empty");

                trans_x = aruco_marker_pose_.pose.position.z;
                trans_y = aruco_marker_pose_.pose.position.x;
                // RCLCPP_INFO(node->get_logger(), "trans_x: %f and trans_y: %f", trans_x,trans_y);
            }
    };

    // Create subscription
    aruco_marker_sub_ = node->create_subscription<nav2_msgs::msg::ArucoMarkers>(
        "aruco_markers", 10, callback);

    rclcpp::spin(node); // Start spinning for the subscription
    return Status::SUCCEEDED;
}


void ArucoDetection::onConfigure()
{
  // auto sub_node = rclcpp::Node::make_shared("sub_node");
  if (!sub_node) {
    throw std::runtime_error{"Failed to lock sub_node"};
  }
   // Thread to create aruco detection node
  std::thread t1(aruco_recognition_launch_task, logger_);
  t1.detach();
  
  //Thread to create a subscriber to aruco_markers
  std::thread t2(aruco_marker_sub_task, sub_node);
  t2.detach();

  // auto pub_node = rclcpp::Node::make_shared("pub_node");
  // if (!pub_node) {
  //   throw std::runtime_error{"Failed to lock node"};
  // }
  // //Thread to create a subscriber to aruco_markers
  // std::thread t3(aruco_marker_pub_task, pub_node);
  // t3.detach();

}


Status ArucoDetection::onRun([[maybe_unused]] const std::shared_ptr<const ArucoDetectionAction::Goal> command)
{
  return Status::SUCCEEDED;
}


Status ArucoDetection::onCycleUpdate()
{
  int pub_count = 0;
  while(pub_count<=5){
    if(!std::isnan(trans_x) && !std::isnan(trans_y)){
      RCLCPP_INFO(sub_node->get_logger(), "trans_x: %f and trans_y: %f", trans_x,trans_y);
      pub_count++;
    }
    else{
      RCLCPP_INFO(sub_node->get_logger(), "No aruco marker detected");
    }
  }

  return Status::SUCCEEDED;
}


void ArucoDetection::onCleanup()
{
  // Shutdown subscription(s)
  if (aruco_marker_sub_) {
        aruco_marker_sub_.reset(); // Reset the subscription
    }
}


}  // namespace nav2_behaviors


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::ArucoDetection, nav2_core::Behavior)
