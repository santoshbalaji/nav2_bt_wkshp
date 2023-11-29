#ifndef TURTLEBOT_GAZEBO__FEATURE_DETECTION_HPP_
#define TURTLEBOT_GAZEBO__FEATURE_DETECTION_HPP_

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace turtlebot_gazebo
{

class FeatureDetection : public rclcpp::Node
{
public:
  FeatureDetection();

private:
  rclcpp::Subscription<
    sensor_msgs::msg::PointCloud2>::SharedPtr unfiltered_point_cloud_subscription_;
  rclcpp::Publisher<
    sensor_msgs::msg::PointCloud2>::SharedPtr filtered_point_cloud_publisher_;

  void unfiltered_point_cloud_subscription_callback(
    sensor_msgs::msg::PointCloud2::SharedPtr msg);
};

}  // namespace turtlebot_gazebo

#endif  // TURTLEBOT_GAZEBO__FEATURE_DETECTION_HPP_
