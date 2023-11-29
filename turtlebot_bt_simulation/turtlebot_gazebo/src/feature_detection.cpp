#include <turtlebot_gazebo/feature_detection.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/colors.h>

#include <vector>
#include <utility>
#include <cmath>

using std::placeholders::_1;

namespace turtlebot_gazebo
{
  FeatureDetection::FeatureDetection() : Node("feature_detection_node")
  {
    rmw_qos_profile_t custom_subscriber_qos = rmw_qos_profile_default;
    custom_subscriber_qos.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;

    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(
      custom_subscriber_qos.history,
      RMW_QOS_POLICY_DEPTH_SYSTEM_DEFAULT),
      custom_subscriber_qos);

    unfiltered_point_cloud_subscription_ =
      create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud",
        qos,
        std::bind(
          &FeatureDetection::unfiltered_point_cloud_subscription_callback,
          this,
          _1
        )
      );
    filtered_point_cloud_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
        "/filtered_cloud",
        10
      );
  }

  void FeatureDetection::unfiltered_point_cloud_subscription_callback(
    sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // converting ros2 pointcloud message type to pcl pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(
      new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud_in);

    // creating kd tree for the pointcloud
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud_in);

    // clustering points in point cloud for identifying the legs of items to pickup
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ece;
    ece.setClusterTolerance(0.20); // 20cm
    ece.setMinClusterSize(1);
    ece.setMaxClusterSize(10);
    ece.setSearchMethod(tree);
    ece.setInputCloud(cloud_in);
    ece.extract(cluster_indices);

    RCLCPP_INFO(rclcpp::get_logger("feature_detection"), "ece indices %ld", cluster_indices.size());

    // creating different point cloud instances based on clusters and adding different color 
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_clusters;
    for (const auto& cluster : cluster_indices)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(
        new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::RGB rgb = pcl::getRandomColor();
      for (const auto& idx : cluster.indices)
      {
        (*cloud_in)[idx].r = rgb.r;
        (*cloud_in)[idx].g = rgb.g;
        (*cloud_in)[idx].b = rgb.b;
        cloud_cluster->push_back((*cloud_in)[idx]);
      }
      cloud_clusters.push_back(cloud_cluster);
    }

    std::vector<std::pair<float, float>> length_points; 
    std::vector<std::pair<float, float>> breadth_points; 

    for (int i = 0; i < cloud_clusters.size(); i++)
    {
      for (int j = i + 1; j < cloud_clusters.size(); j++)
      {
        if (cloud_clusters.at(i)->size() == 0 || 
            cloud_clusters.at(j)->size() == 0)
        {
          break;
        }

        float x1 = cloud_clusters.at(i)->at(0).x;    
        float y1 = cloud_clusters.at(i)->at(0).y;    
        float x2 = cloud_clusters.at(j)->at(0).x;    
        float y2 = cloud_clusters.at(j)->at(0).y;

        float distance = sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)));
        distance = fabs(distance);

        RCLCPP_INFO(rclcpp::get_logger("feature_detection"), "distance %f", distance);

        if (distance < 1.40 && distance > 1.20)
        {
          std::pair<float, float> first_point_pair;
          first_point_pair.first = x1;
          first_point_pair.second = y1;

          std::pair<float, float> second_point_pair;
          second_point_pair.first = x2;
          second_point_pair.second = y2;

          length_points.push_back(first_point_pair);
          length_points.push_back(second_point_pair);
        }
        else if (distance < 0.80 && distance > 0.67)
        {
          std::pair<float, float> first_point_pair;
          first_point_pair.first = x1;
          first_point_pair.second = y1;

          std::pair<float, float> second_point_pair;
          second_point_pair.first = x2;
          second_point_pair.second = y2;

          breadth_points.push_back(first_point_pair);
          breadth_points.push_back(second_point_pair);
        } 
      }
    }

    RCLCPP_INFO(rclcpp::get_logger("feature_detection"), "length %ld", length_points.size());
    RCLCPP_INFO(rclcpp::get_logger("feature_detection"), "breadth %ld", breadth_points.size());

    if (length_points.size() == 4 && breadth_points.size() == 4)
    {
      float x1 = ((length_points.at(0).first + length_points.at(1).first) / 2);
      float y1 = ((length_points.at(0).second + length_points.at(1).second) / 2);

      float x2 = ((length_points.at(2).first + length_points.at(3).first) / 2);
      float y2 = ((length_points.at(2).second + length_points.at(3).second) / 2);

      float x3 = ((breadth_points.at(0).first + breadth_points.at(1).first) / 2);
      float y3 = ((breadth_points.at(0).second + breadth_points.at(1).second) / 2);

      float x4 = ((breadth_points.at(2).first + breadth_points.at(3).first) / 2);
      float y4 = ((breadth_points.at(2).second + breadth_points.at(3).second) / 2);

      float cent_x, cent_y;
      if (fabs(x1 - x2) < fabs(y1 - y2))
      {
        cent_y = ((y1 + y2) / 2);
      }
      else
      {
        cent_x = ((x1 + x2) / 2);
      }

      if (fabs(x3 - x4) < fabs(y3 - y4))
      {
        cent_y = ((y3 + y4) / 2);
      }
      else
      {
        cent_x = ((x3 + x4) / 2);
      }

      RCLCPP_INFO(rclcpp::get_logger("feature_detection"), "cent_x %f", cent_x);
      RCLCPP_INFO(rclcpp::get_logger("feature_detection"), "cent_y %f", cent_y);
    }

    sensor_msgs::msg::PointCloud2::SharedPtr cloud_out(
      new sensor_msgs::msg::PointCloud2());
    pcl::toROSMsg(*cloud_in, *cloud_out);
    unsigned int num_points_out = cloud_out->width;
    cloud_out->header.frame_id = msg->header.frame_id;
    cloud_out->header.stamp = msg->header.stamp;
    filtered_point_cloud_publisher_->publish(*cloud_out);

    exit(0);
  }
}

int main(int argc, char * argv[]) 
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<turtlebot_gazebo::FeatureDetection>());
  rclcpp::shutdown();
  return 0;
}
