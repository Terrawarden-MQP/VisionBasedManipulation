#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h> 
#include <pcl/filters/passthrough.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std::chrono_literals;

class PlaneNode : public rclcpp::Node
{
  public:
    PlaneNode() : Node("PlaneNode")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
         "/move_camera/points", 
         10, 
         std::bind(&PlaneNode::topic_callback, this, std::placeholders::_1));

      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/segmented_pointcloud_data", 10);

    }

  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "Recieved PointCloud2");

      RCLCPP_INFO(this->get_logger(), "Width: %u, Height: %u", msg->width, msg->height);

      // Convert cloud from ROS message to PCL object
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::fromROSMsg(*msg, *cloud);

      // Distance Filter to eliminate 'Max Dist' point cloud (pass in cloud)
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
      
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud (cloud);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (0.01, 1.5);
      //pass.setNegative (true);
      pass.filter (*cloud_filtered);
      // end of Distance filter (cloud_filtered = table + object)

      // Plane Segmentation Start (Inliers = Table, Outliers = Object)
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      // Create the segmentation object
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      // Optional
      seg.setOptimizeCoefficients (true);
      // Mandatory
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (0.01);
      
      // Isolate the inliers of the point clous (distance filtered)
      seg.setInputCloud (cloud_filtered);
      seg.segment (*inliers, *coefficients);
      
      // extract the Inliers from segmented model (Table) <-- Not needed for application (commmented out)
      extract.setInputCloud(cloud_filtered);
      extract.setIndices(inliers);
      // extract.setNegative(false);
      // pcl::PointCloud<pcl::PointXYZ>::Ptr InlierCloud(new pcl::PointCloud<pcl::PointXYZ>);
      // extract.filter(*InlierCloud);

      // extract the Outliers from segmented model (Object) <-- Desired output of Node
      extract.setNegative(true);
      pcl::PointCloud<pcl::PointXYZ>::Ptr OutlierCloud(new pcl::PointCloud<pcl::PointXYZ>);
      extract.filter(*OutlierCloud);

      // If there are no inliers, publish an error (not a problem usually)
      if (inliers->indices.size () == 0)
      {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
        //return (-1);
      }

      // Convert the PCL object to ROS message
      sensor_msgs::msg::PointCloud2 seg_msg;
      pcl::toROSMsg(*OutlierCloud, seg_msg);
      publisher_->publish(seg_msg);
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main (int argc, char **argv) {

  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlaneNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return (0);
}
