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
#include <pcl/common/transforms.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <Eigen/Geometry>
#include <Eigen/Dense>

class StitchNode : public rclcpp::Node
{
public:
  StitchNode() : Node("StitchNode")
  {
    // subscribe to pre-processed pointcloud data from camera
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/segmented_pointcloud_data", 10,
        std::bind(&StitchNode::topic_callback, this, std::placeholders::_1));
    
    // publish combined data
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/combined_PC_data", 10);

    // get camera transformation 
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    stitched_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    count = 0;
  }

private:

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  int count;

  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // get the transform between the camera and the world
    geometry_msgs::msg::TransformStamped transform;
    try
    {
      //frames: camera_frame   camera_link    camera_link_optical    lens
      transform = tf_buffer_->lookupTransform("camera_link", "world", tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
      return;
    }

    // extract the point cloud from the message
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);

    // convert the transform to Eigen::Affine3f
    Eigen::Affine3f transform1 = transformToEigen_(transform, -.2, 0, 0);
    Eigen::Affine3f transform2 = rotateRPY(count++); // increment based on position of camera
    
    // concatenate the new point cloud to the existing one
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed2(new pcl::PointCloud<pcl::PointXYZ>());
    // obtain the point cloud in the world frame
    pcl::transformPointCloud(*cloud, *transformed, transform1);
    pcl::transformPointCloud(*transformed, *transformed2, transform2);

    // combine old and new point clouds
    *stitched_cloud = *stitched_cloud + *transformed2;

    // //print the size of the stitched point cloud 
    RCLCPP_INFO(this->get_logger(), "Stitched PC size: %ld", stitched_cloud->size());
    RCLCPP_INFO(this->get_logger(), "New data PC size: %ld", cloud->size());

    // convert the stitched point cloud to ROS message
    sensor_msgs::msg::PointCloud2 stitched_msg;
    pcl::toROSMsg(*stitched_cloud, stitched_msg);
    stitched_msg.header.frame_id = "world";
    stitched_msg.header.stamp = this->get_clock()->now();


    // publish the stitched point cloud
    publisher_->publish(stitched_msg);
    RCLCPP_INFO(this->get_logger(), "Stitched PC published");

  }

// helper function to apply camera frame transformation with offsets
Eigen::Affine3f transformToEigen_(const geometry_msgs::msg::TransformStamped &transform_stamped, 
                                       float roll_offset, float pitch_offset, float yaw_offset)
{
    // Extract translation from the TransformStamped
    Eigen::Translation3f translation(
        transform_stamped.transform.translation.x,
        transform_stamped.transform.translation.y,
        transform_stamped.transform.translation.z - 0.65);  //radius of rotation

    // Extract quaternion from the TransformStamped
    Eigen::Quaternionf rotation(
        transform_stamped.transform.rotation.w,
        transform_stamped.transform.rotation.x,
        transform_stamped.transform.rotation.y,
        transform_stamped.transform.rotation.z);

    // Create a rotation matrix from the roll, pitch, and yaw offsets
    Eigen::Matrix3f rpy_offset_matrix;
    rpy_offset_matrix = 
        Eigen::AngleAxisf(roll_offset, Eigen::Vector3f::UnitX()) *  // Roll (X-axis)
        Eigen::AngleAxisf(pitch_offset, Eigen::Vector3f::UnitY()) * // Pitch (Y-axis)
        Eigen::AngleAxisf(yaw_offset, Eigen::Vector3f::UnitZ());    // Yaw (Z-axis)

    // Combine the quaternion rotation with the roll, pitch, yaw offset
    Eigen::Matrix3f final_rotation_matrix = rpy_offset_matrix * rotation.toRotationMatrix(); 

    // Combine translation and the final rotation matrix into an Affine transform
    Eigen::Affine3f transform = translation * final_rotation_matrix;

    return transform;
}

// helper function to apply additional offsets based on motion of camera
Eigen::Affine3f rotateRPY(int count)
{
    // identity translation
    Eigen::Translation3f translation(0, 0, 0);
   
    // Create a rotation matrix from camera rotating around object
    Eigen::Matrix3f rpy_offset_matrix;
    rpy_offset_matrix = 
        Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()) *  // Roll (X-axis)
        Eigen::AngleAxisf(-(float)count*6.28/5.0, Eigen::Vector3f::UnitY()) * // Pitch (Y-axis)
        Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());    // Yaw (Z-axis)

    // Combine the quaternion rotation with the roll, pitch, yaw offset
    Eigen::Matrix3f final_rotation_matrix = rpy_offset_matrix;

    // Combine translation and the final rotation matrix into an Affine transform
    Eigen::Affine3f transform = translation * final_rotation_matrix;

    return transform;
}

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr stitched_cloud;
};

int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);
  auto node = std::make_shared<StitchNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return (0);
}
