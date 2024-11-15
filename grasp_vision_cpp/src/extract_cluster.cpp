/*
Vision pipeline to extract object clusters. 

Subscribes to camera depth cloud and separate topic broadcasting center of target object in 2D.
Convert 2D coordinates to 3D points in point cloud. Use Euclidean clustering to detect clusters
and publish the object containing the target 3D point.

Special thanks to: 
https://github.com/IntelRealSense/librealsense/issues/11031#issuecomment-1352879033
https://support.intelrealsense.com/hc/en-us/community/posts/24972964701331--finding-3d-coordinates-of-a-given-point-which-is-specified-in-the-2d-pixel-coordinates [NOT USED]
https://github.com/yehengchen/DOPE-ROS-D435 [NOT USED]
https://medium.com/@pacogarcia3/calculate-x-y-z-real-world-coordinates-from-image-coordinates-using-opencv-from-fdxlabs-0adf0ec37cef [TODO]

*/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/filter.h>
#include <vector>

class PointCloudClusterDetector : public rclcpp::Node {
public:
    PointCloudClusterDetector() : Node("extract_cluster") {
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/realsense/points", 10, std::bind(&PointCloudClusterDetector::pointcloud_callback, this, std::placeholders::_1)); // TODO

        coord_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/target_2d_coords", 10, std::bind(&PointCloudClusterDetector::coord_callback, this, std::placeholders::_1));

        cluster_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/detected_cluster", 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr coord_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_pub_;
    sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_data_;
    std::pair<int, int> latest_2d_point_;

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Point cloud received!");
        pointcloud_data_ = msg;
    }

    void coord_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        latest_2d_point_ = std::make_pair(static_cast<int>(msg->x), static_cast<int>(msg->y));
        RCLCPP_INFO(this->get_logger(), "New 2D point received: (%d, %d)", latest_2d_point_.first, latest_2d_point_.second);

        if (pointcloud_data_) {
            process_coordinates();
        } else {
            RCLCPP_WARN(this->get_logger(), "No point cloud received yet");
        }
    }

    void process_coordinates() {
        int u = latest_2d_point_.first;
        int v = latest_2d_point_.second;

        // Get dimensions of the image
        int width = 640; // pointcloud_data_->width;
        int height = 480; // pointcloud_data_->height;
        // TODO get from topic

        // Calculate index within point cloud array
        int index = v * width + u;

        // Convert PointCloud2 to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*pointcloud_data_, *cloud);

        if (index >= static_cast<int>(cloud->points.size()) || index < 0) {
            RCLCPP_WARN(this->get_logger(), "Index %d is out of bounds for point cloud data %ld", index, cloud->points.size());
            return;
        } 

        // Retrieve the 3D point corresponding to the 2D coordinates
        pcl::PointXYZ target_point = cloud->points[index];
        if (!std::isfinite(target_point.x) || !std::isfinite(target_point.y) || !std::isfinite(target_point.z)) {
            RCLCPP_WARN(this->get_logger(), "Invalid 3D point (%.3f, %.3f, %.3f) at index %d",target_point.x, target_point.y, target_point.z, index);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Converted 3D Point: (%.3f, %.3f, %.3f)", target_point.x, target_point.y, target_point.z);

        // Find the object cluster containing this point
        auto cluster = find_object_cluster(cloud, target_point);
        if (cluster) {
            RCLCPP_INFO(this->get_logger(), "Cluster found with %lu points", cluster->points.size());
            sensor_msgs::msg::PointCloud2 cluster_msg;
            pcl::toROSMsg(*cluster, cluster_msg);
            cluster_msg.header.frame_id = "camera_link";  // TODO check
            cluster_msg.header.stamp = this->now();
            cluster_pub_->publish(cluster_msg);
        } else {
            RCLCPP_INFO(this->get_logger(), "No cluster :/");
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr find_object_cluster(
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
            const pcl::PointXYZ &target_point) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud_filtered, indices);

        // Set up clustering parameters
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        tree->setInputCloud(cloud_filtered);
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.02); // 2cm
        ec.setMinClusterSize(100);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        
        // Perform clustering
        std::vector<pcl::PointIndices> cluster_indices;
        ec.setInputCloud(cloud_filtered);
        ec.extract(cluster_indices);

        // Check if target point is in any of the clusters
        for (const auto& indices : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>());
            for (int index : indices.indices) {
                cluster->points.push_back(cloud->points[index]);
            }

            // Check if target_point is in this cluster
            double tolerance = 0.02;
            for (const auto& point : cluster->points) {
                if (std::fabs(point.x - target_point.x) < tolerance &&
                    std::fabs(point.y - target_point.y) < tolerance &&
                    std::fabs(point.z - target_point.z) < tolerance) {
                    return cluster;
                }
            }
        }
        return nullptr;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudClusterDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
