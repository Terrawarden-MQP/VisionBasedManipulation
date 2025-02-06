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
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/filter.h>
#include <vector>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
// #include <pcl/features/boundary.h>
// #include <pcl/search/kdtree.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/surface/convex_hull.h>
// #include <pcl/features/principal_curvatures.h>
#include <Eigen/Core>
#include <chrono>

class PointCloudClusterDetector : public rclcpp::Node {
public:
    PointCloudClusterDetector() : Node("extract_cluster") {
        // ROS parameters
        this->declare_parameter<std::string>("pointcloud_topic", "/realsense/points");
        this->declare_parameter<std::string>("coord_topic", "/target_2d_coords");
        this->declare_parameter<std::string>("cluster_topic", "/detected_cluster");
        this->declare_parameter<std::string>("camera_info_topic", "/realsense/camera_info");
        this->declare_parameter<bool>("visualize", false);
        this->declare_parameter<double>("crop_radius", 0.2);
        this->declare_parameter<int>("sor_mean_k", 50);
        this->declare_parameter<double>("sor_stddev_mul_thresh", 1.0);
        this->declare_parameter<double>("voxel_leaf_size", 0.01);
        this->declare_parameter<int>("ransac_max_iterations", 1000);
        this->declare_parameter<double>("ransac_distance_threshold", 0.01);
        this->declare_parameter<std::string>("header_frame","camera_link");
        this->declare_parameter<double>("cluster_tolerance", 0.02);
        this->declare_parameter<int>("min_cluster_size", 100);
        this->declare_parameter<int>("max_cluster_size", 25000);
        this->declare_parameter<double>("target_point_tolerance",0.02);

        // Retrieve ROS parameters
        pointcloud_topic = this->get_parameter("pointcloud_topic").as_string();
        coord_topic = this->get_parameter("coord_topic").as_string();
        cluster_topic = this->get_parameter("cluster_topic").as_string();
        camera_info_topic = this->get_parameter("camera_info_topic").as_string();
        VISUALIZE = this->get_parameter("visualize").as_bool();
        crop_radius = this->get_parameter("crop_radius").as_double();
        sor_mean_k = this->get_parameter("sor_mean_k").as_int();
        sor_stddev_mul_thresh = this->get_parameter("sor_stddev_mul_thresh").as_double();
        voxel_leaf_size = this->get_parameter("voxel_leaf_size").as_double();
        ransac_max_iterations = this->get_parameter("ransac_max_iterations").as_int();
        ransac_distance_threshold = this->get_parameter("ransac_distance_threshold").as_double();
        header_frame = this->get_parameter("header_frame").as_string();
        cluster_tolerance = this->get_parameter("cluster_tolerance").as_double();
        min_cluster_size = this->get_parameter("min_cluster_size").as_int();
        max_cluster_size = this->get_parameter("max_cluster_size").as_int();
        target_point_tolerance = this->get_parameter("target_point_tolerance").as_double();

        // Subscriptions + Publishers 
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic, 10, std::bind(&PointCloudClusterDetector::pointcloud_callback, this, std::placeholders::_1));
        coord_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            coord_topic, 10, std::bind(&PointCloudClusterDetector::coord_callback, this, std::placeholders::_1));
        camera_info_subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic, 10, 
            [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "Camera RGB image width: %d, height: %d", msg->width, msg->height);
                image_width_ = msg->width;
                image_height_ = msg->height;
                // Unsubscribe after receiving the data once
                camera_info_subscription_.reset();
            });
        cluster_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cluster_topic, 10);
        if(VISUALIZE){
            crop_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/visualize/crop", 10);
            sor_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/visualize/sor", 10);
            voxel_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/visualize/voxel", 10);
            plane_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/visualize/plane", 10);        
        }
    }

private:
    std::string pointcloud_topic, coord_topic, cluster_topic, camera_info_topic, header_frame;
    double crop_radius, sor_stddev_mul_thresh, voxel_leaf_size, ransac_distance_threshold, cluster_tolerance, target_point_tolerance;
    bool VISUALIZE = false;
    int sor_mean_k, ransac_max_iterations, min_cluster_size, max_cluster_size;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr coord_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_pub_, crop_pub_, sor_pub_, voxel_pub_, plane_pub_;    
    sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_data_;
    std::pair<int, int> latest_2d_point_;
    int image_width_, image_height_;

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "Point cloud received!");
        pointcloud_data_ = msg;
    }

    void coord_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        latest_2d_point_ = std::make_pair(static_cast<int>(msg->x), static_cast<int>(msg->y));
        RCLCPP_DEBUG(this->get_logger(), "New 2D point received: (%d, %d)", latest_2d_point_.first, latest_2d_point_.second);

        if (pointcloud_data_) {
            auto t1 = std::chrono::high_resolution_clock::now();
            process_coordinates();
            auto t2 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double,std::milli> elapsed = t2-t1;
            RCLCPP_INFO(this->get_logger(),"Cluster_time elapsed from receiving point: %f",elapsed.count());            
        } else {
            RCLCPP_WARN(this->get_logger(), "No point cloud received yet");
        }
    }

    void process_coordinates() {
        int u = latest_2d_point_.first;
        int v = latest_2d_point_.second;

        // Get dimensions of the image
        int width = image_width_; 
        int height = image_height_; 

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

        // Filter cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_processed(new pcl::PointCloud<pcl::PointXYZ>);

        // Crop cloud to region around point of interest
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_crop(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::CropBox<pcl::PointXYZ> crop;
        crop.setInputCloud(cloud);
        float radius = crop_radius; // meters
        crop.setMin(Eigen::Vector4f(target_point.x - radius, target_point.y - radius, target_point.z - radius, 0));
        crop.setMax(Eigen::Vector4f(target_point.x + radius, target_point.y + radius, target_point.z + radius, 0));
        crop.filter(*cloud_crop);
        // RCLCPP_DEBUG(this->get_logger(), "Crop box found with %lu points with size %f", cloud_crop->points.size(),(1.5-(count*0.1)));

        // Remove noise using a Statistical Outlier Removal filter
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sor(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud_crop);
        sor.setMeanK(sor_mean_k);  // Number of neighbors to analyze for each point
        sor.setStddevMulThresh(sor_stddev_mul_thresh);  // Standard deviation multiplier
        sor.filter(*cloud_sor);

        // Downsample point cloud using VoxelGrid filter
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud_sor);
        vg.setLeafSize(voxel_leaf_size,voxel_leaf_size,voxel_leaf_size); 
        vg.filter(*cloud_voxel);

        // Remove ground (plane segmentation)
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients);
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (ransac_max_iterations); 
        seg.setDistanceThreshold (ransac_distance_threshold);
        // Segment the largest planar component from the cropped cloud
        seg.setInputCloud (cloud_voxel);
        seg.segment (*inliers, *coeff);
        // coefficients = coeff; // Store plane coefficients if desired
        if (inliers->indices.size () == 0)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to estimate planar model");
        }
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud_voxel);
        extract.setIndices(inliers);
        extract.setNegative (true); // false = return plane
        extract.filter (*cloud_processed);

        if(VISUALIZE){
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_processed_inverted(new pcl::PointCloud<pcl::PointXYZ>);
            // Invert remove ground (plane segmentation)
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients);
            seg.setOptimizeCoefficients (true);
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setMaxIterations (ransac_max_iterations);
            seg.setDistanceThreshold (ransac_distance_threshold);
            // Segment the largest planar component from the cropped cloud
            seg.setInputCloud (cloud_voxel);
            seg.segment (*inliers, *coeff);
            // coefficients = coeff; // Store plane coefficients if desired
            if (inliers->indices.size () == 0)
            {
                RCLCPP_WARN(this->get_logger(), "Failed to estimate planar model");
            }
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud (cloud_voxel);
            extract.setIndices(inliers);
            extract.setNegative (false); // false = return plane
            extract.filter (*cloud_processed_inverted);

            sensor_msgs::msg::PointCloud2 cloud_msg;
            pcl::toROSMsg(*cloud_crop, cloud_msg);
            cloud_msg.header.frame_id = header_frame; 
            cloud_msg.header.stamp = this->now();
            crop_pub_->publish(cloud_msg); 
            pcl::toROSMsg(*cloud_sor, cloud_msg);
            sor_pub_->publish(cloud_msg);
            pcl::toROSMsg(*cloud_voxel,cloud_msg);
            voxel_pub_->publish(cloud_msg);
            pcl::toROSMsg(*cloud_processed_inverted,cloud_msg);
            plane_pub_->publish(cloud_msg);
        }

        // Find the object cluster containing this point
        auto cluster = find_object_cluster(cloud_processed, target_point);
        if (cluster) {
            RCLCPP_INFO(this->get_logger(), "Cluster found with %lu points", cluster->points.size());
            sensor_msgs::msg::PointCloud2 cluster_msg;
            pcl::toROSMsg(*cluster, cluster_msg);
            cluster_msg.header.frame_id = header_frame;
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
        ec.setClusterTolerance(cluster_tolerance); // 2cm
        ec.setMinClusterSize(min_cluster_size);
        ec.setMaxClusterSize(max_cluster_size);
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
            double tolerance = target_point_tolerance;
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
