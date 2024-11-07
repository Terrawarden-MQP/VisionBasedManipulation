#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/search/kdtree.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <random>
#include <cmath>

using namespace std::chrono_literals;

class SurfaceNormNode : public rclcpp::Node
{
public:
    SurfaceNormNode() : Node("SurfaceNormNode"), points_selected_(false)
    {
        // Create a subscription for the segmented point cloud data
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/combined_PC_data", 10,
            std::bind(&SurfaceNormNode::surfaceNormalCallback, this, std::placeholders::_1));

        // Create a publisher for the visualization markers
        normal_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/normal_markers", 10);

        // Create a publisher for the centroid marker
        centroid_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/centroid_marker", 10);

        // Create a publisher for the grasp line marker
        grasp_line_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/grasp_line_marker", 10);

        // Create a publisher for the grasp circle marker
        grasp_circle_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/grasp_circle_marker", 10);

        RCLCPP_INFO(this->get_logger(), "Node initialized!");
    }

private:
    void surfaceNormalCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received point cloud!");

        // Convert ROS PointCloud2 message to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*cloud_msg, *cloud);

        // Remove NaN points from the point cloud
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

        if (cloud->points.empty()) {
            RCLCPP_WARN(this->get_logger(), "Point cloud is empty after removing NaNs!");
            return;
        }

        // Downsample the point cloud to reduce the number of points
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f);  // Adjust this value as needed
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        voxel_filter.filter(*cloud_filtered);

        if (cloud_filtered->points.empty()) {
            RCLCPP_WARN(this->get_logger(), "Filtered point cloud is empty!");
            return;
        }

        // Create the normal estimation class and set the input cloud
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(cloud_filtered);

        // Create an empty KdTree representation and pass it to the normal estimation object
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setSearchMethod(tree);

        // Output dataset for normals
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

        // Use all neighbors in a sphere of radius 5 cm
        ne.setRadiusSearch(0.05);

        // Compute the normals
        ne.compute(*cloud_normals);

        if (cloud_normals->points.empty()) {
            RCLCPP_WARN(this->get_logger(), "No normals were computed!");
            return;
        }

        // Create a vector to store the centroid
        Eigen::Vector4f centroid;

        // Compute the centroid
        pcl::compute3DCentroid(*cloud, centroid);

        // The centroid gives you the center of mass
        std::cout << "Center of mass (centroid): " << centroid.head<3>() << std::endl;

        // Publish the centroid marker
        visualization_msgs::msg::Marker centroid_marker;
        centroid_marker.header = cloud_msg->header;
        centroid_marker.ns = "centroid";
        centroid_marker.id = 0;
        centroid_marker.type = visualization_msgs::msg::Marker::SPHERE;
        centroid_marker.action = visualization_msgs::msg::Marker::ADD;
        centroid_marker.pose.position.x = centroid[0];
        centroid_marker.pose.position.y = centroid[1];
        centroid_marker.pose.position.z = centroid[2];
        centroid_marker.scale.x = 0.02;
        centroid_marker.scale.y = 0.02;
        centroid_marker.scale.z = 0.02;
        centroid_marker.color.a = 1.0;
        centroid_marker.color.r = 1.0;
        centroid_marker.color.g = 0.0;
        centroid_marker.color.b = 0.0;
        centroid_marker_publisher_->publish(centroid_marker);

        // Define the grasp radius for the fixed threshold circle (adjustable threshold)
        double circle_radius = 0.025;  // 5 cm radius for the grasp circle

        // Ensure points are generated symmetrically around the centroid along a direction vector
        if (!points_selected_) {
            std::random_device rd;
            std::mt19937 gen(rd());

            // Generate random direction
            std::uniform_real_distribution<> dis(-1.0, 1.0);
            direction_vector_.x = dis(gen);
            direction_vector_.y = dis(gen);
            direction_vector_.z = dis(gen);

            // Normalize the direction vector
            double magnitude = sqrt(direction_vector_.x * direction_vector_.x +
                                    direction_vector_.y * direction_vector_.y +
                                    direction_vector_.z * direction_vector_.z);
            direction_vector_.x /= magnitude;
            direction_vector_.y /= magnitude;
            direction_vector_.z /= magnitude;

            points_selected_ = true;
            RCLCPP_INFO(this->get_logger(), "Generated random direction vector for grasp line.");
        }

        // Set the length of the line segment (extend symmetrically from the centroid)
        double line_length = 0.2;  // 20 cm line

        // Compute the two points on either side of the centroid along the direction vector
        geometry_msgs::msg::Point p1, p2, center;
        center.x = centroid[0];
        center.y = centroid[1];
        center.z = centroid[2];

        p1.x = center.x - direction_vector_.x * line_length;
        p1.y = center.y - direction_vector_.y * line_length;
        p1.z = center.z - direction_vector_.z * line_length;

        p2.x = center.x + direction_vector_.x * line_length;
        p2.y = center.y + direction_vector_.y * line_length;
        p2.z = center.z + direction_vector_.z * line_length;

        // Publish a line marker representing the grasp line through the centroid
        visualization_msgs::msg::Marker grasp_line_marker;
        grasp_line_marker.header = cloud_msg->header;
        grasp_line_marker.ns = "grasp_line";
        grasp_line_marker.id = 2;
        grasp_line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        grasp_line_marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the scale and color of the line
        grasp_line_marker.scale.x = 0.01;  // Line width
        grasp_line_marker.color.a = 1.0;
        grasp_line_marker.color.r = 1.0;
        grasp_line_marker.color.g = 0.0;
        grasp_line_marker.color.b = 0.0;  // Red line

        // Add the two endpoints to the line marker (p1 -> centroid -> p2)
        grasp_line_marker.points.push_back(p1);
        grasp_line_marker.points.push_back(p2);

        // Publish the grasp line marker
        grasp_line_marker_publisher_->publish(grasp_line_marker);

        // Publish a fixed circle centered at the centroid, aligned with the normal to the surface
        visualization_msgs::msg::Marker grasp_circle_marker;
        createNormalAlignedCircleMarker(grasp_circle_marker, center, cloud_normals->points[0], circle_radius, 3, cloud_msg->header);

        // Publish the grasp circle
        grasp_circle_marker_publisher_->publish(grasp_circle_marker);

        // Continue with normal marker array publishing (normals)
        visualization_msgs::msg::MarkerArray marker_array;

        for (size_t i = 0; i < cloud_filtered->points.size(); ++i) {
            if (!pcl::isFinite(cloud_normals->points[i])) {
                continue;
            }

            visualization_msgs::msg::Marker marker;
            marker.header = cloud_msg->header;
            marker.ns = "surface_normals";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.scale.x = 0.01;
            marker.scale.y = 0.02;
            marker.scale.z = 0.02;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            geometry_msgs::msg::Point start, end;
            start.x = cloud_filtered->points[i].x;
            start.y = cloud_filtered->points[i].y;
            start.z = cloud_filtered->points[i].z;

            end.x = start.x + cloud_normals->points[i].normal_x * 0.1;
            end.y = start.y + cloud_normals->points[i].normal_y * 0.1;
            end.z = start.z + cloud_normals->points[i].normal_z * 0.1;

            marker.points.push_back(start);
            marker.points.push_back(end);
            marker_array.markers.push_back(marker);
        }

        RCLCPP_INFO(this->get_logger(), "Publishing marker array with %ld markers.", marker_array.markers.size());

        normal_marker_publisher_->publish(marker_array);
    }

    // Helper function to create a circle marker around the centroid, aligned with the normal direction
    void createNormalAlignedCircleMarker(visualization_msgs::msg::Marker& circle_marker, const geometry_msgs::msg::Point& center,
                                         const pcl::Normal& normal, double radius, int id, const std_msgs::msg::Header& header)
    {
        circle_marker.header = header;
        circle_marker.ns = "grasp_circle";
        circle_marker.id = id;
        circle_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        circle_marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the scale and color of the circle
        circle_marker.scale.x = 0.01;  // Line width
        circle_marker.color.a = 1.0;
        circle_marker.color.r = 0.0;
        circle_marker.color.g = 0.0;
        circle_marker.color.b = 1.0;  // Blue circle

        // Define the number of points on the circle (e.g., 36 points for a smooth circle)
        int num_points = 36;

        // Find two orthogonal vectors to the normal
        Eigen::Vector3f normal_vec(normal.normal_x, normal.normal_y, normal.normal_z);
        Eigen::Vector3f arbitrary_vec(1.0, 0.0, 0.0);  // Arbitrary vector
        if (normal_vec.dot(arbitrary_vec) > 0.99) {
            arbitrary_vec = Eigen::Vector3f(0.0, 1.0, 0.0);
        }
        Eigen::Vector3f tangent_vec1 = normal_vec.cross(arbitrary_vec).normalized();
        Eigen::Vector3f tangent_vec2 = normal_vec.cross(tangent_vec1).normalized();

        // This loop will create a circle perpendicular to the normal
        for (int i = 0; i <= num_points; ++i) {
            geometry_msgs::msg::Point point;
            double angle = 2 * M_PI * i / num_points;  // Compute the angle in radians

            // Compute the point on the circle in the plane defined by the normal
            Eigen::Vector3f point_on_circle = tangent_vec1 * (radius * cos(angle)) + tangent_vec2 * (radius * sin(angle));

            point.x = center.x + point_on_circle.x();
            point.y = center.y + point_on_circle.y();
            point.z = center.z + point_on_circle.z();

            circle_marker.points.push_back(point);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr normal_marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr centroid_marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr grasp_line_marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr grasp_circle_marker_publisher_;

    // Variables to store the direction vector for the grasp line
    geometry_msgs::msg::Point direction_vector_;
    bool points_selected_;  // Flag to indicate if points have been selected
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SurfaceNormNode>();
    RCLCPP_INFO(node->get_logger(), "Starting node...");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
