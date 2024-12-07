#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/search/kdtree.h>
// #include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
// #include <random>
// #include <cmath>

class OptimalGraspNode : public rclcpp::Node
{
public:
    OptimalGraspNode() : Node("optimal_grasp")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/detected_cluster", 10, // from extract_cluster
            std::bind(&OptimalGraspNode::graspPlanningCallback, this, std::placeholders::_1));

        normal_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/normal_markers", 10);
        grasp_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/grasp_markers", 10);
    }

private:
    void graspPlanningCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received point cloud!");

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*cloud_msg, *cloud);

        // Remove NaN points
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

        if (cloud->points.empty()) {
            RCLCPP_WARN(this->get_logger(), "Point cloud is empty after removing NaNs!");
            return;
        }

        // Compute surface normals
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(cloud);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setSearchMethod(tree);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());
        ne.setRadiusSearch(0.03); // TODO tune? Add vars to top / ROS args lmao
        ne.compute(*cloud_normals);
        publishNormalMarkers(cloud, cloud_normals,cloud_msg->header);

        if (cloud_normals->points.empty()) {
            RCLCPP_WARN(this->get_logger(), "No normals were computed!");
            return;
        }

        // Compute centroid
        // Eigen::Vector4f centroid;
        // pcl::compute3DCentroid(*cloud, centroid);

        // Find optimal grasp points
        geometry_msgs::msg::Point grasp_point1, grasp_point2;
        if (!findOptimalGraspPoints(cloud, cloud_normals, grasp_point1, grasp_point2)) { // findOptimalGraspPoints
            RCLCPP_WARN(this->get_logger(), "Failed to find optimal grasp points!");
            return;
        }

        // Publish grasp markers for visualization
        publishGraspMarkers(grasp_point1, grasp_point2, cloud_msg->header);
    }

    bool findOptimalGraspPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                const pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                // const Eigen::Vector4f &centroid,
                                geometry_msgs::msg::Point &point1,
                                geometry_msgs::msg::Point &point2)
    {
        double max_quality = std::numeric_limits<double>::infinity();
        geometry_msgs::msg::Point best_point1, best_point2;

        // Iterate through all point pairs to evaluate grasp quality
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            for (size_t j = i + 1; j < cloud->points.size(); ++j) {
                // Compute the vector between the two points
                Eigen::Vector3f p1(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
                Eigen::Vector3f p2(cloud->points[j].x, cloud->points[j].y, cloud->points[j].z);
                Eigen::Vector3f grasp_vector = p2 - p1;

                // Skip if the points are too close or too far
                double distance = grasp_vector.norm();
                if (distance < 0.02 || distance > 0.1) { // Adjust thresholds for gripper size
                    continue;
                }

                // Compute quality based on alignment with normals
                Eigen::Vector3f normal1(normals->points[i].normal_x, normals->points[i].normal_y, normals->points[i].normal_z);
                Eigen::Vector3f normal2(normals->points[j].normal_x, normals->points[j].normal_y, normals->points[j].normal_z);
                // double alignment = std::abs(grasp_vector.dot(normal1)) + std::abs(grasp_vector.dot(normal2));
                double alignment = normal1.dot(normal2)/(normal1.norm()*normal2.norm()); // -1 = anti-||
                // TODO calculate alignment based on grasp quality metrics from that paper  / HW

                // TODO: Add reachability (disntance + orientation), gripper width, and collision checking

                if (alignment < max_quality) { // goal is -1
                    max_quality = alignment;
                    best_point1.x = p1[0];
                    best_point1.y = p1[1];
                    best_point1.z = p1[2];
                    best_point2.x = p2[0];
                    best_point2.y = p2[1];
                    best_point2.z = p2[2];
                }
            }
        }

        if (max_quality == -std::numeric_limits<double>::infinity()) {
            return false; // No valid grasp found
        }

        point1 = best_point1;
        point2 = best_point2;
        RCLCPP_INFO(this->get_logger(), "Optimal Point 1: (%f,%f)",best_point1.x,best_point1.y);
        RCLCPP_INFO(this->get_logger(), "Optimal Point 2: (%f,%f)",best_point2.x,best_point2.y);
        return true;
    }

    bool findOptimalGraspPointsWithUncertainty(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        const pcl::PointCloud<pcl::Normal>::Ptr &normals,
        // const Eigen::Vector4f &centroid,
        geometry_msgs::msg::Point &point1,
        geometry_msgs::msg::Point &point2)
    {
        double max_region_score = -std::numeric_limits<double>::infinity();
        geometry_msgs::msg::Point best_point1, best_point2;

        // KDTree for neighborhood search
        pcl::search::KdTree<pcl::PointXYZ> kd_tree;
        kd_tree.setInputCloud(cloud);

        // Iterate through all point pairs
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            for (size_t j = i + 1; j < cloud->points.size(); ++j) {
                // Compute the vector between the two points
                Eigen::Vector3f p1(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
                Eigen::Vector3f p2(cloud->points[j].x, cloud->points[j].y, cloud->points[j].z);
                Eigen::Vector3f grasp_vector = p2 - p1;

                // Skip if the points are too close or too far
                double distance = grasp_vector.norm();
                if (distance < 0.02 || distance > 0.1) { // Adjust thresholds for gripper size
                    continue;
                }

                // Compute quality based on alignment with normals
                Eigen::Vector3f normal1(normals->points[i].normal_x, normals->points[i].normal_y, normals->points[i].normal_z);
                Eigen::Vector3f normal2(normals->points[j].normal_x, normals->points[j].normal_y, normals->points[j].normal_z);
                // double alignment = std::abs(grasp_vector.dot(normal1)) + std::abs(grasp_vector.dot(normal2)); // TODO fix or remove

                // Evaluate the grasp's region quality
                double region_score = computeRegionScore(kd_tree, cloud, normals, cloud->points[i], cloud->points[j]);

                // TODO: Add reachability, gripper width, and collision checking

                if (region_score > max_region_score) {
                    max_region_score = region_score;
                    best_point1.x = p1[0];
                    best_point1.y = p1[1];
                    best_point1.z = p1[2];
                    best_point2.x = p2[0];
                    best_point2.y = p2[1];
                    best_point2.z = p2[2];
                }
            }
        }

        if (max_region_score == -std::numeric_limits<double>::infinity()) {
            return false; // No valid grasp found
        }

        point1 = best_point1;
        point2 = best_point2;
        RCLCPP_INFO(this->get_logger(), "Optimal Point 1: (%f,%f)",best_point1.x,best_point1.y);
        RCLCPP_INFO(this->get_logger(), "Optimal Point 2: (%f,%f)",best_point2.x,best_point2.y);
        return true;
    }

    double computeRegionScore(
        const pcl::search::KdTree<pcl::PointXYZ> &kd_tree,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        const pcl::PointCloud<pcl::Normal>::Ptr &normals,
        const pcl::PointXYZ &point1,
        const pcl::PointXYZ &point2)
    {
        // Radius for neighborhood search
        double region_radius = 0.03; // Adjust as needed
        std::vector<int> neighbors1, neighbors2;
        std::vector<float> distances1, distances2;

        // Search neighbors for point1
        kd_tree.radiusSearch(point1, region_radius, neighbors1, distances1);

        // Search neighbors for point2
        kd_tree.radiusSearch(point2, region_radius, neighbors2, distances2);

        // Compute the combined region score
        double total_score = 0.0;
        for (int idx1 : neighbors1) {
            for (int idx2 : neighbors2) {
                Eigen::Vector3f p1(cloud->points[idx1].x, cloud->points[idx1].y, cloud->points[idx1].z);
                Eigen::Vector3f p2(cloud->points[idx2].x, cloud->points[idx2].y, cloud->points[idx2].z);
                Eigen::Vector3f grasp_vector = p2 - p1;

                Eigen::Vector3f normal1(normals->points[idx1].normal_x, normals->points[idx1].normal_y, normals->points[idx1].normal_z);
                Eigen::Vector3f normal2(normals->points[idx2].normal_x, normals->points[idx2].normal_y, normals->points[idx2].normal_z);

                double alignment = std::abs(grasp_vector.dot(normal1)) + std::abs(grasp_vector.dot(normal2));
                total_score += alignment;
            }
        }

        return total_score;
    }

    void publishGraspMarkers(const geometry_msgs::msg::Point &point1,
                             const geometry_msgs::msg::Point &point2,
                             const std_msgs::msg::Header &header)
    {
        visualization_msgs::msg::Marker grasp_line_marker;
        grasp_line_marker.header = header;
        grasp_line_marker.ns = "optimal_grasp";
        grasp_line_marker.id = 0;
        grasp_line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        grasp_line_marker.action = visualization_msgs::msg::Marker::ADD;

        // Set line properties
        grasp_line_marker.scale.x = 0.01; // Line thickness
        grasp_line_marker.color.a = 1.0;
        grasp_line_marker.color.r = 1.0;
        grasp_line_marker.color.g = 0.0;
        grasp_line_marker.color.b = 0.0;

        // Add points to the line
        grasp_line_marker.points.push_back(point1);
        grasp_line_marker.points.push_back(point2);

        grasp_marker_publisher_->publish(grasp_line_marker);
        RCLCPP_INFO(this->get_logger(), "Published grasp markers.");
    }

    void publishNormalMarkers(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                              const pcl::PointCloud<pcl::Normal>::Ptr &normals,
                              const std_msgs::msg::Header &header)
    {
        visualization_msgs::msg::MarkerArray marker_array;

        for (size_t i = 0; i < cloud->points.size(); ++i) {
            // Skip invalid normals
            if (!pcl::isFinite(normals->points[i])) {
                continue;
            }

            visualization_msgs::msg::Marker marker;
            marker.header = header;
            marker.ns = "surface_normals";
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Set the scale for the arrow (normal visualization)
            marker.scale.x = 0.01; // Shaft diameter
            marker.scale.y = 0.02; // Head diameter
            marker.scale.z = 0.02; // Head length

            // Set the color for the normals
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0; // Green arrows

            // Define the start (origin) and end (tip) points of the arrow
            geometry_msgs::msg::Point start, end;
            start.x = cloud->points[i].x;
            start.y = cloud->points[i].y;
            start.z = cloud->points[i].z;

            end.x = start.x + normals->points[i].normal_x * 0.1; // Scale the normal vector for visualization
            end.y = start.y + normals->points[i].normal_y * 0.1;
            end.z = start.z + normals->points[i].normal_z * 0.1;

            marker.points.push_back(start);
            marker.points.push_back(end);

            // Add the marker to the array
            marker_array.markers.push_back(marker);
        }

        // Publish the marker array
        RCLCPP_INFO(this->get_logger(), "Publishing %ld normal markers.", marker_array.markers.size());
        normal_marker_publisher_->publish(marker_array);
    }


    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr normal_marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr grasp_marker_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OptimalGraspNode>();
    RCLCPP_INFO(node->get_logger(), "Starting Optimal Grasp Node...");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
