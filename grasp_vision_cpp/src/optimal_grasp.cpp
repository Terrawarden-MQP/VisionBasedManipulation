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
#include <sstream>
#include <chrono>

class OptimalGraspNode : public rclcpp::Node
{
public:
    OptimalGraspNode() : Node("optimal_grasp")
    {
        // ROS parameters
        this->declare_parameter<std::string>("cluster_topic", "/detected_cluster");
        this->declare_parameter<double>("normal_search_radius", 0.03);
        this->declare_parameter<bool>("robust_search", false);
        this->declare_parameter<double>("min_search_threshold", 0.02);
        this->declare_parameter<double>("max_search_threshold", 0.1);
        this->declare_parameter<bool>("visualize", false);
        this->declare_parameter<int>("select_stability_metric", 1);

        // Retrieve ROS parameters
        cluster_topic = this->get_parameter("cluster_topic").as_string();
        normal_search_radius = this->get_parameter("normal_search_radius").as_double();
        robust_search = this->get_parameter("robust_search").as_bool();
        min_search_threshold = this->get_parameter("min_search_threshold").as_double();
        max_search_threshold = this->get_parameter("max_search_threshold").as_double();
        VISUALIZE = this->get_parameter("visualize").as_bool();
        select_stability_metric = this->get_parameter("select_stability_metric").as_int();

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cluster_topic, 10, // from extract_cluster
            std::bind(&OptimalGraspNode::graspPlanningCallback, this, std::placeholders::_1));

        normal_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/normal_markers", 10);
        grasp_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/grasp_markers", 10);
    }

private:
    std::string cluster_topic;
    double normal_search_radius, min_search_threshold, max_search_threshold; 
    bool robust_search, VISUALIZE;
    int select_stability_metric;

    void graspPlanningCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received point cloud!");

        // Timer
        auto t1 = std::chrono::high_resolution_clock::now();

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
        ne.setRadiusSearch(normal_search_radius);
        ne.compute(*cloud_normals);
        if(VISUALIZE){
            publishNormalMarkers(cloud, cloud_normals,cloud_msg->header);
        }

        if (cloud_normals->points.empty()) {
            RCLCPP_WARN(this->get_logger(), "No normals were computed!");
            return;
        }

        // Compute centroid
        Eigen::Vector4f center;
        pcl::compute3DCentroid(*cloud, center);
        Eigen::Vector3d centroid;
        centroid[0] = center[0];
        centroid[1] = center[1];
        centroid[2] = center[3];

        // Find optimal grasp points
        geometry_msgs::msg::Point grasp_point1, grasp_point2;
        // bool grasp_algorithm = !robust_search ? findOptimalGraspPoints(cloud, cloud_normals, grasp_point1, grasp_point2) : 
        //     findOptimalGraspPointsWithUncertainty(cloud,cloud_normals,grasp_point1,grasp_point2);
        bool grasp_algorithm = findOptimalGraspPointsQuality(cloud, cloud_normals, grasp_point1, grasp_point2, centroid);
        if (!grasp_algorithm) {
            RCLCPP_WARN(this->get_logger(), "Failed to find optimal grasp points!");
            return;
        }

        // Publish grasp markers for visualization
        publishGraspMarkers(grasp_point1, grasp_point2, cloud_msg->header);

        // Timer
        auto t2 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double,std::milli> elapsed = t2-t1;
        RCLCPP_INFO(this->get_logger(),"Grasp time elapsed from receiving cloud: %f",elapsed);

    }

    // Based on RBE 595 VBM F24 HW 3
    
    // Helper function to calculate skew-symmetric matrix
    Eigen::Matrix3d skew(const Eigen::Vector3d& vec) {
        Eigen::Matrix3d skewMat;
        skewMat <<  0,        -vec.z(),  vec.y(),
                    vec.z(),   0,       -vec.x(),
                -vec.y(),   vec.x(),  0;
        return skewMat;
    }

    // Helper function to compute rotation matrix from surface normal
    Eigen::Matrix3d rotationMatrix(const Eigen::Vector3d& normal) {
        Eigen::Vector3d z = normal.normalized(); // Surface normal as z-axis
        Eigen::Vector3d x, y;

        // Orthogonal x-axis (avoid collinearity)
        if (std::abs(z.x()) > std::abs(z.y()))
            x = Eigen::Vector3d(-z.z(), 0, z.x()).normalized();
        else
            x = Eigen::Vector3d(0, z.z(), -z.y()).normalized();

        y = z.cross(x); // Cross product y-axis

        Eigen::Matrix3d R; // Rotation matrix
        R.col(0) = x;
        R.col(1) = y;
        R.col(2) = z;
        return R;
    }

    // Helper function to calculate the grasp P matrix
    Eigen::MatrixXd calcGraspPMatrix(const Eigen::MatrixXd& contact_points, const Eigen::Vector3d& centroid) {
        int N = contact_points.rows(); // Number of contact points
        Eigen::MatrixXd P = Eigen::MatrixXd::Zero(6 * N, 6); // Initialize P matrix

        for (int i = 0; i < N; ++i) {
            Eigen::Vector3d contact_point_i = contact_points.row(i); // Contact point
            Eigen::MatrixXd Pi(6, 6);
            Pi.setZero();

            Pi.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
            Pi.block<3, 3>(0, 3) = skew(contact_point_i - centroid).transpose();
            Pi.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();

            P.block<6, 6>(i * 6, 0) = Pi;
        }

        return P;
    }

    // Helper function to calculate the grasp matrix for N contact points
    Eigen::MatrixXd calcGraspMatrix(const Eigen::MatrixXd& contact_points, const Eigen::MatrixXd& normals, const Eigen::Vector3d& centroid) {
        int N = contact_points.rows(); // Number of contact points
        Eigen::MatrixXd G = Eigen::MatrixXd::Zero(6, 6 * N); // Initialize G matrix
        Eigen::MatrixXd P = calcGraspPMatrix(contact_points, centroid);          // Compute P matrix
        for (int i = 0; i < N; ++i) {
            Eigen::Vector3d normal = normals.row(i);
            // Eigen::Vector3d normal = normals.segment<3>(i * 3); // Extract surface normal for contact point i
            Eigen::Matrix3d Ri = rotationMatrix(normal); // Compute rotation matrix from surface normal

            Eigen::MatrixXd Ri_bar(6, 6); // Block diagonal rotation matrix
            Ri_bar.setZero();
            Ri_bar.block<3, 3>(0, 0) = Ri; // Top-left 3x3 block
            Ri_bar.block<3, 3>(3, 3) = Ri; // Bottom-right 3x3 block

            Eigen::MatrixXd Gi_T = Ri_bar * P.block<6, 6>(i * 6, 0); // Compute Gi transpose
            G.block<6, 6>(0, i * 6) = Gi_T.transpose();             // Append Gi^T to G
        }

        return G;
    }

    bool findOptimalGraspPointsQuality(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        const pcl::PointCloud<pcl::Normal>::Ptr &normals,
        geometry_msgs::msg::Point &point1,
        geometry_msgs::msg::Point &point2,
        Eigen::Vector3d &centroid)
    {
        double max_quality = -std::numeric_limits<double>::infinity();
        geometry_msgs::msg::Point best_point1, best_point2;

        // Iterate through all point pairs to calculate grasp matrices
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            for (size_t j = i + 1; j < cloud->points.size(); ++j) {
                // Extract points and their normals
                Eigen::Vector3d p1(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
                Eigen::Vector3d p2(cloud->points[j].x, cloud->points[j].y, cloud->points[j].z);
                Eigen::Vector3d n1(normals->points[i].normal_x, normals->points[i].normal_y, normals->points[i].normal_z);
                Eigen::Vector3d n2(normals->points[j].normal_x, normals->points[j].normal_y, normals->points[j].normal_z);

                // Skip if the points are too close or too far
                double distance = (p2 - p1).norm();
                if (distance < min_search_threshold || distance > max_search_threshold) {
                    continue;
                }

                // Build grasp matrix G for the two contact points
                Eigen::MatrixXd contact_points(2,3);
                Eigen::MatrixXd normals(2,3);
                contact_points.row(0) = p1;
                contact_points.row(1) = p2;
                normals.row(0) = n1;
                normals.row(1) = n2;

                Eigen::MatrixXd G = calcGraspMatrix(contact_points, normals, centroid);

                // Evaluate grasp quality using the singular value decomposition (SVD)
                Eigen::JacobiSVD<Eigen::MatrixXd> svd(G, Eigen::ComputeThinU | Eigen::ComputeThinV);

                // Select stability metric
                double singular_value;
                switch(select_stability_metric){
                    case 1:
                        // maximum minimum singular value of grasp matrix
                        singular_value = svd.singularValues().minCoeff();
                        break;
                    case 2:
                        // maximum volume of the ellipsoid in the wrench space 
                        singular_value = svd.singularValues().prod();
                        break;
                    case 3:
                        // grasp isotropy index (1 = isotropic / optimal, 0 = singular configuration)
                        singular_value = svd.singularValues().minCoeff() / svd.singularValues().maxCoeff();
                        break;
                    default:
                        RCLCPP_WARN(this->get_logger(),"Stability metric not set");
                        singular_value = 0;
                }
                
                // Update best grasp if this one is better
                RCLCPP_DEBUG(this->get_logger(), "SVD: %f",singular_value);
                RCLCPP_DEBUG(this->get_logger(), "G:\n%s", (std::ostringstream() << G).str().c_str());
                if (singular_value > max_quality) {
                    max_quality = singular_value;
                    best_point1.x = p1[0];
                    best_point1.y = p1[1];
                    best_point1.z = p1[2];
                    best_point2.x = p2[0];
                    best_point2.y = p2[1];
                    best_point2.z = p2[2];
                }
            }
        }

        // Check if a valid grasp was found
        if (max_quality == -std::numeric_limits<double>::infinity()) {
            RCLCPP_WARN(this->get_logger(), "Failed to find a valid grasp matrix!");
            return false;
        }

        // Assign the best points to the output parameters
        point1 = best_point1;
        point2 = best_point2;

        RCLCPP_INFO(this->get_logger(), "Optimal Point 1: (%f, %f, %f)", best_point1.x, best_point1.y, best_point1.z);
        RCLCPP_INFO(this->get_logger(), "Optimal Point 2: (%f, %f, %f)", best_point2.x, best_point2.y, best_point2.z);
        RCLCPP_INFO(this->get_logger(), "Max Grasp Quality (Min Singular Value): %f", max_quality);

        return true;
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
                if (distance < min_search_threshold || distance > max_search_threshold) { // Adjust thresholds for gripper size
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
                if (distance < min_search_threshold || distance > max_search_threshold) { // Adjust thresholds for gripper size
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
