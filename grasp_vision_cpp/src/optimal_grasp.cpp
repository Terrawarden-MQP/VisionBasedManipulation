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
#include <random>
// #include <cmath>
#include <sstream>
#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>

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
        this->declare_parameter<double>("curvature", 0.01);
        this->declare_parameter<std::string>("header_frame","camera_link");
        this->declare_parameter<int>("variance_neighbors", 10);
        this->declare_parameter<double>("variance_threshold", 0.2);
        this->declare_parameter<std::string>("pos_topic","grasp_pose");

        // Retrieve ROS parameters
        cluster_topic = this->get_parameter("cluster_topic").as_string();
        normal_search_radius = this->get_parameter("normal_search_radius").as_double();
        robust_search = this->get_parameter("robust_search").as_bool();
        min_search_threshold = this->get_parameter("min_search_threshold").as_double();
        max_search_threshold = this->get_parameter("max_search_threshold").as_double();
        VISUALIZE = this->get_parameter("visualize").as_bool();
        select_stability_metric = this->get_parameter("select_stability_metric").as_int();
        curvature = this->get_parameter("curvature").as_double();
        header_frame = this->get_parameter("header_frame").as_string();
        variance_neighbors = this->get_parameter("variance_neighbors").as_int();
        variance_threshold = this->get_parameter("variance_threshold").as_double();
        pos_topic = this->get_parameter("pos_topic").as_string();

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cluster_topic, 10, // from extract_cluster
            std::bind(&OptimalGraspNode::graspPlanningCallback, this, std::placeholders::_1));
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pos_topic, 10);

        if(VISUALIZE){
            normal_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/normal_markers", 10);
            grasp_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/grasp_markers", 10);
            curvature_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/visualize/curvature", 10);
        }
    }

private:
    std::string cluster_topic, header_frame,  pos_topic;
    double normal_search_radius, min_search_threshold, max_search_threshold, curvature, variance_threshold; 
    bool robust_search, VISUALIZE;
    int select_stability_metric, variance_neighbors;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr normal_marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr grasp_marker_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr curvature_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;

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
        // if(VISUALIZE){
        //     publishNormalMarkers(cloud, cloud_normals,cloud_msg->header);
        // }

        if (cloud_normals->points.empty()) {
            RCLCPP_WARN(this->get_logger(), "No normals were computed!");
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_processed(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_processed(new pcl::PointCloud<pcl::Normal>());

        // Curvature edge detection (select only stable points)
        for(int i = 0; i < cloud->size(); i++){
            if(cloud_normals->points[i].curvature < curvature){
                cluster_processed->push_back(cloud->points[i]);
                cloud_normals_processed->push_back(cloud_normals->points[i]);
            }
        }        
        RCLCPP_INFO(this->get_logger(), "Stable cloud has %lu points", cluster_processed->points.size());
        
        if(VISUALIZE){
            publishNormalMarkers(cluster_processed, cloud_normals_processed,cloud_msg->header);
            sensor_msgs::msg::PointCloud2 cloud_msg_;
            pcl::toROSMsg(*cluster_processed,cloud_msg_);
            cloud_msg_.header.frame_id = header_frame; 
            cloud_msg_.header.stamp = this->now();
            curvature_publisher_->publish(cloud_msg_);
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
        bool grasp_algorithm = findOptimalGraspPointsQuality(cluster_processed, cloud_normals_processed, grasp_point1, grasp_point2, centroid);
        if (!grasp_algorithm) {
            RCLCPP_WARN(this->get_logger(), "Failed to find optimal grasp points!");
            return;
        }

        // Publish grasp markers for visualization
        if(VISUALIZE){
            publishGraspMarkers(grasp_point1, grasp_point2, cloud_msg->header);
        }

        // Publish grasp position and orientation
        publishGraspPose(grasp_point1, grasp_point2, cloud_msg->header);

        // Timer
        auto t2 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double,std::milli> elapsed = t2-t1;
        RCLCPP_INFO(this->get_logger(),"Grasp time elapsed from receiving cloud: %f",elapsed.count());

    }

    void publishGraspPose(
        const geometry_msgs::msg::Point& grasp_point1, 
        const geometry_msgs::msg::Point& grasp_point2, 
        const std_msgs::msg::Header& header) 
    {
        // Compute the grasp position (midpoint)
        Eigen::Vector3d p1(grasp_point1.x, grasp_point1.y, grasp_point1.z);
        Eigen::Vector3d p2(grasp_point2.x, grasp_point2.y, grasp_point2.z);
        Eigen::Vector3d grasp_position = (p1 + p2) / 2.0;

        // Compute the grasp approach direction (z-axis)
        Eigen::Vector3d grasp_z = (p2 - p1).normalized();

        // Define an arbitrary x-axis perpendicular to grasp_z
        Eigen::Vector3d grasp_x;
        if (std::abs(grasp_z.x()) > std::abs(grasp_z.y())) {
            grasp_x = Eigen::Vector3d(-grasp_z.z(), 0, grasp_z.x()).normalized();
        } else {
            grasp_x = Eigen::Vector3d(0, grasp_z.z(), -grasp_z.y()).normalized();
        }

        // Compute the y-axis (perpendicular to x and z)
        Eigen::Vector3d grasp_y = grasp_z.cross(grasp_x).normalized();

        // Construct the rotation matrix
        Eigen::Matrix3d R;
        R.col(0) = grasp_x;
        R.col(1) = grasp_y;
        R.col(2) = grasp_z;

        // Convert rotation matrix to quaternion
        Eigen::Quaterniond quat(R);

        // Fill PoseStamped message
        geometry_msgs::msg::PoseStamped grasp_pose_msg;
        grasp_pose_msg.header = header;
        grasp_pose_msg.pose.position.x = grasp_position.x();
        grasp_pose_msg.pose.position.y = grasp_position.y();
        grasp_pose_msg.pose.position.z = grasp_position.z();
        grasp_pose_msg.pose.orientation.x = quat.x();
        grasp_pose_msg.pose.orientation.y = quat.y();
        grasp_pose_msg.pose.orientation.z = quat.z();
        grasp_pose_msg.pose.orientation.w = quat.w();

        // Publish the grasp pose
        pose_publisher_->publish(grasp_pose_msg);
    }

    // ============================= GRASP MATRIX =============================
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

        // Choose an arbitrary "up" vector that is not collinear with z (Gram-Schmidt orthonormalization)
        Eigen::Vector3d up = (std::abs(z.z()) > 0.9) ? Eigen::Vector3d(1, 0, 0) : Eigen::Vector3d(0, 0, 1);


        // Orthogonal x-axis (avoid collinearity)
        // if (std::abs(z.x()) > std::abs(z.y()))
        //     x = Eigen::Vector3d(-z.z(), 0, z.x()).normalized();
        // else
        //     x = Eigen::Vector3d(0, z.z(), -z.y()).normalized();
        x = up.cross(z).normalized();

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

                // Skip if normal variance too great
                double variance1 = computeNormalVariance(cloud, normals, i, variance_neighbors);
                double variance2 = computeNormalVariance(cloud, normals, j, variance_neighbors);
                if(variance1 > variance_threshold || variance2 > variance_threshold){
                    RCLCPP_DEBUG(this->get_logger(), "Normal variance at index %ld: %f", i,variance1);
                    RCLCPP_DEBUG(this->get_logger(), "Normal variance at index %ld: %f", j,variance2);
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
                    case 4: 
                        // testing numeric stability in svd selection
                        singular_value = svd.singularValues().array().abs().minCoeff();
                        break;
                    case 5:
                        // testing robust grasping by weighing multiple metrics
                        singular_value = 0.5 * svd.singularValues().minCoeff() + 0.5 * svd.singularValues().prod();
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

    // Helper function to handle uncertainty in grasp selection (robust grasp)
    // Weights grasp selection based on normal deviation (variance) over neighboring points
    double computeNormalVariance(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,const pcl::PointCloud<pcl::Normal>::Ptr &normals, int idx, int k_neighbors){
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        tree->setInputCloud(cloud);
        std::vector<int> pointIdxNKNSearch(k_neighbors);
        std::vector<float> pointNKNSquaredDistance(k_neighbors);
        double variance = 0.0;
        if (tree->nearestKSearch(cloud->points[idx], k_neighbors, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            Eigen::Vector3d avg_normal(0, 0, 0);
            for (int i : pointIdxNKNSearch) {
                avg_normal += Eigen::Vector3d(normals->points[i].normal_x, normals->points[i].normal_y, normals->points[i].normal_z);
            }
            avg_normal /= k_neighbors;

            for (int i : pointIdxNKNSearch) {
                Eigen::Vector3d diff = Eigen::Vector3d(normals->points[i].normal_x, normals->points[i].normal_y, normals->points[i].normal_z) - avg_normal;
                variance += diff.squaredNorm();
            }
        }
        else{
            RCLCPP_WARN(this->get_logger(),"AAAAAAAAAAAAAAAAAAAAa");
        }
        return variance / k_neighbors;
    }

    // ============================= VISUALIZE IN RVIZ =============================

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
        RCLCPP_DEBUG(this->get_logger(), "Published grasp markers.");
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
        RCLCPP_DEBUG(this->get_logger(), "Publishing %ld normal markers.", marker_array.markers.size());
        normal_marker_publisher_->publish(marker_array);
    }
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
