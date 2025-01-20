from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments for all parameters (dynamic)
        DeclareLaunchArgument('cluster_topic', default_value='/detected_cluster', description='Cluster topic name'),
        DeclareLaunchArgument('pointcloud_topic', default_value='/realsense/points', description='Pointcloud topic name'),
        DeclareLaunchArgument('coord_topic', default_value='/target_2d_coords', description='2D centroid target coordinates topic'),
        DeclareLaunchArgument('camera_info_topic', default_value='/realsense/camera_info', description='Camera info topic'),
        DeclareLaunchArgument('visualize', default_value='false', description='Enable visualization in RViz of filters'),
        DeclareLaunchArgument('crop_radius', default_value='0.2', description='Crop box radius'),
        DeclareLaunchArgument('sor_mean_k', default_value='50', description='SOR mean K'),
        DeclareLaunchArgument('sor_stddev_mul_thresh', default_value='1.0', description='SOR stddev multiplier threshold'),
        DeclareLaunchArgument('voxel_leaf_size', default_value='0.01', description='Voxel leaf size'),
        DeclareLaunchArgument('ransac_max_iterations', default_value='1000', description='RANSAC max iterations'),
        DeclareLaunchArgument('ransac_distance_threshold', default_value='0.01', description='RANSAC distance threshold'),
        DeclareLaunchArgument('header_frame', default_value='camera_link', description='Frame of reference for the header'),
        DeclareLaunchArgument('cluster_tolerance', default_value='0.02', description='Cluster tolerance'),
        DeclareLaunchArgument('min_cluster_size', default_value='100', description='Minimum cluster size'),
        DeclareLaunchArgument('max_cluster_size', default_value='25000', description='Maximum cluster size'),
        DeclareLaunchArgument('target_point_tolerance', default_value='0.02', description='Target point tolerance'),
        DeclareLaunchArgument('normal_search_radius', default_value='0.03', description='Normal search radius'),
        DeclareLaunchArgument('robust_search', default_value='false', description='Enable robust search'),
        DeclareLaunchArgument('min_search_threshold', default_value='0.02', description='Minimum search threshold'),
        DeclareLaunchArgument('max_search_threshold', default_value='0.1', description='Maximum search threshold'),
        Node(
            package='grasp_vision_cpp',
            executable='extract_cluster',
            name='extract_cluster',
            parameters=[
                {
                    'cluster_topic': LaunchConfiguration('cluster_topic'),
                    'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
                    'coord_topic': LaunchConfiguration('coord_topic'),
                    'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                    'visualize': LaunchConfiguration('visualize'),
                    'crop_radius': LaunchConfiguration('crop_radius'),
                    'sor_mean_k': LaunchConfiguration('sor_mean_k'),
                    'sor_stddev_mul_thresh': LaunchConfiguration('sor_stddev_mul_thresh'),
                    'voxel_leaf_size': LaunchConfiguration('voxel_leaf_size'),
                    'ransac_max_iterations': LaunchConfiguration('ransac_max_iterations'),
                    'ransac_distance_threshold': LaunchConfiguration('ransac_distance_threshold'),
                    'header_frame': LaunchConfiguration('header_frame'),
                    'cluster_tolerance': LaunchConfiguration('cluster_tolerance'),
                    'min_cluster_size': LaunchConfiguration('min_cluster_size'),
                    'max_cluster_size': LaunchConfiguration('max_cluster_size'),
                    'target_point_tolerance': LaunchConfiguration('target_point_tolerance'),
                }
            ]
        ),
        Node(
            package='grasp_vision_cpp',
            executable='optimal_grasp',
            name='optimal_grasp',
            parameters=[
                {
                    'cluster_topic': LaunchConfiguration('cluster_topic'),
                    'normal_search_radius': LaunchConfiguration('normal_search_radius'),
                    'robust_search': LaunchConfiguration('robust_search'),
                    'min_search_threshold': LaunchConfiguration('min_search_threshold'),
                    'max_search_threshold': LaunchConfiguration('max_search_threshold')
                }
            ]
        )
    ])
