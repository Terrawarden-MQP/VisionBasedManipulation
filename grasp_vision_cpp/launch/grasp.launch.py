from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments for all parameters (dynamic)
        DeclareLaunchArgument('log_level', default_value='INFO', description='Log verbosity level'),
        DeclareLaunchArgument('cluster_topic', default_value='/detected_cluster', description='Cluster topic name'),
        DeclareLaunchArgument('pointcloud_topic', default_value='/camera/camera/depth/color/points', description='Pointcloud topic name'),
        DeclareLaunchArgument('coord_topic', default_value='/target_2d_coords', description='2D centroid target coordinates topic'),
        DeclareLaunchArgument('camera_info_topic_depth', default_value='/camera/camera/depth/camera_info', description='Camera depth image info topic'),
        DeclareLaunchArgument('camera_info_topic_color', default_value='/camera/camera/color/camera_info', description='Camera color image info topic'),
        DeclareLaunchArgument('camera_depth_topic', default_value='/camera/camera/depth/image_rect_raw', description='Camera depth image topic'),
        DeclareLaunchArgument('visualize', default_value='false', description='Enable visualization in RViz of filters and normals'),
        DeclareLaunchArgument('crop_radius', default_value='0.2', description='Crop box radius'),
        DeclareLaunchArgument('sor_mean_k', default_value='50', description='SOR mean K'),
        DeclareLaunchArgument('sor_stddev_mul_thresh', default_value='1.0', description='SOR stddev multiplier threshold'),
        DeclareLaunchArgument('voxel_leaf_size', default_value='0.01', description='Voxel leaf size'),
        DeclareLaunchArgument('ransac_max_iterations', default_value='1000', description='RANSAC max iterations'),
        DeclareLaunchArgument('ransac_distance_threshold', default_value='0.005', description='RANSAC distance threshold'),
        DeclareLaunchArgument('header_frame', default_value='camera_depth_optical_frame', description='Frame of reference for the header'),
        DeclareLaunchArgument('cluster_tolerance', default_value='0.02', description='Cluster tolerance'),
        DeclareLaunchArgument('min_cluster_size', default_value='100', description='Minimum cluster size'),
        DeclareLaunchArgument('max_cluster_size', default_value='25000', description='Maximum cluster size'),
        DeclareLaunchArgument('target_point_tolerance', default_value='0.02', description='Target point tolerance'),
        DeclareLaunchArgument('curvature', default_value='0.01', description='Curvature value for edge detection'),
        DeclareLaunchArgument('normal_search_radius', default_value='0.03', description='Normal search radius'),
        DeclareLaunchArgument('min_search_threshold', default_value='0.035', description='Minimum search threshold'),
        DeclareLaunchArgument('max_search_threshold', default_value='0.08', description='Maximum search threshold'),
        DeclareLaunchArgument('select_stability_metric', default_value='1', description='1: maximum minimum svd, 2: maximum volume ellipsoid in wrench space,\
                               3: isotropy index, 4: maximum minimum svd with abs for numeric stability, 5: weighing (1) and (2) equally'),
        DeclareLaunchArgument('variance_neighbors', default_value='4', description='Grasp uncertainty variance neighbors to search'),
        DeclareLaunchArgument('variance_threshold', default_value='0.2', description='Grasp uncertainty variance threshold'),
        DeclareLaunchArgument('pos_topic', default_value='grasp_pose', description='Grasp pose topic'),
        Node(
            package='grasp_vision_cpp',
            executable='extract_cluster',
            name='extract_cluster',
            parameters=[
                {
                    'cluster_topic': LaunchConfiguration('cluster_topic'),
                    'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
                    'coord_topic': LaunchConfiguration('coord_topic'),
                    'camera_info_topic_depth': LaunchConfiguration('camera_info_topic_depth'),
                    'camera_info_topic_color': LaunchConfiguration('camera_info_topic_color'),
                    'camera_depth_topic': LaunchConfiguration('camera_depth_topic'),
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
                    'curvature': LaunchConfiguration('curvature'),
                    'normal_search_radius': LaunchConfiguration('normal_search_radius'),
                }
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),
        Node(
            package='grasp_vision_cpp',
            executable='optimal_grasp',
            name='optimal_grasp',
            parameters=[
                {
                    'cluster_topic': LaunchConfiguration('cluster_topic'),
                    'normal_search_radius': LaunchConfiguration('normal_search_radius'),
                    'min_search_threshold': LaunchConfiguration('min_search_threshold'),
                    'max_search_threshold': LaunchConfiguration('max_search_threshold'),
                    'visualize': LaunchConfiguration('visualize'),
                    'select_stability_metric': LaunchConfiguration('select_stability_metric'),
                    'variance_neighbors': LaunchConfiguration('variance_neighbors'),
                    'variance_threshold': LaunchConfiguration('variance_threshold'),
                    'pos_topic': LaunchConfiguration('pos_topic'),
                }
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        )
    ])
