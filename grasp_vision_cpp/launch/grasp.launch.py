from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Flags for debugging
        DeclareLaunchArgument('log_level', default_value='INFO', description='Log verbosity level'),
        DeclareLaunchArgument('visualize', default_value='false', description='Enable visualization in RViz of PCL filtered clouds and normals'),
        DeclareLaunchArgument('extract', default_value='false',description='Switch between extracting cluster in 3D to get centroid (true, more accurate but runs slower) and just doing 2D->3D conversion (false)'),

        # Topic configuration
        DeclareLaunchArgument('cluster_topic', default_value='/detected_cluster', description='Cluster topic name'),
        DeclareLaunchArgument('pointcloud_topic', default_value='/camera/camera/depth/color/points', description='Pointcloud topic name'),
        DeclareLaunchArgument('coord_topic', default_value='/joisie_vision/detected_object_centroid', description='2D centroid target coordinates topic'),
        DeclareLaunchArgument('camera_info_topic_depth', default_value='/camera/camera/aligned_depth_to_color/camera_info', description='Camera depth image info topic'),
        DeclareLaunchArgument('camera_info_topic_color', default_value='/camera/camera/color/camera_info', description='Camera color image info topic'),
        DeclareLaunchArgument('camera_depth_topic', default_value='/camera/camera/aligned_depth_to_color/image_raw', description='Camera depth image topic'),
        DeclareLaunchArgument('pos_topic', default_value='grasp_pose', description='Grasp pose topic'),

        # Header frame names
        DeclareLaunchArgument('header_frame', default_value='camera_color_optical_frame', description='Frame of reference for the camera point cloud color'),
        DeclareLaunchArgument('header_frame_drone', default_value='drone_frame', description='Frame of reference for the drone'),
        DeclareLaunchArgument('header_frame_depth', default_value='camera_depth_optical_frame', description='Frame of reference for the camera point cloud depth'),

        # PCL filter arguments for extract cluster node
        DeclareLaunchArgument('crop_radius', default_value='0.2', description='Crop box radius'),
        DeclareLaunchArgument('sor_mean_k', default_value='50', description='SOR mean K'),
        DeclareLaunchArgument('sor_stddev_mul_thresh', default_value='1.0', description='SOR stddev multiplier threshold'),
        DeclareLaunchArgument('voxel_leaf_size', default_value='0.01', description='Voxel leaf size'),
        DeclareLaunchArgument('ransac_max_iterations', default_value='1000', description='RANSAC max iterations'),
        DeclareLaunchArgument('ransac_distance_threshold', default_value='0.005', description='RANSAC distance threshold'),
        DeclareLaunchArgument('cluster_tolerance', default_value='0.02', description='Cluster tolerance in m'),
        DeclareLaunchArgument('min_cluster_size', default_value='100', description='Minimum cluster size in number of points'),
        DeclareLaunchArgument('max_cluster_size', default_value='25000', description='Maximum cluster size in number of points'),
        DeclareLaunchArgument('target_point_tolerance', default_value='0.02', description='Target point tolerance in m'),

        # PCL filter arguments for optimal grasp node
        DeclareLaunchArgument('curvature', default_value='0.01', description='Curvature value for edge detection'),
        DeclareLaunchArgument('normal_search_radius', default_value='0.03', description='Normal search radius in m'),
        DeclareLaunchArgument('min_search_threshold', default_value='0.035', description='Minimum search threshold'),
        DeclareLaunchArgument('max_search_threshold', default_value='0.08', description='Maximum search threshold'),

        # Grasp stability metrics for optimal grasp node
        DeclareLaunchArgument('select_stability_metric', default_value='1', description='1: maximum minimum svd, 2: maximum volume ellipsoid in wrench space,\
                               3: isotropy index, 4: maximum minimum svd with abs for numeric stability, 5: weighing (1) and (2) equally'),
        DeclareLaunchArgument('variance_neighbors', default_value='4', description='Grasp uncertainty variance neighbors to search'),
        DeclareLaunchArgument('variance_threshold', default_value='0.2', description='Grasp uncertainty variance threshold'),
        
        Node(
            package='grasp_vision_cpp',
            executable='extract_cluster',
            name='extract_cluster',
            parameters=[{
                'cluster_topic': LaunchConfiguration('cluster_topic'),
                'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
                'coord_topic': LaunchConfiguration('coord_topic'),
                'camera_info_topic_depth': LaunchConfiguration('camera_info_topic_depth'),
                'camera_info_topic_color': LaunchConfiguration('camera_info_topic_color'),
                'camera_depth_topic': LaunchConfiguration('camera_depth_topic'),
                'visualize': LaunchConfiguration('visualize'),
                'extract': LaunchConfiguration('extract'),
                'crop_radius': LaunchConfiguration('crop_radius'),
                'sor_mean_k': LaunchConfiguration('sor_mean_k'),
                'sor_stddev_mul_thresh': LaunchConfiguration('sor_stddev_mul_thresh'),
                'voxel_leaf_size': LaunchConfiguration('voxel_leaf_size'),
                'ransac_max_iterations': LaunchConfiguration('ransac_max_iterations'),
                'ransac_distance_threshold': LaunchConfiguration('ransac_distance_threshold'),
                'header_frame': LaunchConfiguration('header_frame'),
                'header_frame_drone': LaunchConfiguration('header_frame_drone'),
                'cluster_tolerance': LaunchConfiguration('cluster_tolerance'),
                'min_cluster_size': LaunchConfiguration('min_cluster_size'),
                'max_cluster_size': LaunchConfiguration('max_cluster_size'),
                'target_point_tolerance': LaunchConfiguration('target_point_tolerance'),
            }],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),
        Node(
            package='grasp_vision_cpp',
            executable='optimal_grasp',
            name='optimal_grasp',
            parameters=[{
                'cluster_topic': LaunchConfiguration('cluster_topic'),
                'normal_search_radius': LaunchConfiguration('normal_search_radius'),
                'min_search_threshold': LaunchConfiguration('min_search_threshold'),
                'max_search_threshold': LaunchConfiguration('max_search_threshold'),
                'curvature': LaunchConfiguration('curvature'),
                'visualize': LaunchConfiguration('visualize'),
                'select_stability_metric': LaunchConfiguration('select_stability_metric'),
                'variance_neighbors': LaunchConfiguration('variance_neighbors'),
                'variance_threshold': LaunchConfiguration('variance_threshold'),
                'pos_topic': LaunchConfiguration('pos_topic'),
            }],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_pub",
            # Pitch rotation 30 deg + translation
            arguments = ['--x', '0.1', '--y', '0', '--z', '-0.16', '--yaw', '0', '--pitch', '0.523599', '--roll', '0', '--frame-id', 'drone_frame', '--child-frame-id', 'camera_link']
        )
    ])
