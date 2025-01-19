from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    common_params = {
        'cluster_topic': '/detected_cluster'
    }
    return LaunchDescription([
        Node(
            package='grasp_vision_cpp', 
            executable='extract_cluster',
            name='extract_cluster',
            parameters=[
                {
                    common_params,
                    'pointcloud_topic': '/realsense/points',
                    'coord_topic': '/target_2d_coords',
                    'camera_info_topic': '/realsense/camera_info',
                    'visualize': False,
                    'crop_radius': 0.2,
                    'sor_mean_k': 50,
                    'sor_stddev_mul_thresh': 1.0,
                    'voxel_leaf_size': 0.01,
                    'ransac_max_iterations': 1000,
                    'ransac_distance_threshold': 0.01,
                    'header_frame': 'camera_link',
                    'cluster_tolerance': 0.02,
                    'min_cluster_size': 100,
                    'max_cluster_size': 25000,
                    'target_point_tolerance': 0.02
                }
            ]
        ),
        Node( # TODO
            package='grasp_vision_cpp', 
            executable='optimal_grasp',
            name='optimal_grasp',
            parameters=[
                {
                    common_params,
                    'normal_search_radius': 0.03,
                    'robust_search': False,
                    'min_search_threshold': 0.02,
                    'max_search_threshold': 0.1
                    
                }
            ]
        )
    ])
