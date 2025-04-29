# Vision-Based Manipulation (VBM) for Terrawarden MQP
## Overview
Drone pipeline of getting 2D coordinates of an object's centroid, using that to extract the 3D PointCloud Euclidean cluster of the object, and from there determining the optimal grasp for the selected object, if feasible. Optionally for speed increase (but with potential accuracy decrease), set ROS arg `extract` to false for simple 2D->3D conversion without using PCL filters or extraction. For debugging PCL filters, set ROS arg `visualize` to true for publishing clouds for each filter and open in RViz. Currently utilizes ROS2 Humble. If not running on the Jetson with physical RealSense, switch to branch `sim`. 

## Current workflow if not running in task manager:
1) Launch the extract cluster and optimal grasp nodes: `ros2 launch grasp_vision_cpp grasp.launch.py`, append `--show-arguments` to see optional ROS arguemnts (or see below). To launch individually: 
    a) Run point cluster extractor node: `ros2 run grasp_vision_cpp extract_cluster`
    b) Run optimal grasp node: `ros2 run grasp_vision_cpp optimal_grasp`
2) Visualize in `rviz2` by opening the `vbm_visualize_drone.rviz` file.

## Node Descriptions:
- `extract_cluster`: Receives a 2D point and converts it into 3D. If parameter `visualize` is true, it will then take that point, apply Point Cloud Library filtering, and extract the object cluster for processing by `optimal_grasp` and also publishes the 3D centroid of that object. Setting `visualize` to true is compuationally expensive and currently does not work in real time (takes on average 0.25-0.5 seconds).
- `optimal_grasp`: Receives an object cluster as a point cloud and uses grasp matricies to determine the ideal grasp for the given object. Not as effective with only a 3DOF arm + computationally expensive.
- `static_transform_publisher`: Publish transformation from camera frame to drone/arm frame.

## Additional Testing Script
Warning: some parts of these scripts were hard-coded and do not utilize parameters
- `2d_click.py`: Prints out the (x,y) coordinates of a mouse click and the corresponding depth, and highlights it on the depth map.
- ` 2d_to_3d_test.py`: Change the (u,v) value to extract the (x,y,z) depth value at the pixel of interest.

## Launch Arguments

### Debug & General Settings

| Name         | Default | Description |
|--------------|---------|-------------|
| `log_level`  | `INFO`  | Log verbosity level (e.g., DEBUG, INFO, WARN) |
| `visualize`  | `false` | Enable RViz visualization of PCL filtered clouds and normals (slightly time- and computationally-expensive) |
| `extract`    | `false` | Enable 3D centroid extraction (time- and computationally-expensive); otherwise uses 2D->3D conversion |

### Topic Configuration

| Name                      | Default                                                  | Description |
|---------------------------|----------------------------------------------------------|-------------|
| `cluster_topic`           | `/detected_cluster`                                      | Output topic for extracted clusters |
| `pointcloud_topic`        | `/camera/camera/depth/color/points`                      | Input pointcloud topic from RealSense |
| `coord_topic`             | `/joisie_vision/detected_object_centroid`               | 2D centroid detection topic |
| `camera_info_topic_depth` | `/camera/camera/aligned_depth_to_color/camera_info`      | Depth camera info topic |
| `camera_info_topic_color` | `/camera/camera/color/camera_info`                       | Color camera info topic |
| `camera_depth_topic`      | `/camera/camera/aligned_depth_to_color/image_raw`        | Aligned depth image topic |
| `pos_topic`               | `grasp_pose`                                             | Output topic for computed grasp pose |

### Frame Names

| Name                  | Default                    | Description |
|-----------------------|----------------------------|-------------|
| `header_frame`        | `camera_color_optical_frame` | Camera's color pointcloud reference frame |
| `header_frame_drone`  | `drone_frame`               | Drone's base reference frame |
| `header_frame_depth`  | `camera_depth_optical_frame`| Depth pointcloud reference frame |

### PCL Filtering – `extract_cluster`

| Name                     | Default  | Description |
|--------------------------|----------|-------------|
| `crop_radius`            | `0.2`    | Radius for crop box filter (m) |
| `sor_mean_k`             | `50`     | Mean K for Statistical Outlier Removal |
| `sor_stddev_mul_thresh`  | `1.0`    | Stddev multiplier for SOR |
| `voxel_leaf_size`        | `0.01`   | Leaf size for voxel downsampling (m) |
| `ransac_max_iterations`  | `1000`   | Max iterations for RANSAC plane fitting |
| `ransac_distance_threshold` | `0.005` | Distance threshold for RANSAC |
| `cluster_tolerance`      | `0.02`   | Cluster extraction tolerance (m) |
| `min_cluster_size`       | `100`    | Minimum number of points per cluster |
| `max_cluster_size`       | `25000`  | Maximum number of points per cluster |
| `target_point_tolerance` | `0.02`   | Tolerance for matching to target point (m) |

### Grasp Estimation – `optimal_grasp`

| Name                   | Default  | Description |
|------------------------|----------|-------------|
| `curvature`            | `0.01`   | Surface normal curvature threshold for edge detection |
| `normal_search_radius` | `0.03`   | Search radius for surface normals (m) |
| `min_search_threshold` | `0.035`  | Minimum distance between optimal grasp points |
| `max_search_threshold` | `0.08`   | Maximum distance between optimal grasp points |

### Grasp Stability Metrics

| Name                    | Default | Description |
|-------------------------|---------|-------------|
| `select_stability_metric` | `1`   | Grasp quality metric: <br>1: Max-min SVD<br>2: Max ellipsoid volume<br>3: Isotropy index<br>4: Absolute max-min SVD<br>5: Weighted (1)+(2) |
| `variance_neighbors`    | `4`     | Number of neighbors to estimate grasp uncertainty |
| `variance_threshold`    | `0.2`   | Threshold for variance-based stability filtering |

## Static Transform

A static transform is published between the `drone_frame` and `camera_link` with the following transform:

- Translation: `x = 0.1`, `y = 0`, `z = -0.16`
- Rotation: `pitch = 0.523599` (≈30°), `yaw = 0`, `roll = 0`

## TODO
Some ideas future years. These are not comprehsensive and not in any particular order. 
1) Refine / tune parameters for the `optimal_grasp` code and the object extraction part of `extract_cluster` for speed and accuracy
2) Refine cluster detection and PCL filters with noise / nearby objects for speed and accuracy
3) Change point cluster extractor node to rely on RealSense code instead of PCL + ROS (see links in file)
4) Consider making some PCL filters dynamic
5) Document this: https://askubuntu.com/questions/165679/how-to-manage-available-wireless-network-priority
