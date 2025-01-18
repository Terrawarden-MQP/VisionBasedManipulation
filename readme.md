# Vision-Based Manipulation (VBM) for Terrawarden MQP
Simulated environment for testing pipeline of getting 2D coordinates of an object's centroid, using that to extract the 3D PointCloud Euclidean cluster of the object, and from there determining the optimal grasp for the selected object, if feasible.

## Current workflow 
(in all terminals, source the workspace after running `colcon build`!!):
1) Launch the simulated environmnet (Gazebo, RViz): `ros2 launch vbm_project_env simulation.launch.py`
    1a) If the camera fails to spawn (edit path as necessary): `/opt/ros/humble/lib/gazebo_ros/spawn_entity.py -file /home/mcalec/mqp/install/vbm_project_env/share/vbm_project_env/urdf/camera.urdf -entity camera -z 1 -P 1.57`
    1b) Remember to close Gazebo from the GUI before hitting Ctrl+C or Ctrl+Z or be prepared to restart Linux to restore Gazebo's functionality!
2) To easily move camera, run this and stop it when in an ideal spot: `ros2 run vbm_project_env move_camera.py`
3) Publish a point representing the center of the detected object in 2D (change x/y as necessary): `ros2 topic pub /target_2d_coords geometry_msgs/Point "{x: 320, y: 240, z: 0}"`
    3a) To visualize the 3D point, get the point from step 4, then run `ros2 topic pub /target_coords geometry_msgs/msg/PointStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'camera_link'}, point: {x: 0.001, y: 0.001, z: 0.632}}"` and open the cooresponding RViz file.
4) Launch point cluster extractor node: `ros2 run grasp_vision_cpp extract_cluster`
5) Launch optimal grasp node: `ros2 run grasp_vision_cpp optimal_grasp`

## TODO
Some of these can be for future years. These are not comprehsensive and not in any particular order. 
1) Test optimal grasp code
2) Test cluster detection with noise / nearby objects
3) Pipeline from model detection to output here
4) Uncertainty in grasp location (i.e. add robustness)
5) Constraints on potential grasps (get gripper numbers lmao)
6) Test on the Jetson with realsense
7) Change point cluster extractor node to rely on realsense code instead of PCL + ROS (see links in file)
8) Add optional launch params to tune filters, or make filters dynamic
9) Check topics in RViz file, potentially modify colors to visualize better for testing purposes
10) New launch file (i.e. above minus `vbm_project_env`)
    a) For flight, get rid of `vbm_project_env`

## Launch order (temporary TODO DELETE)
Terminals:
1. `ros2 launch vbm_project_env simulation.launch.py`
2. `ros2 run vbm_project_env move_camera.py` with sub `/opt/ros/humble/lib/gazebo_ros/spawn_entity.py -file /home/mcalec/mqp/install/vbm_project_env/share/vbm_project_env/urdf/camera.urdf -entity camera -z 1 -P 1.57`
3. `ros2 topic pub /target_2d_coords geometry_msgs/Point "{x: 320, y: 240, z: 0}"`
4. `ros2 run grasp_vision_cpp extract_cluster`
5. `ros2 run grasp_vision_cpp optimal_grasp`