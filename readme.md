# Vision-Based Manipulation (VBM) for Terrawarden MQP
Simulated environment for testing pipeline of getting 2D coordinates of an object's centroid, using that to extract the 3D PointCloud Euclidean cluster of the object, and from there determining the optimal grasp for the selected object, if feasible.

## Current workflow 
(in all terminals, source the workspace after running `colcon build`!!):
1) Launch the simulated environmnet (Gazebo, RViz): `ros2 launch vbm_project_env simulation.launch.py`
   
   a) If the camera fails to spawn (edit path as necessary): `/opt/ros/humble/lib/gazebo_ros/spawn_entity.py -file /home/mcalec/mqp/install/vbm_project_env/share/vbm_project_env/urdf/camera.urdf -entity camera -z 1 -P 1.57`

    b) Remember to close Gazebo from the GUI before hitting Ctrl+C or Ctrl+Z or be prepared to restart Linux to restore Gazebo's functionality!
2) To easily move camera, run this and stop it when in an ideal spot: `ros2 run vbm_project_env move_camera.py`
3) Publish a point representing the center of the detected object in 2D (change x/y as necessary): `ros2 topic pub /target_2d_coords geometry_msgs/Point "{x: 320, y: 240, z: 0}"`

    a) To visualize the 3D point, get the point from step 4, then run `ros2 topic pub /target_coords geometry_msgs/msg/PointStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'camera_depth_optical_frame'}, point: {x: 0.001, y: 0.001, z: 0.632}}"` and open the cooresponding RViz file.
4) Launch the extract cluster and optimal grasp nodes: `ros2 launch grasp_vision_cpp grasp.launch.py`, append `--show-arguments` to see optional ROS arguemnts. To launch individually: 

    a) Run point cluster extractor node: `ros2 run grasp_vision_cpp extract_cluster`

    b) Run optimal grasp node: `ros2 run grasp_vision_cpp optimal_grasp`

## TODO
Some of these can be for future years. These are not comprehsensive and not in any particular order. 
1) Test optimal grasp code
2) Test cluster detection with noise / nearby objects
3) Pipeline from model detection to output here
4) Uncertainty in grasp location (i.e. add robustness)
5) Constraints on potential grasps (get gripper numbers lmao)
6) Test on the Jetson with realsense
7) Change point cluster extractor node to rely on realsense code instead of PCL + ROS (see links in file)
8) Consider making some PCL filters dynamic
9) For flight, get rid of `vbm_project_env` and integrate `grasp.launch.py` with rest of code
10) Document this: https://askubuntu.com/questions/165679/how-to-manage-available-wireless-network-priority