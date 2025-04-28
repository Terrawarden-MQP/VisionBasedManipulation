# Vision-Based Manipulation (VBM) for Terrawarden MQP
Drone pipeline of getting 2D coordinates of an object's centroid, using that to extract the 3D PointCloud Euclidean cluster of the object, and from there determining the optimal grasp for the selected object, if feasible. Optionally for speed increase (but with potential accuracy decrease), set ROS arg `extract` to false for simple 2D->3D conversion without using PCL filters or extraction. For debugging PCL filters, set ROS arg `visualize` to true for publishing clouds for each filter and open in RViz. Currently utilizes ROS2 Humble. If not running on the Jetson with physical RealSense, switch to branch `sim`. 

## Current workflow if not running in task manager
1) Launch the extract cluster and optimal grasp nodes: `ros2 launch grasp_vision_cpp grasp.launch.py`, append `--show-arguments` to see optional ROS arguemnts. To launch individually: 
    a) Run point cluster extractor node: `ros2 run grasp_vision_cpp extract_cluster`
    b) Run optimal grasp node: `ros2 run grasp_vision_cpp optimal_grasp`
2) Visualize in `rviz2` by opening the `vbm_visualize_drone.rviz` file.

## TODO
Some of these can be for future years. These are not comprehsensive and not in any particular order. 
1) Refine optimal grasp code for speed and accuracy, as well as extraction
2) Refine cluster detection and PCL filters with noise / nearby objects for speed and accuracy
3) Change point cluster extractor node to rely on realsense code instead of PCL + ROS (see links in file)
4) Consider making some PCL filters dynamic
5) Document this: https://askubuntu.com/questions/165679/how-to-manage-available-wireless-network-priority
