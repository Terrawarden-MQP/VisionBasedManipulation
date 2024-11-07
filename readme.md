1) Go out of your source director
2) colcon build
3) source install/setup.bash
4) Launch Gazero
   1) ros2 launch vbm_project_env simulator.launch.py
5) Start the publisher node
   1) ros2 run pcl transform_pointcloud
6) Start the subscriber node
   1) ros2 run pcl plane_segmentation
