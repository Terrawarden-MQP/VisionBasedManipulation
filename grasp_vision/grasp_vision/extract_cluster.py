"""
Vision pipeline to extract object clusters. 

Subscribes to camera depth cloud and separate topic broadcasting center of target object in 2D.
Convert 2D coordinates to 3D points in point cloud. Use Euclidean clustering to detect clusters
and publish the object containing the target 3D point.

Special thanks to: 
https://github.com/IntelRealSense/librealsense/issues/11031#issuecomment-1352879033
https://support.intelrealsense.com/hc/en-us/community/posts/24972964701331--finding-3d-coordinates-of-a-given-point-which-is-specified-in-the-2d-pixel-coordinates [NOT USED]
https://github.com/yehengchen/DOPE-ROS-D435 [NOT USED]
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
import numpy as np
import struct
import pcl 
import sensor_msgs_py.point_cloud2 as pc2

class PointCloudClusterDetector(Node):
    def __init__(self):
        super().__init__('pointcloud_cluster_detector')

        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/realsense/points',  # TODO
            self.pointcloud_callback,
            10)
        
        self.coord_sub = self.create_subscription(
            Point,
            '/target_2d_coords', 
            self.coord_callback,
            10)
        
        self.pointcloud_data = None
        self.latest_2d_point = None

    def pointcloud_callback(self, msg):
        print("Cloud received!")
        self.pointcloud_data = msg

    def coord_callback(self, msg):
        self.latest_2d_point = (msg.x, msg.y)  # Store latest 2D point
        print("New points!")
        if self.pointcloud_data is not None:
            print("Processing")
            self.process_coordinates()
        else:
            print("No cloud received yet")

    def process_coordinates(self):
        # Convert 2D coordinates to an index within the point cloud
        u, v = int(self.latest_2d_point[0]), int(self.latest_2d_point[1])

        # Extract width and height from the PointCloud2 metadata
        width = self.pointcloud_data.width
        height = self.pointcloud_data.height
        # TODO get from topic
        width = 640
        height = 480

        # Calculate the index in the point cloud array
        index = v * width + u

        # Parse the point cloud data to find the 3D point at the index
        points = list(pc2.read_points(self.pointcloud_data, field_names=("x", "y", "z"), skip_nans=True))
        
        if index >= len(points) or index < 0:
            self.get_logger().warning(f"Index {index} out of bounds for point cloud data {len(points)}")
            return

        # Get the 3D point corresponding to the 2D coordinates
        point = points[index]
        x, y, z = point

        # Check if the point is valid
        if not np.isfinite([x, y, z]).all():
            self.get_logger().warning(f"Invalid 3D point ({x}, {y}, {z})")
            return

        self.get_logger().info(f"Converted 3D Point: ({x}, {y}, {z})")
        
        cluster = self.find_object_cluster(self.pointcloud_data,point)
        if cluster is not None:
            self.get_logger().info("Got cluster!")
        else:
            self.get_logger().info("No cluster :/")


    def find_object_cluster(self, pointcloud, target_point): # TODO
        # Convert to PCL PointCloud for clustering
        cloud = pcl.PointCloud(np.array([[p[0], p[1], p[2]] for p in pointcloud.reshape(-1, 3)], dtype=np.float32))
        
        # Set up clustering parameters
        tree = cloud.make_kdtree()
        ec = cloud.make_EuclideanClusterExtraction()
        ec.set_ClusterTolerance(0.02)    # 2cm
        ec.set_MinClusterSize(100)
        ec.set_MaxClusterSize(25000)
        ec.set_SearchMethod(tree)
        
        cluster_indices = ec.Extract()
        
        # Iterate over clusters and check if target_point is in any
        for j, indices in enumerate(cluster_indices):
            points = np.array([cloud[i] for i in indices])
            if any(np.allclose(point, target_point, atol=0.01) for point in points):  # Check if point in cluster
                return points
        return None

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudClusterDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
