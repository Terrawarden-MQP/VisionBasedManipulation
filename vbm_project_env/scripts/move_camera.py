#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from std_srvs.srv import Empty
from geometry_msgs.msg import Quaternion
import math
import time
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2

class MoveCamera(Node):
    def __init__(self):
        super().__init__('move_camera_node')

        # client to pause physics (disable gravity)
        self.physics_client = self.create_client(Empty, '/pause_physics')
        request = Empty.Request()
        future = self.physics_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Physics paused successfully.')
        else:
            self.get_logger().error('Failed to pause physics.')

        # client for Gazebo SetEntityState service
        self.client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the /gazebo/set_entity_state service...')

        # subscribe to PointClouds from camera
        self.pc_subscribe = self.create_subscription(
            PointCloud2,
            '/realsense/points',
            self.pointcloud_callback,
            10
        )
        self.latest_pointcloud = None

        # publishers for TF2 and PointCloud on move
        # https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-TF2.html
        self.tfb_ = TransformBroadcaster(self)
        self.publisher_ = self.create_publisher(PointCloud2,'/move_camera/points',10)

        
        # object parameters for camera locations
        self.height = 0.7 # m
        self.center_x = 0.0 # m
        self.center_y = 0.0 # m
        self.angle = 0.0 # rad
        
        self.roll = 0.0 # rad
        self.pitch = 0.2 # rad
        
        self.count_pos = 5
        self.delta_angle = 6.28/self.count_pos # rad/s
        self.delta_height = 0.1 # m
        self.radius = 0.65 # m
        
        self.timer = 1.0 # s
        self.start_height = self.height
        self.height_steps = 1

        self.running = True

        self.move_camera()

    # moves camera around object
    def move_camera(self):
        request = SetEntityState.Request()
        state = EntityState()
        state.name = 'camera_link'  
        
        def quaternion_from_euler(roll, pitch, yaw):
            q = Quaternion()
            # calculate the half angles and then convert to quaternion
            q.x = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
            q.y = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
            q.z = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
            q.w = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
            return q

        while rclpy.ok() and self.running:
            # position
            state.pose.position.x = self.center_x + self.radius * math.cos(self.angle)
            state.pose.position.y = self.center_y + self.radius * math.sin(self.angle)
            state.pose.position.z = self.height
            
            # face camera toward object at center
            yaw = math.atan2(-state.pose.position.y,-state.pose.position.x) # z axis rotation
            quat = quaternion_from_euler(self.roll, self.pitch, yaw)
            state.pose.orientation = quat
            
            # assign the state to the request
            request.state = state
            
            # call the service to set the entity state
            future = self.client.call_async(request)

            # wait for the response
            rclpy.spin_until_future_complete(self, future)

            if future.result() is not None:
                self.get_logger().info('Camera moved successfully.')
            else:
                self.get_logger().error('Failed to move the camera.')

            # publish pointcloud
            time.sleep(self.timer/2.0) # wait for updated pointcloud
            self.publish_pointcloud()

            # update next location
            self.angle += self.delta_angle

            # after full rotation, reset and increment height
            if self.angle >= 2*math.pi:
                self.angle -= 2*math.pi
                self.height += self.delta_height
                self.get_logger().info(f'Current height: {round(self.height,2)}')
            time.sleep(self.timer/2.0)
            
            # if the camera has reached the desired height, stop the node
            if self.height >= self.start_height + self.height_steps * self.delta_height:
                self.get_logger().info('Camera has reached the desired height.')
                self.running = False
                break

            # publish TF2
            tfs = TransformStamped()
            tfs.header.stamp = self.get_clock().now().to_msg()
            tfs.header.frame_id = "world"
            tfs.child_frame_id = "camera_frame"
            tfs.transform.translation.x = state.pose.position.x
            tfs.transform.translation.y = state.pose.position.y
            tfs.transform.translation.z = state.pose.position.z  

            tfs.transform.rotation.x = state.pose.orientation.x
            tfs.transform.rotation.y = state.pose.orientation.y
            tfs.transform.rotation.z = state.pose.orientation.z
            tfs.transform.rotation.w = state.pose.orientation.w
            self.tfb_.sendTransform(tfs) 
    
    def pointcloud_callback(self,msg):
        # update latest pointcloud
        self.latest_pointcloud = msg

    def publish_pointcloud(self):
        # publish pointcloud to new topic
        if self.latest_pointcloud is not None:
            self.publisher_.publish(self.latest_pointcloud)
            self.get_logger().info("Republished point cloud")

def main(args=None):
    rclpy.init(args=args)
    move_camera_node = MoveCamera()
    rclpy.spin(move_camera_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
