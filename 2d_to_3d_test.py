import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class PixelTo3DConverter(Node):
    def __init__(self):
        super().__init__('pixel_to_3d_converter')
        self.bridge = CvBridge()
        self.intrinsics = None

        # Subscribe to depth image and camera info topics
        self.create_subscription(CameraInfo, '/camera/camera/depth/camera_info', self.camera_info_callback, 10)
        self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.depth_image_callback, 10)

    def camera_info_callback(self, msg):
        """ Extract camera intrinsic parameters """
        self.intrinsics = {
            'fx': msg.k[0],  # Focal length x
            'fy': msg.k[4],  # Focal length y
            'cx': msg.k[2],  # Optical center x
            'cy': msg.k[5],  # Optical center y
        }
        self.get_logger().info("Camera intrinsics received.")

    def depth_image_callback(self, msg):
        """ Convert a 2D pixel to 3D using the depth image """
        if self.intrinsics is None:
            self.get_logger().warn("Camera intrinsics not yet received.")
            return
        
        # Convert ROS Image to OpenCV format
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")

        # Example 2D pixel coordinate (u, v)
        u, v = 240, 135  # Change based on the pixel of interest
        depth_scale = 0.001  # Depth scale from meters

        # Get depth value at the given (u, v) coordinate
        depth_value = depth_image[v, u] * depth_scale  # Convert to meters

        if depth_value == 0:
            self.get_logger().warn("Invalid depth value at the selected pixel!")
            return

        # Convert (u, v) to (X, Y, Z) using pinhole camera model
        x = (u - self.intrinsics['cx']) * depth_value / self.intrinsics['fx']
        y = (v - self.intrinsics['cy']) * depth_value / self.intrinsics['fy']
        z = depth_value

        self.get_logger().info(f"3D Point at ({u}, {v}): X={x:.3f}, Y={y:.3f}, Z={z:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = PixelTo3DConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
