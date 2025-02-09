'''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ColorImageClicker(Node):
    def __init__(self):
        super().__init__('color_image_clicker')

        # Create a subscription to the color image topic
        self.color_image_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  # Change this to your camera's color topic if different
            # '/camera/camera/depth/image_rect_raw',
            self.color_image_callback,
            10
        )

        # Initialize CvBridge to convert ROS images to OpenCV images
        self.bridge = CvBridge()

        # Store the latest color image
        self.color_image = None

        # Set up OpenCV window and mouse click handler
        cv2.namedWindow("Color Image", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("Color Image", self.on_mouse_click)

    def color_image_callback(self, msg):
        """ Callback to receive the color image """
        try:
            # Convert the ROS Image message to an OpenCV image
            color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            self.color_image = color_image
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Display the color image in OpenCV window
        if self.color_image is not None:
            cv2.imshow("Color Image", self.color_image)
            cv2.waitKey(1)

    def on_mouse_click(self, event, x, y, flags, param):
        """ Callback for handling mouse clicks on the image """
        if event == cv2.EVENT_LBUTTONDOWN:
            # Print the 2D pixel coordinates where the user clicked
            # Hardcoded to currently set (depth image res / color image res)
            x = x
            y = y
            x_a = x * 480/1280
            y_a = y * 270/720
            self.get_logger().info(f"Clicked at 2D point: ({x}, {y}), adjusted: ({x_a},{y_a})")

def main(args=None):
    rclpy.init(args=args)

    # Create the node and spin it
    node = ColorImageClicker()
    rclpy.spin(node)

    # Shutdown when the node is stopped
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageClicker(Node):
    def __init__(self):
        super().__init__('image_clicker')

        # Subscriptions to the color and depth image topics
        self.color_image_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  # Adjust topic if needed
            self.color_image_callback,
            10
        )
        self.depth_image_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',  # Adjust topic if needed
            self.depth_image_callback,
            10
        )

        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None

        # Create OpenCV windows
        cv2.namedWindow("Color Image", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Depth Image", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("Color Image", self.on_mouse_click)

    def color_image_callback(self, msg):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert color image: {e}")
            return

        cv2.imshow("Color Image", self.color_image)
        cv2.waitKey(1)

    def depth_image_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")
            return

        depth_display = cv2.normalize(self.depth_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        cv2.imshow("Depth Image", depth_display)
        cv2.waitKey(1)

    def on_mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.depth_image is not None:
            x_a = int(x * self.depth_image.shape[1] / self.color_image.shape[1])
            y_a = int(y * self.depth_image.shape[0] / self.color_image.shape[0])
            depth_value = self.depth_image[y_a, x_a]
            self.get_logger().info(f"Clicked at ({x}, {y}) -> Depth Image ({x_a}, {y_a}) with depth value: {depth_value}")

            depth_display = cv2.normalize(self.depth_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
            depth_display = cv2.cvtColor(depth_display, cv2.COLOR_GRAY2BGR)
            cv2.circle(depth_display, (x_a, y_a), 5, (0, 0, 255), -1)
            cv2.imshow("Depth Image", depth_display)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ImageClicker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
