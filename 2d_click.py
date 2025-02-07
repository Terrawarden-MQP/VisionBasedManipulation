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
            x = x * 480/1280
            y = y * 270/720
            self.get_logger().info(f"Clicked at 2D point: ({x}, {y})")

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
