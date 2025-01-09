import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
import cv2
from cv_bridge import CvBridge

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_streaming')

        self.declare_parameter('system_ip_address', '192.168.1.21')
        self.declare_parameter('camera_topic_name', 'camera/id_70001/image_raw')
        self.system_ip_address = self.get_parameter('system_ip_address').get_parameter_value().string_value
        self.camera_topic_name = self.get_parameter('camera_topic_name').get_parameter_value().string_value

        # Create a publisher for the '/camera/image_raw' topic
        self.image_pub = self.create_publisher(Image, self.camera_topic_name, 10)
        self.bridge = CvBridge()

        # Initialize camera and state variables
        self.cam = None
        self.is_streaming = False

        # Service to start the camera stream
        self.start_service = self.create_service(Trigger, 'px4/start_streaming', self.start_camera)
        # Service to stop the camera stream
        self.stop_service = self.create_service(Trigger, 'px4/stop_streaming', self.stop_camera)

        self.timer = None
        self.get_logger().info('Camera node initialized!')

    def start_camera(self, request, response):
        """Starts the camera stream."""
        if self.is_streaming:
            self.get_logger().info('Camera stream already running.')
            response.success = True
            response.message = 'Camera stream is already running.'
            return response

        # Try to open the camera
        for i in range(0, 10):
            try:
                cam_index = f'/dev/video{i}'
                self.cam = cv2.VideoCapture(cam_index)
                if self.cam.isOpened():
                    self.is_streaming = True
                    self.frame_width = int(self.cam.get(cv2.CAP_PROP_FRAME_WIDTH))
                    self.frame_height = int(self.cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

                    # Set a timer to call the callback function every 100ms (10Hz)
                    self.timer = self.create_timer(0.1, self.publish_image)
                    self.get_logger().info(f'Camera stream started at {cam_index}')
                    response.success = True
                    response.message = 'Camera stream started.'
                    return response
            except Exception as e:
                self.get_logger().warn(f'Error opening camera at {cam_index}: {e}')

        # If we get here, no camera could be opened
        self.get_logger().error("Failed to open camera")
        response.success = False
        response.message = 'Failed to open camera.'
        return response

    def stop_camera(self, request, response):
        """Stops the camera stream."""
        if not self.is_streaming:
            self.get_logger().info('No camera stream running.')
            response.success = True
            response.message = 'No camera stream running.'
            return response

        # Stop the timer and release the camera
        if self.timer:
            self.timer.cancel()
            self.timer = None

        if self.cam:
            self.cam.release()
            self.cam = None
            self.get_logger().info('Camera stream stopped.')

        self.is_streaming = False
        cv2.destroyAllWindows()
        response.success = True
        response.message = 'Camera stream stopped.'
        return response

    def publish_image(self):
        """Capture and publish an image from the camera."""
        if not self.is_streaming:
            return

        ret, frame = self.cam.read()
        if not ret:
            self.get_logger().error("Failed to capture image")
            return

        try:
            # Convert the OpenCV frame to a ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            # Publish the image
            self.image_pub.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f"Error converting OpenCV image to ROS Image message: {e}")
            return

    def cleanup(self):
        """Ensure proper cleanup of resources."""
        if self.is_streaming:
            self.stop_camera(None, None)
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    camera_publisher = CameraPublisher()

    try:
        # Spin the node so it continues to process events
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up before shutting down
        camera_publisher.cleanup()

if __name__ == '__main__':
    main()
