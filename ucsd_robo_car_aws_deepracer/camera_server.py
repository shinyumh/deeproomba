import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

CAMERA_NODE_NAME = 'camera_server'
CAMERA_TOPIC_NAME = '/camera_pkg/display_mjpeg'
FREQUENCY = 10  # hz
PERIOD = 1 / FREQUENCY


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__(CAMERA_NODE_NAME)
        self.camera_subscriber = self.create_subscription(Image, CAMERA_TOPIC_NAME, self.live_calibration_values, 10)
        self.camera_subscriber
        self.bridge = CvBridge()

    def live_calibration_values(self, data):
        current_frame = self.bridge.imgmsg_to_cv2(data)
        cv2.imshow("camera", current_frame)
        cv2.waitKey(1)

    def path_planner(self, data):
        kp = self.steer_sensitivity
        centroid = data.data[0]
        width = data.data[1]  # width of camera frame

        if data.data == 0:
            error_x = 0
            throttle_float = self.zero_error_throttle
        else:
            error_x = float(centroid - (width / 2))
            throttle_float = self.error_throttle

        steering_float = -1 * float(kp * (error_x / (width / 2)))
        if steering_float < -1:
            steering_float = -1
        elif steering_float > 1:
            steering_float = 1
        else:
            pass

        self.throttle_and_steering_values.angle = steering_float
        self.throttle_and_steering_values.throttle = throttle_float
        self.throttle_and_steering_publisher.publish(self.throttle_and_steering_values)

        self.get_logger().info(
            f'[centroid, width, steering, throttle]: {centroid, width, steering_float, throttle_float}')


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
