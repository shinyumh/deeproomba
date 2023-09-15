import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from deepracer_interfaces_pkg.msg import ServoCtrlMsg
from rclpy.parameter import Parameter
from cv_bridge import CvBridge
import cv2
import numpy as np
import os.path

# Nodes in this program
DRC_NODE_NAME = 'deepracer_calibration_node'

# Nodes listening to rosparameters
LG_NODE_NAME = 'lane_guidance_node'
LD_NODE_NAME = 'lane_detection_node'

# Topics subscribed/published to in this program
CAMERA_TOPIC_NAME = '/camera_pkg/display_mjpeg'
THROTTLE_AND_STEERING_TOPIC_NAME = '/ctrl_pkg/servo_msg'

cv2.namedWindow('sliders')


def callback(x):
    pass


def slider_to_normalized(slider_input):
    input_start = 0
    input_end = 2000
    output_start = -1
    output_end = 1
    normalized_output = output_start + (slider_input - input_start) * (
            (output_end - output_start) / (input_end - input_start))
    return normalized_output


lowH = 0
highH = 179
lowS = 0
highS = 255
lowV = 0
highV = 255

not_inverted = 0
inverted = 1

min_width = 10
max_width = 500

max_number_of_lines = 100
max_error_threshold = 100

min_frame_height = 1
max_frame_height = 100
default_frame_width = 100
max_frame_width = 100
default_min_rows = 50
max_rows = 100
default_min_offset = 50
max_offset = 100

steer_left = 0
steer_straight = 1000
steer_right = 2000

steer_sensitivity_max = 100
steer_sensitivity_default = 100

throttle_reverse = 0
throttle_neutral = 1100
throttle_forward = 2000

zero_error_throttle_mode = 0
error_throttle_mode = 1

cv2.createTrackbar('lowH', 'sliders', lowH, highH, callback)
cv2.createTrackbar('highH', 'sliders', highH, highH, callback)
cv2.createTrackbar('lowS', 'sliders', lowS, highS, callback)
cv2.createTrackbar('highS', 'sliders', highS, highS, callback)
cv2.createTrackbar('lowV', 'sliders', lowV, highV, callback)
cv2.createTrackbar('highV', 'sliders', highV, highV, callback)

cv2.createTrackbar('Inverted_filter', 'sliders', not_inverted, inverted, callback)

cv2.createTrackbar('min_width', 'sliders', min_width, max_width, callback)
cv2.createTrackbar('max_width', 'sliders', max_width, max_width, callback)
cv2.createTrackbar('number_of_lines', 'sliders', max_number_of_lines, max_number_of_lines, callback)
cv2.createTrackbar('error_threshold', 'sliders', max_error_threshold, max_error_threshold, callback)

cv2.createTrackbar('frame_width', 'sliders', default_frame_width, max_frame_width, callback)
cv2.createTrackbar('rows_to_watch', 'sliders', default_min_rows, max_rows, callback)
cv2.createTrackbar('rows_offset', 'sliders', default_min_offset, max_offset, callback)

cv2.createTrackbar('Steering_sensitivity', 'sliders', steer_sensitivity_default, steer_sensitivity_max, callback)
cv2.createTrackbar('Steering_value', 'sliders', steer_straight, steer_right, callback)
cv2.createTrackbar('Throttle_mode', 'sliders', zero_error_throttle_mode, error_throttle_mode, callback)
cv2.createTrackbar('Throttle_value', 'sliders', throttle_neutral, throttle_forward, callback)


class DeepRacerCalibration(Node):
    def __init__(self):
        super().__init__(DRC_NODE_NAME)
        self.throttle_and_steering_publisher = self.create_publisher(ServoCtrlMsg, THROTTLE_AND_STEERING_TOPIC_NAME, 10)
        self.throttle_and_steering_publisher
        self.throttle_and_steering_values = ServoCtrlMsg()
        self.camera_subscriber = self.create_subscription(Image, CAMERA_TOPIC_NAME, self.live_calibration_values, 10)
        self.camera_subscriber
        self.bridge = CvBridge()

        # declare parameters
        self.declare_parameter('Hue_low')
        self.declare_parameter('Hue_high')
        self.declare_parameter('Saturation_low')
        self.declare_parameter('Saturation_high')
        self.declare_parameter('Value_low')
        self.declare_parameter('Value_high')
        self.declare_parameter('inverted_filter')
        self.declare_parameter('Width_min')
        self.declare_parameter('Width_max')
        self.declare_parameter('camera_start_height')
        self.declare_parameter('camera_bottom_height')
        self.declare_parameter('camera_left_width')
        self.declare_parameter('camera_right_width')
        self.declare_parameter('Steering_sensitivity')
        self.declare_parameter('zero_error_throttle')
        self.declare_parameter('error_throttle')

        # setting default values for actuators
        self.zero_error_throttle = 0.55
        self.error_throttle = 0.5

        self.max_num_lines_detected = 10
        self.error_threshold = 0.1


    def live_calibration_values(self, data):
        # get trackbar positions
        lowH = cv2.getTrackbarPos('lowH', 'sliders')
        highH = cv2.getTrackbarPos('highH', 'sliders')
        lowS = cv2.getTrackbarPos('lowS', 'sliders')
        highS = cv2.getTrackbarPos('highS', 'sliders')
        lowV = cv2.getTrackbarPos('lowV', 'sliders')
        highV = cv2.getTrackbarPos('highV', 'sliders')
        min_width = cv2.getTrackbarPos('min_width', 'sliders')
        max_width = cv2.getTrackbarPos('max_width', 'sliders')
        
        number_of_lines = cv2.getTrackbarPos('number_of_lines', 'sliders')
        error_threshold = float(cv2.getTrackbarPos('error_threshold', 'sliders')/100)

        inverted_filter = cv2.getTrackbarPos('Inverted_filter', 'sliders')

        crop_width_percent = cv2.getTrackbarPos('frame_width', 'sliders')
        rows_to_watch_percent = cv2.getTrackbarPos('rows_to_watch', 'sliders')
        rows_offset_percent = cv2.getTrackbarPos('rows_offset', 'sliders')

        steer_input = cv2.getTrackbarPos('Steering_value', 'sliders')
        Steering_sensitivity = float(cv2.getTrackbarPos('Steering_sensitivity', 'sliders')/100)

        Throttle_mode = cv2.getTrackbarPos('Throttle_mode', 'sliders')
        throttle_input = cv2.getTrackbarPos('Throttle_value', 'sliders')

        if Throttle_mode == 0:
            self.zero_error_throttle = slider_to_normalized(throttle_input)
        else:
            self.error_throttle = slider_to_normalized(throttle_input)

        self.throttle_and_steering_values.angle = Steering_sensitivity*slider_to_normalized(steer_input)
        self.throttle_and_steering_values.throttle = slider_to_normalized(throttle_input)
        self.throttle_and_steering_publisher.publish(self.throttle_and_steering_values)

        if crop_width_percent < 1:
            crop_width_percent = 1

        if rows_to_watch_percent < 1:
            rows_to_watch_percent = 1

        if rows_offset_percent < 1:
            rows_offset_percent = 1

        # Image processing from slider values
        frame = self.bridge.imgmsg_to_cv2(data)
        height, width, channels = frame.shape

        rows_to_watch_decimal = rows_to_watch_percent / 100
        rows_offset_decimal = rows_offset_percent / 100
        crop_width_decimal = crop_width_percent / 100

        rows_to_watch = int(height * rows_to_watch_decimal)
        rows_offset = int(height * (1 - rows_offset_decimal))

        start_height = int(height - rows_offset)
        bottom_height = int(start_height + rows_to_watch)

        left_width = int((width / 2) * (1 - crop_width_decimal))
        right_width = int((width / 2) * (1 + crop_width_decimal))

        img = frame[start_height:bottom_height, left_width:right_width]

        image_width = right_width-left_width
        image_height = bottom_height-start_height

        # changing color space to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower = np.array([lowH, lowS, lowV])
        higher = np.array([highH, highS, highV])
        mask = cv2.inRange(hsv, lower, higher)

        if inverted_filter == 1:
            bitwise_mask = cv2.bitwise_and(img, img, mask=cv2.bitwise_not(mask))
        else:
            bitwise_mask = cv2.bitwise_and(img, img, mask=mask)

        # changing to gray color space
        gray = cv2.cvtColor(bitwise_mask, cv2.COLOR_BGR2GRAY)

        # changing to black and white color space
        gray_lower = 50
        gray_upper = 255
        (dummy, blackAndWhiteImage) = cv2.threshold(gray, gray_lower, gray_upper, cv2.THRESH_BINARY)
        contours, dummy = cv2.findContours(blackAndWhiteImage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        centers = []
        cx_list = []
        cy_list = []

        start_point = (int(image_width/2),0)
        end_point = (int(image_width/2),int(bottom_height))

        start_point_thresh_pos_x = int((image_width/2)*(1-error_threshold))
        start_point_thresh_neg_x = int((image_width/2)*(1+error_threshold))
        
        start_point_thresh_pos = (start_point_thresh_pos_x,0)
        end_point_thresh_pos = (start_point_thresh_pos_x, int(bottom_height))

        start_point_thresh_neg = (start_point_thresh_neg_x,0)
        end_point_thresh_neg = (start_point_thresh_neg_x, int(bottom_height))

        # plotting contours and their centroids
        for contour in contours[:number_of_lines]:
            # area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(contour)
            if min_width < w < max_width:
                try:
                    x, y, w, h = cv2.boundingRect(contour)
                    img = cv2.drawContours(img, contour, -1, (0, 255, 0), 3)
                    m = cv2.moments(contour)
                    cx = int(m['m10'] / m['m00'])
                    cy = int(m['m01'] / m['m00'])
                    centers.append([cx, cy])
                    cx_list.append(cx)
                    cy_list.append(cy)
                    cv2.circle(img, (cx, cy), 7, (0, 255, 0), -1)
                    img = cv2.line(img, start_point, end_point, (0,255,0), 4)
                    img = cv2.line(img, start_point_thresh_pos, end_point_thresh_pos, (0,0,255), 2)
                    img = cv2.line(img, start_point_thresh_neg, end_point_thresh_neg, (0,0,255), 2)
                except ZeroDivisionError:
                    pass
        try:
            if len(cx_list) > 1:
                error_list = []
                count = 0
                for cx_pos in cx_list:
                    error = float(((image_width/2) - cx_pos) / (image_width/2))
                    error_list.append(error)
                avg_error = (sum(error_list) / float(len(error_list)))
                p_horizon_diff = error_list[0] - error_list[-1]
                if abs(p_horizon_diff) <=error_threshold:
                    error_x = avg_error
                    pixel_error = int((image_width/2)*(1-error_x))
                    mid_x, mid_y = pixel_error, int((image_height/2))
                    self.get_logger().info(f"straight curve: {error_x}, {error_list}")
                else: 
                    for error in error_list:
                        if abs(error) < error_threshold:
                            error = 1
                            error_list[count] = error
                        count+=1
                    error_x = min(error_list, key=abs)
                    error_x_index = error_list.index(min(error_list, key=abs))
                    mid_x, mid_y = cx_list[error_x_index], cy_list[error_x_index]
                    self.get_logger().info(f"curvy road: {error_x}, {error_list}")

                cv2.circle(img, (mid_x, mid_y), 7, (255, 0, 0), -1)
                start_point_error = (int(image_width/2), mid_y)
                img = cv2.line(img, start_point_error, (mid_x, mid_y), (0,0,255), 4)
                #self.centroid_error.data = float(error_x)
                #self.centroid_error_publisher.publish(self.centroid_error)
                centers = []
                cx_list = []
                cy_list = []
            elif len(cx_list) == 1:
                mid_x, mid_y = cx_list[0], cy_list[0]
                self.get_logger().info(f"only detected one line")

                cv2.circle(img, (mid_x, mid_y), 7, (0, 0, 255), -1)
                #self.centroid_error.data = mid_x
                #self.centroid_error_publisher.publish(self.centroid_error)
            else:
                pass
        except ValueError:
            pass

        # plotting results
        cv2.imshow('img', img)
        cv2.imshow('mask', mask)
        cv2.imshow('bitwise_mask', bitwise_mask)
        cv2.imshow('gray', gray)
        cv2.imshow('blackAndWhiteImage', blackAndWhiteImage)
        cv2.waitKey(1)

        # Write files to yaml file for storage
        color_config_path = '/media/deepracer/expansion/finalproject/deepracer_ws/src/c11-team-6-final-project-deeproomba/config/deepracer_config.yaml'
        f = open(color_config_path, "w")
        f.write(
            f"{LD_NODE_NAME}: \n"
            f"  ros__parameters: \n"
            f"    Hue_low : {lowH} \n"
            f"    Hue_high : {highH} \n"
            f"    Saturation_low : {lowS} \n"
            f"    Saturation_high : {highS} \n"
            f"    Value_low : {lowV} \n"
            f"    Value_high : {highV} \n"
            f"    number_of_lines : {number_of_lines} \n"
            f"    error_threshold : {error_threshold} \n"
            f"    Width_min : {min_width} \n"
            f"    Width_max : {max_width} \n"
            f"    inverted_filter : {inverted_filter} \n"
            f"    camera_start_height : {start_height} \n"
            f"    camera_bottom_height : {bottom_height} \n"
            f"    camera_left_width : {left_width} \n"
            f"    camera_right_width : {right_width} \n"
            f"{LG_NODE_NAME}: \n"
            f"  ros__parameters: \n"
            f"    steering_sensitivity : {Steering_sensitivity} \n"
            f"    no_error_throttle : {self.zero_error_throttle} \n"
            f"    error_throttle : {self.error_throttle} \n"
            f"    error_threshold : {error_threshold} \n"
        )
        f.close()


def main(args=None):
    rclpy.init(args=args)
    DRC_publisher = DeepRacerCalibration()
    try:
        rclpy.spin(DRC_publisher)
        DRC_publisher.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print(f"\nShutting down {DRC_NODE_NAME}...")
        DRC_publisher.throttle_and_steering_values.angle = 0.0
        DRC_publisher.throttle_and_steering_values.throttle = 0.0
        DRC_publisher.throttle_and_steering_publisher.publish(DRC_publisher.throttle_and_steering_values)
        cv2.destroyAllWindows()
        DRC_publisher.destroy_node()
        rclpy.shutdown()
        print(f"{DRC_NODE_NAME} shut down successfully.")


if __name__ == '__main__':
    main()
