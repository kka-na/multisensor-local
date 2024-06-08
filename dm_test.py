import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, Imu
from uwb_driver.msg import UwbRange
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError

class StereoImageProcessor:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('stereo_image_processor', anonymous=True)

        # A bridge to convert ROS Image messages to OpenCV images
        self.bridge = CvBridge()

        # Subscribers for the left and right images
        self.sub_image_left = rospy.Subscriber("/left/image_raw", Image, self.callback_left)
        self.sub_image_right = rospy.Subscriber("/right/image_raw", Image, self.callback_right)
        self.sub_imu = rospy.Subscriber("/imu/imu", Imu, self.callback_imu)
        self.sub_uwb = rospy.Subscriber("/uwb_endorange_info", UwbRange, self.callback_uwb)
        self.sub_gt = rospy.Subscriber("/leica/pose/relative", PoseStamped, self.callback_gt)

        # Publisher for disparity map
        self.pub_disparity = rospy.Publisher("/disparity/image", Image, queue_size=10)

        # Placeholders for the images
        self.image_left = None
        self.image_right = None
        self.imu = {'accel':None, 'gyro':None}
        self.uwb = None
        self.gt = None

    def callback_left(self, data):
        try:
            # Convert the image from a ROS image to an OpenCV image
            self.image_left = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def callback_right(self, data):
        try:
            # Convert the image from a ROS image to an OpenCV image
            self.image_right = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        self.process_images()
    
    def callback_imu(self, data):
        self.imu['accel'] = data.linear_acceleration
        self.imu['gyro']= data.angular_velocity

    def callback_uwb(self, data):
        if data.responder_location.x != 0.0:
            self.uwb = data.responder_location
    
    def callback_gt(self, data):
        self.gt = data.pose.position

    def process_images(self):
        if self.image_left is not None and self.image_right is not None:
            # Convert images to grayscale
            image_left_gray = cv2.cvtColor(self.image_left, cv2.COLOR_BGR2GRAY)
            image_right_gray = cv2.cvtColor(self.image_right, cv2.COLOR_BGR2GRAY)

            # Calculate the disparity map
            disparity_map = self.calculate_disparity(image_left_gray, image_right_gray)

            # Publish the disparity map
            try:
                disparity_image = self.bridge.cv2_to_imgmsg(disparity_map.astype(np.uint8), "mono8")
                self.pub_disparity.publish(disparity_image)
                print("IMU Accel: ", self.imu['accel'], "\nIMU Gyro: ", self.imu['gyro'])
                print("UWB Responder Position: ", self.uwb)
                print("GT by Laser: ", self.gt)
            except CvBridgeError as e:
                rospy.logerr(e)

    def calculate_disparity(self, image_left, image_right):
        # Create StereoBM object
        stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
        # Compute disparity map
        disparity = stereo.compute(image_left, image_right)
        return disparity

if __name__ == '__main__':
    try:
        processor = StereoImageProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
