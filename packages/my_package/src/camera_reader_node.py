#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, Image, CameraInfo

import cv2
from cv_bridge import CvBridge

class CameraReaderNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._camera_info = f"/{self._vehicle_name}/camera_node/image/camera_info"
        # bridge between OpenCV and ROS
        self._bridge = CvBridge()
        # create window
        self._window = "camera-reader"
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        # construct subscriber
        self.sub_info = rospy.Subscriber(self._camera_info, CameraInfo, self.callback_info)
        self.sub_image = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback_image)

        self.K = None
        self.D = None

    def callback_image(self, msg):
        # convert JPEG bytes to CV image
        # rospy.Rate(5)
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        # display frame
        cv2.imshow(self._window, image)
        # https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
        h,w = image.shape[:2]
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(self.K, self.D, (w,h), 1, (w,h))
        dst = cv.undistort(image, self.K, self.D, None, newcameramtx)
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        cv2.imshow(self._window, dst)

    def callback_info(self, msg):
        # https://stackoverflow.com/questions/55781120/subscribe-ros-image-and-camerainfo-sensor-msgs-format
        # http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html
        # https://github.com/IntelRealSense/realsense-ros/issues/709ss
        self.K = np.array(msg.K).reshape(3, 3)
        self.D = np.array(msg.D)
        rospy.loginfo("Camera parameters received.")
        pass

if __name__ == '__main__':
    # create the node
    node = CameraReaderNode(node_name='camera_reader_node')
    # keep spinning
    rospy.spin()