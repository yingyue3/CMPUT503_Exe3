#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
import numpy as np

import cv2 as cv
from cv_bridge import CvBridge

class CameraReaderNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._camera_info = f"/{self._vehicle_name}/camera_node/camera_info"
        # bridge between OpenCV and ROS
        self._bridge = CvBridge()
        rospy.loginfo("Camera parameters finding...")
        # create window
        # self._window = "camera-reader"
        # cv.namedWindow(self._window, cv.WINDOW_AUTOSIZE)
        # construct subscriber
        self.sub_info = rospy.Subscriber(self._camera_info, CameraInfo, self.callback_info)
        self.sub_image = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback_image)

        self.K = None
        self.D = None
        self._custom_topic = f"/{self._vehicle_name}/custom_node/image/compressed"
        self.pub = rospy.Publisher(self._custom_topic, Image) # queue_size=10
        self.disorted_image = None

    def callback_image(self, msg):
        # convert JPEG bytes to CV image
        rate = rospy.Rate(5)
        if self.K is None:
            return
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        # display frame
        # cv2.imshow(self._window, image)
        # https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
        h,w = image.shape[:2]
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(self.K, self.D, (w,h), 1, (w,h))
        dst = cv.undistort(image, self.K, self.D, None, newcameramtx)
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        self.disorted_image = dst
        rospy.loginfo("Image Calibrated")
        rate.sleep()
        # cv2.imshow(self._window, dst)

    def callback_info(self, msg):
        rate = rospy.Rate(1)
        # https://stackoverflow.com/questions/55781120/subscribe-ros-image-and-camerainfo-sensor-msgs-format
        # http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html
        # https://github.com/IntelRealSense/realsense-ros/issues/709ss
        self.K = np.array(msg.K).reshape(3, 3)
        self.D = np.array(msg.D)
        rospy.loginfo("Camera parameters received.")
        rate.sleep()
    
    def start(self):
        # https://stackoverflow.com/questions/55377442/how-to-subscribe-and-publish-images-in-ros
        rate = rospy.Rate(6)
        while not rospy.is_shutdown():       
            if self.disorted_image is not None:
                rospy.loginfo('publishing image')
                image_msg = self._bridge.cv2_to_imgmsg(self.disorted_image, encoding="bgr8")
                self.pub.publish(image_msg)
            #self.pub.publish(self.raw_image)
            rate.sleep()

if __name__ == '__main__':
    # create the node
    node = CameraReaderNode(node_name='camera_reader_node')
    node.start()
    # keep spinning
    rospy.spin()