#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_callback(msg):
    # Convert the ROS Image message to OpenCV format
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

    # Display the image
    cv2.imshow("Camera Image", cv_image)
    cv2.waitKey(1)

def main():
    # Initialize the ROS node
    rospy.init_node('image_viewer', anonymous=True)

    # Subscribe to the camera image topic
    rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    main()
