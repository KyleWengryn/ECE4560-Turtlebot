#!/usr/bin/env python2.7
  # Import ROS libraries and messages
import rospy
import numpy as np
from sensor_msgs.msg import Image

# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Print "Hello!" to terminal
print "Hello!"

# Initialize the ROS Node named 'opencv_example', allow multiple nodes to be run with this name
rospy.init_node('opencv_example', anonymous=True)

# Print "Hello ROS!" to the Terminal and to a ROS Log file located in ~/.ros/log/loghash/*.log
rospy.loginfo("Hello ROS!")

# Initialize the CvBridge class
bridge = CvBridge()

# Define a function to show the image in an OpenCV Window
def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(3)

def show_image2(img):
    cv2.imshow("Image Window2", img)
    cv2.waitKey(3)

# Define a callback for the Image message
def image_callback(img_msg):
    # log some info about the image topic
    #rospy.loginfo(img_msg.header)

    # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError, e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    # Flip the image 90deg
    # cv_image = cv2.transpose(cv_image)
    # cv_image = cv2.flip(cv_image,1)
    cv_image_arr = np.array(cv_image)
    rospy.loginfo(cv_image_arr.shape)

    for x in 480:
        for y in 640:
            if cv_image_arr[x][y][0] > 200 and cv_image_arr[x][y][1] < 50 and cv_image_arr[x][y][2] < 50:
                pass
            else:
                cv_image_arr[x][y] = [255, 255, 255]
    # Show the converted image
    show_image(cv_image_arr)

def image_callback2(img_msg):
    try:
        # The depth image is a single-channel float32 image
        # the values is the distance in mm in z axis
        cv_image = bridge.imgmsg_to_cv2(img_msg, "32FC1")
        # Convert the depth image to a Numpy array since most cv2 functions
        # require Numpy arrays.
        cv_image_array = np.array(cv_image, dtype = np.dtype('f8'))
        # Normalize the depth image to fall between 0 (black) and 1 (white)
        # http://docs.ros.org/electric/api/rosbag_video/html/bag__to__video_8cpp_source.html lines 95-125
        cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
        # Resize to the desired size
        #cv_image_resized = cv2.resize(cv_image_norm, self.desired_shape, interpolation = cv2.INTER_CUBIC)
        #depthimg = cv_image_resized
        cv2.imshow("Image from my node", cv_image_norm)
        cv2.waitKey(3)

    except CvBridgeError as e:
        print e

# Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
color_image = rospy.Subscriber("/camera/rgb/image_color", Image, image_callback)
depth_image = rospy.Subscriber("/camera/depth_registered/image_raw", Image, image_callback2)


# Initialize an OpenCV Window named "Image Window"
cv2.namedWindow("Image Window", 1)

# Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
while not rospy.is_shutdown():
    rospy.spin()