#!/usr/bin/env python2.7
  # Import ROS libraries and messages
import rospy
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Print "Hello!" to terminal
print "Hello!"
rospy.init_node('opencv_example', anonymous=True)
move_cmd = Twist()
r = rospy.Rate(10)
NORM_IMAGE = None
PREVIOUS_LINEAR = 0

cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

# Initialize the ROS Node named 'opencv_example', allow multiple nodes to be run with this name


# Print "Hello ROS!" to the Terminal and to a ROS Log file located in ~/.ros/log/loghash/*.log
rospy.loginfo("Hello ROS!")

# Initialize the CvBridge class
bridge = CvBridge()

# Define a function to show the image in an OpenCV Window

def show_color_highlight(img):
    global NORM_IMAGE, PREVIOUS_LINEAR
    # for x in range(0,480):
    #     for y in range(0, 640):
    #         if cv_image_arr[x][y][0] > 200 and cv_image_arr[x][y][1] < 50 and cv_image_arr[x][y][2] < 50:
    #             pass
    #         else:
    #             cv_image_arr[x][y] = [255, 255, 255]

    # cv2.imshow("Image Window2", cv_image_arr)

    #convert the BGR image to HSV colour space
    result = img.copy()
    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # lower boundary RED color range values; Hue (0 - 10)
    lower1 = np.array([0, 100, 20])
    upper1 = np.array([10, 255, 255])
    
    # upper boundary RED color range values; Hue (160 - 180)
    lower2 = np.array([160,100,20])
    upper2 = np.array([179,255,255])
    
    lower_mask = cv2.inRange(img, lower1, upper1)
    upper_mask = cv2.inRange(img, lower2, upper2)
    
    full_mask = lower_mask + upper_mask

    M = cv2.moments(full_mask)
 
    # calculate x,y coordinate of center
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])

    move_cmd.linear.x = 0.0
    angular = (320 - cX) * 0.1

    if angular > 0.40:
        angular = 0.40
    
    if angular < -0.40:
        angular = -0.40

    move_cmd.angular.z = angular

    if NORM_IMAGE is not None:
        distance = NORM_IMAGE[cY, cX]

    linear = (distance - 0.20) 

    if linear > 0.20:
        linear = 0.20
    if linear < -0.20:
        linear = -0.20


    if distance != 0 and distance < 0.70:       
        move_cmd.linear.x = linear

        cmd_vel.publish(move_cmd)
        r.sleep()

    # goal is to get centroid to be 320, 240

    print cX, cY, angular, linear, distance


    result = cv2.bitwise_and(result, result, mask=full_mask)

    result[cY - 20:cY +20,cX-20:cX+20] = (255, 0, 0) 

    
    #cv2.imshow('mask', full_mask)
    cv2.imshow('result', result)
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
    show_color_highlight(cv_image_arr)

    # Show the converted image
    #show_image(cv_image)

def image_callback2(img_msg):
    global NORM_IMAGE
    try:
        # The depth image is a single-channel float32 image
        # the values is the distance in mm in z axis
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        # Convert the depth image to a Numpy array since most cv2 functions
        # require Numpy arrays.
        NORM_IMAGE = cv2.normalize(cv_image, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
        #cv2.imshow("Image from my node", NORM_IMAGE)
        #cv2.waitKey(3)

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