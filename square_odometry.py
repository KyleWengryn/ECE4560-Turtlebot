#!/usr/bin/env python

'''
Copyright (c) 2015, Mark Silliman
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

# A very basic TurtleBot script that moves TurtleBot forward indefinitely. Press CTRL + C to stop.  To run:
# On TurtleBot:
# roslaunch turtlebot_bringup minimal.launch
# On work station:
# python goforward.py

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg  import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import time


class GoForward():

    def __init__(self):
        # initiliaze
        rospy.init_node('GoForwardOdometry', anonymous=False)

	    # tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")

        # What function to call when you ctrl + c    
        rospy.on_shutdown(self.shutdown)
        
	    # Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)


        rospy.Subscriber("/odom", Odometry, self.get_rotation)
	    #TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        self.r = rospy.Rate(10)

        # Twist is a datatype for velocity
        self.move_cmd = Twist()
	
        target = 0

        self.yaw = None
        while not self.yaw:
            pass

        target = self.yaw

        self.move_cmd.linear.x = 0.1
        self.move_cmd.angular.z = 0.0

        gain = 0.50

        drive_state = {0: self.go_forward(), 1: self.turn_left()}
        state = 0
	    # as long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():

    
            
            self.go_forward()
            self.turn_left()
        
        

            #rospy.loginfo(self.yaw)
            #print(f"CURRENT: {self.yaw} | ANGULAR VELO: {0.50 * (target - self.yaw)}")
    


    def go_forward(self):
        target = self.yaw

        start = time.time()
        while time.time() - start < 2 and not rospy.is_shutdown():
            z = 0.50 * (target - self.yaw)
            self.move_cmd.linear.x = 0.2
            self.move_cmd.angular.z = z
            self.cmd_vel.publish(self.move_cmd)
            self.r.sleep()
        
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = 0
        self.cmd_vel.publish(self.move_cmd)
        print(self.yaw)
        self.r.sleep()



    def turn_left(self):
        
        start = self.yaw 

        target = start + math.pi/2

        if target > math.pi:
            target = -math.pi + (target - math.pi)
            
       
        while abs(self.yaw - target) > 0.05 and not rospy.is_shutdown():

            if start > math.pi/2 and self.yaw > 0:
                difference = (start + math.pi / 2) - self.yaw
            else:
                difference = target - self.yaw

            
            angular_velo = 3 * difference
            
            if angular_velo > 1.0:
                angular_velo = 1
            
            if angular_velo < -1.0:
                angular_velo = -1

            self.move_cmd.angular.z = angular_velo
            self.move_cmd.linear.x = 0
            self.cmd_vel.publish(self.move_cmd)
            print(self.yaw * 180 / math.pi)
            self.r.sleep()






    def get_rotation(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        self.yaw = yaw

        
        #print(orientation_list)
        #print(roll, pitch, yaw)
        #rospy.loginfo(yaw)
                        
        
    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
	    # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
	    # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        GoForward()
    except Exception as e:
        rospy.loginfo(e)
        rospy.loginfo("GoForward node terminated.")

