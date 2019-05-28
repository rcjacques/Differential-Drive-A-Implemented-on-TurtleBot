#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import numpy as np
import time

# path = [[0,10],[10,10],[10,10],[10,10],[10,10],[10,10],[10,10],[10,10],[10,10],[10,10],[10,10],[0,5],[10,0],[0,5],[10,10],[0,5],[10,0],[5,5],[0,5],[10,10]]
# path = [[0, 2.5], [0, 5], [0, 2.5], [5, 5], [5, 5], [5, 5], [5, 5], [5, 5], [5, 5], [5, 5], [5, 5], [5, 5], [5, 5], [5, 5], [5, 5], [5, 5], [5, 5], [5, 2.5], [0, 2.5], [5, 5], [5, 5], [5, 5], [5, 5], [5, 5], [5, 5], [5, 2.5], [2.5, 5], [5, 2.5], [2.5, 5], [5, 2.5], [2.5, 5], [5, 5], [5, 2.5]]
# path = [[0.0, 3.1415926535897927], [0.0, 6.283185307179585], [6.283185307179585, 6.283185307179585], [6.283185307179585, 6.283185307179585], [6.283185307179585, 6.283185307179585], [6.283185307179585, 6.283185307179585], [6.283185307179585, 6.283185307179585], [6.283185307179585, 6.283185307179585], [6.283185307179585, 6.283185307179585], [6.283185307179585, 6.283185307179585], [6.283185307179585, 6.283185307179585], [6.283185307179585, 6.283185307179585], [6.283185307179585, 6.283185307179585], [6.283185307179585, 6.283185307179585], [6.283185307179585, 6.283185307179585], [3.1415926535897927, 6.283185307179585], [6.283185307179585, 3.1415926535897927], [6.283185307179585, 6.283185307179585], [3.1415926535897927, 6.283185307179585], [6.283185307179585, 3.1415926535897927], [6.283185307179585, 6.283185307179585], [0.0, 3.1415926535897927], [6.283185307179585, 6.283185307179585], [6.283185307179585, 3.1415926535897927], [6.283185307179585, 6.283185307179585], [0.0, 3.1415926535897927], [6.283185307179585, 6.283185307179585], [6.283185307179585, 6.283185307179585]]
# path = [[0.0, 10.471975511965978], [10.471975511965978, 10.471975511965978], [10.471975511965978, 10.471975511965978], [10.471975511965978, 10.471975511965978], [10.471975511965978, 10.471975511965978], [10.471975511965978, 10.471975511965978], [10.471975511965978, 10.471975511965978], [10.471975511965978, 10.471975511965978], [0.0, 5.235987755982989], [5.235987755982989, 10.471975511965978], [0.0, 5.235987755982989], [10.471975511965978, 5.235987755982989], [0.0, 5.235987755982989], [10.471975511965978, 5.235987755982989], [10.471975511965978, 5.235987755982989]]
path = [[0.0, 10.471975511965978], [10.471975511965978, 10.471975511965978], [10.471975511965978, 10.471975511965978], [10.471975511965978, 10.471975511965978], [10.471975511965978, 10.471975511965978], [10.471975511965978, 10.471975511965978], [10.471975511965978, 10.471975511965978], [10.471975511965978, 10.471975511965978], [0.0, 5.235987755982989], [5.235987755982989, 10.471975511965978], [0.0, 5.235987755982989], [10.471975511965978, 5.235987755982989], [0.0, 5.235987755982989], [10.471975511965978, 5.235987755982989], [5.235987755982989, 10.471975511965978]]

radius = 0.038
L = 0.3

def move_forward(pub,rpm):
    speed = rpm*(2*np.pi*radius)

    vel_msg = Twist()
    vel_msg.linear.x = speed
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    pub.publish(vel_msg)

def drive_rotate(pub,rpm,t):
    # v1 = rpm[0]*(2*np.pi*radius)
    # v2 = rpm[1]*(2*np.pi*radius)
    v1 = rpm[0]*radius
    v2 = rpm[1]*radius
    # print(v1,v2)

    speed = (v1+v2)/2
    omega = (v1-v2)/L

    if v1 == v2:
        t = 1.65
        speed = speed
    else:
        t = 1
        speed = speed
        omega = omega

    print(rpm,v1,v2,omega)

    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    # pub.publish(vel_msg)

    if v2!=v1:
        R = (L/2)*(v1+v2)/(v2-v1)
        # theta = (radius/L)*(v2-v1)*1
        theta = omega
        dx = 2*R*np.sin(theta/2)*np.cos(theta/2)
        dy = 2*R*np.sin(theta/2)*np.sin(theta/2)

        # print(dx,dy,theta)
    else:
        theta = omega
        dx = v1*np.cos(theta)
        dy = v1*np.sin(theta)

        # print(dx,dy,theta)

    t0 = time.time()
    t1 = time.time()

    # correction = 0.0145

    while t1-t0 <= t:
        t1 = time.time()
        vel_msg.linear.x = speed
        vel_msg.angular.z = omega
        pub.publish(vel_msg)

def stop(pub):
    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    pub.publish(vel_msg)

# def accel(pub,vel,t):
#     for i in range(0,int(vel/t+t)):
#         pass

def talker():
    # pub = rospy.Publisher('chatter', String, queue_size=10)
    # rospy.init_node('talker', anonymous=True)

    rospy.init_node('controller',anonymous=True)
    pub = rospy.Publisher('mobile_base/commands/velocity',Twist,queue_size=10)

    rate = rospy.Rate(2) 
    rpm = [2.5,5]

    np = [[10.471975511965978, 10.471975511965978]]
    ap = [[5.235987755982989, 5.235987755982989]]
    bp = [[5.235987755982989, 0]]
    cp = [[0.0, 5.235987755982989]]
    dp = [[0.0, 10.471975511965978]]
    ep = [[10.471975511965978, 0.0]]
    fp = [[10.471975511965978, 5.235987755982989]]
    gp = [[5.235987755982989, 10.471975511965978]]

    rospy.loginfo('Move...')
    while not rospy.is_shutdown():
        rospy.sleep(0.25)
        # for p in path:
            
            # count = 0
        for p in path:
            # accel(1.2,0.2)
            drive_rotate(pub,[p[0],p[1]],1.1)
            stop(pub)
            rospy.sleep(0.25)
            # rate.sleep()
            # t1 = time.time()
            # print(t1,t0,t1-t0)
            # count += 1

            # rospy.sleep(2)
        # drive_rotate(pub,[rpm[0],rpm[0]])
        # rate.sleep()
        # drive_rotate(pub,[rpm[0],rpm[0]])
        # rate.sleep()
        # drive_rotate(pub,[rpm[0],rpm[0]])
        # rate.sleep()
        # drive_rotate(pub,[rpm[0],rpm[0]])
        # rate.sleep()
        # drive_rotate(pub,[rpm[0],rpm[0]])
        # rate.sleep()
        # for p in path:
        #     drive_rotate(pub,[p[0]/2,p[1]/2])
        #     rospy.sleep(0.24)
        #     print(p)
        break
    rospy.loginfo('Finished')

    # while not rospy.is_shutdown():
    #     hello_str = "hello world %s" % rospy.get_time()
    #     rospy.loginfo(hello_str)
    #     pub.publish(hello_str)
    #     rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
