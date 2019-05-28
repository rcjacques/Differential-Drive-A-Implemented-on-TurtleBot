#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import numpy as np
import time

class PathPublisher:
    def __init__(self,radius,l):
        self.radius = radius
        self.L = l

        rospy.init_node('controller',anonymous=True)
        self.pub = rospy.Publisher('mobile_base/commands/velocity',Twist,queue_size=10)

        self.rate = rospy.Rate(1)

    def run(self,path):
        rospy.loginfo('Move...')
        while not rospy.is_shutdown():
            rospy.sleep(1)
            for p in path:
                if p[0] != None:
                    self.drive_rotate(self.pub,[p[0],p[1]],1.2)
                    # self.stop(self.pub)
                    # self.rate.sleep()
                    # rospy.sleep(0.1)
                    # self.drive_rotate(self.pub,[0,0])
                    # rospy.sleep(0.5)
                    print(p)
            break
        rospy.loginfo('Finished')

    # def drive_rotate(self,pub,rpm):
    #     v1 = rpm[0]*(2*np.pi*self.radius)
    #     v2 = rpm[1]*(2*np.pi*self.radius)

    #     omega = (v1-v2)/self.L

    #     vel_msg = Twist()
    #     vel_msg.linear.x = 5*(v1+v2)/11
    #     vel_msg.linear.y = 0
    #     vel_msg.linear.z = 0
    #     vel_msg.angular.x = 0
    #     vel_msg.angular.y = 0
    #     vel_msg.angular.z = omega

    #     pub.publish(vel_msg)
    def drive_rotate(self,pub,rpm,t):
        # v1 = rpm[0]*(2*np.pi*radius)
        # v2 = rpm[1]*(2*np.pi*radius)
        v1 = rpm[0]*self.radius
        v2 = rpm[1]*self.radius
        # print(v1,v2)

        speed = (v1+v2)/2
        omega = (v1-v2)/self.L

        if v1 == v2:
            t = 1.5
            speed = speed
        else:
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

        # if v2!=v1:
        #     R = (L/2)*(v1+v2)/(v2-v1)
        #     # theta = (radius/L)*(v2-v1)*1
        #     theta = omega
        #     dx = 2*R*np.sin(theta/2)*np.cos(theta/2)
        #     dy = 2*R*np.sin(theta/2)*np.sin(theta/2)

        #     # print(dx,dy,theta)
        # else:
        #     theta = omega
        #     dx = v1*np.cos(theta)
        #     dy = v1*np.sin(theta)

            # print(dx,dy,theta)

        t0 = time.time()
        t1 = time.time()

        # correction = 0.0145

        while t1-t0 <= t:
            print(t1-t0,t)
            t1 = time.time()
            vel_msg.linear.x = speed
            vel_msg.angular.z = omega
            pub.publish(vel_msg)

    def stop(self,pub):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        pub.publish(vel_msg)

if __name__ == '__main__':
    try:
        p = PathPublisher(radius=0.076/2,l=0.27)
        path = [[0,10],[10,10],[10,10],[10,10],[10,10],[10,10],[10,10],[10,10],[10,10],[10,10],[10,10],[0,5],[10,0],[0,5],[10,10],[0,5],[10,0],[5,5],[0,5],[10,10]]
        p.run(path)
    except rospy.ROSInterruptException:
        pass
