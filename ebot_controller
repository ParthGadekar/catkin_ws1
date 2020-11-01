#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

def Waypoints(t):
    if t == 0:
        x = 0.74
        y = 0.488
    elif t == 1:
        x = 1.42
        y = 1.289
    elif t == 2:
        x = 1.911
        y = 1.54
    elif t == 3:
        x = 2.45
        y = 1.2
    elif t == 4:
        x = 3.141
        y = 0
    elif t == 5:
        x = 3.91
        y = -1.289
    elif t == 6:
        x = 4.373
        y = -1.54
    elif t == 7:
        x = 5.02
        y = -1.125
    elif t == 8:
        x = 5.72
        y = -0.297
    elif t == 9:
        x = 6.283
        y = 0
    else:
        x = 0
        y = 0

    return [x, y]


def control_loop():
    rospy.init_node('ebot_controller')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odom_callback)

    rate = rospy.Rate(10)

    vel_msg = Twist()
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.0
    pub.publish(vel_msg)
    i = 0

    while not rospy.is_shutdown() and i < 10:

        [x1, y1] = [x, y]
        [x2, y2] = Waypoints(i)

        theta_goal = math.atan((y2-y1)/(x2-x1))
        e_theta = ebot_theta-theta_goal
        vel_msg.linear.x = 10
        vel_msg.angular.z = (-1)*e_theta
        pub.publish(vel_msg)
        i = i+1
        print("Controller message pushed at {}".format(rospy.get_time()))
        rate.sleep()


def odom_callback(data):
    global pose, x, y, ebot_theta
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y,
            euler_from_quaternion([x, y, z, w])[2]]
    ebot_theta = euler_from_quaternion([x, y, z, w])[2]



if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
