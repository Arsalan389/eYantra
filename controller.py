#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseArray
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import sin,cos
from time import sleep


hola_x = 0
hola_y = 0
hola_theta = 0


x_goals= []
y_goals = []
theta_goals = []


def task1_goals_Cb(msg):
	global x_goals, y_goals, theta_goals

	x_goals.clear()
	y_goals.clear()
	theta_goals.clear()

	for waypoint_pose in msg.poses:
		x_goals.append(waypoint_pose.position.x)
		y_goals.append(waypoint_pose.position.y)

		orientation_q = waypoint_pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		theta_goal = euler_from_quaternion (orientation_list)[2]
		theta_goals.append(theta_goal)

def odometryCb(msg):

    global hola_x, hola_y, hola_theta

    hola_x = msg.pose.pose.position.x
    hola_y = msg.pose.pose.position.y

    q = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]

    hola_theta = euler_from_quaternion(q)[2]



def main():


    rospy.init_node('controller_node_task1')

    # odom subscribers
    rospy.Subscriber('/odom',Odometry,odometryCb)
    rospy.Subscriber('/task1_goals', PoseArray, task1_goals_Cb)

    # cmd_vel publisher
    pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)

    vel = Twist()
    rate = rospy.Rate(100)

    while len(x_goals) == 0:
        pub.publish(vel)

    for x,y,theta in zip(x_goals,y_goals,theta_goals):

        while not rospy.is_shutdown() :

            # global error
            del_x = x - hola_x
            del_y = y - hola_y

            # robot frame error
            x_err = ( del_x * cos(hola_theta) ) + ( del_y * sin(hola_theta) )
            y_err = ( del_y * cos(hola_theta) ) - ( del_x * sin(hola_theta) )
            theta_err = theta - hola_theta



            vel.linear.x = x_err * 2
            vel.linear.y = y_err * 2
            vel.angular.z = theta_err * 5

            # exit if goal is reached
            if abs(x_err) < 0.0075 and abs(y_err) < 0.0075 and abs(theta_err) < 0.0075 :
                break

            pub.publish(vel)
            rate.sleep()

        vel = Twist()

        pub.publish(vel)
        sleep(1.2);
    

    while 1:
        pub.publish(vel)
        rate.sleep()
        
        



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
