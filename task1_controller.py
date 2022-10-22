#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import sin,cos,pi


hola_x = 0
hola_y = 0
hola_theta = 0


x_goals= [1, -1, -1, 1, 0]
y_goals = [1, 1, -1, -1, 0]
theta_goals = [0.785,2.355,-2.355,-0.785,0]

def odometryCb(msg):

    global hola_x, hola_y, hola_theta

    hola_x = msg.pose.pose.position.x
    hola_y = msg.pose.pose.position.y

    q = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]

    hola_theta = euler_from_quaternion(q)[2]



def main():


    rospy.init_node('controller', anonymous=True)

    # odom subscriber
    rospy.Subscriber('/odom',Odometry,odometryCb)

    # cmd_vel publisher
    pub = rospy.Publisher('/cmd_vel',Twist,queue_size=100)

    vel = Twist()
    rate = rospy.Rate(10)


    for x,y,theta in zip(x_goals,y_goals,theta_goals):


        while not rospy.is_shutdown():

            # global error
            del_x = x - hola_x
            del_y = y - hola_y

            # robot frame error
            x_err = ( del_x * cos(hola_theta) ) + ( del_y * sin(hola_theta) )
            y_err = ( del_y * cos(hola_theta) ) - ( del_x * sin(hola_theta) )
            theta_err = theta - hola_theta


            # exit if goal is reached
            if x_err < 0.01 and y_err < 0.01 and theta_err < 0.01 :
                break

            print(x_err)
            print(y_err)
            print(theta_err)
            print()

            vel.linear.x = x_err * 2
            vel.linear.y = y_err * 2
            vel.angular.z = theta_err * 5

            pub.publish(vel)
            rate.sleep()
        
        vel.linear.x = 0
        vel.linear.y = 0
        vel.angular.z = 0
        
        pub.publish(vel)
        rate.sleep()



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptExceptionc:
        pass
