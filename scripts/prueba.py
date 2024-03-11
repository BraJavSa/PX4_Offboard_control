#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

def publish_thrust_cmds():
    rospy.init_node('thrust_cmds_publisher', anonymous=True)
    
    left_thrust_pub = rospy.Publisher('/wamv/thrusters/left_thrust_cmd', Float32, queue_size=10)
    left1_thrust_pub = rospy.Publisher('/wamv/thrusters/left1_thrust_cmd', Float32, queue_size=10)
    right_thrust_pub = rospy.Publisher('/wamv/thrusters/right_thrust_cmd', Float32, queue_size=10)
    right1_thrust_pub = rospy.Publisher('/wamv/thrusters/right1_thrust_cmd', Float32, queue_size=10)
    
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        left_thrust_pub.publish(1.0)
        left1_thrust_pub.publish(1.0)
        right_thrust_pub.publish(-1.0)
        right1_thrust_pub.publish(-1.0)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_thrust_cmds()
    except rospy.ROSInterruptException:
        pass
