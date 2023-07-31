#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from setpoint_gazebo_print import load_gazebo_model
from setpoint_gazebo_delete import delete_gazebo_model

class PositionController:
    def __init__(self):
        self.hxd = 200
        self.hyd = 200
        model_name = 'pos_d'
        model_file = 'model.sdf'
        model_pose = (self.hxd, self.hyd, 0.1, 0, 0, 0, 1)  # Posición (x, y, z) y Orientación (x, y, z, w) del modelo
        delete_gazebo_model(model_name)
        load_gazebo_model(model_name, model_file, model_pose)

        self.uRef=0
        self.wRef=0
        self.hxe=0
        self.hye=0
        self.a=0.1 #punto de interes
        self.current_x=0
        self.current_y=0
        self.orientation=0
        self.current_heading=0
        #kv= u max,    wmax= kk*pi + kv*0.5

        self.kv=3.5
        self.kk=0.35
        self.k2=0.07
        self.odom_subscriber = rospy.Subscriber('boat/odom', Odometry, self.odom_callback)
        self.vel_publisher = rospy.Publisher('vehicle/vel_cmd', Twist, queue_size=10)
        self.rate = rospy.Rate(10)

    def odom_callback(self, odom_msg):
        self.current_x = odom_msg.pose.pose.position.x
        self.current_y = odom_msg.pose.pose.position.y
        self.orientation = odom_msg.pose.pose.orientation.z
    

    def control(self):
        self.hxe=self.hxd-self.current_x     
        self.hye=self.hyd-self.current_y     
        error=math.sqrt(self.hxe**2 + self.hye**2)
        a_e= math.atan2(self.hye, self.hxe)-self.orientation

        if error <=1:
            self.uRef=0
            self.wRef=0




        else:
            self.uRef=self.kv*math.tanh(self.k2*error)*math.cos(a_e)
            self.wRef=self.kk*a_e+self.kv*(math.tanh(self.k2*error)/error)*math.sin(a_e)*math.cos(a_e)
 
        
        twist_msg = Twist()
        twist_msg.linear.x = self.uRef
        twist_msg.angular.z = -self.wRef
        

        print(str(self.uRef)+","+str(self.wRef)+", a_e: "+str(a_e)+", error:"+str(error))
        self.vel_publisher.publish(twist_msg)
        self.rate.sleep()



# Inicialización de ROS
rospy.init_node('position_controller')

# Ejemplo de uso
controller = PositionController()

while not rospy.is_shutdown():
    controller.control()