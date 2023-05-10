import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from mavros_msgs.msg import State, ManualControl
import numpy as np
import time

class PositionController:
    def __init__(self):
        self.hxd = 50
        self.hyd = 0
        self.uRef=0
        self.wRef=0
        self.hxe=0
        self.hye=0
        self.a=0.01 #punto de interes
        self.ts=0.01
        self.current_x=0
        self.current_y=0
        self.current_heading=0
        
        self.odom_subscriber = rospy.Subscriber('/mavros/local_position/odom', Odometry, self.odom_callback)
        self.vel_publisher = rospy.Publisher('/vehicle/manual', ManualControl, queue_size=10)

    def odom_callback(self, odom_msg):
        self.current_x = odom_msg.pose.pose.position.x
        self.current_y = odom_msg.pose.pose.position.y
        orientation_quat = odom_msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w])
        self.current_heading = yaw

    def control(self):
        it=time.time()
        self.hxe=self.hxd-self.current_x
        self.hye=self.hyd-self.current_y
        
        he=np.array([[self.hxe],[self.hye]])
        J=np.array([[np.cos(self.current_heading),-self.a*np.sin(self.current_heading)],
                   [np.sin(self.current_heading), self.a*np.cos(self.current_heading)]])
        
        K=np.array([[0.1, 0],
                    [0, 0.1]])
        
        qpRef=np.linalg.pinv(J)@K@he

        uRef=qpRef[0][0]*500
        wRef=-qpRef[1][0]*200
        if uRef>500:
            uRef=500
        if uRef<-500:
            uRef=-500    
        if wRef>200:
            wRef=200
        if wRef<-200:
            wRef=-200
        
        twist_msg = ManualControl()
        twist_msg.z = uRef

        twist_msg.y = wRef
        
        while (time.time()-it<self.ts):
            pass

        self.vel_publisher.publish(twist_msg)



# Inicialización de ROS
rospy.init_node('position_controller')

# Ejemplo de uso
controller = PositionController()

while not rospy.is_shutdown():
    controller.control()