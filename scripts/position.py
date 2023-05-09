import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State, ManualControl
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import math
import numpy as np

class PositionController:
    def __init__(self):
        self.hxd = 80
        self.hyd = 0
        self.uRef=0
        self.wRef=0
        self.hxe=0
        self.hye=0
        self.a=0.1 #punto de interes
        
        self.odom_subscriber = rospy.Subscriber('/mavros/odometry/in', Odometry, self.odom_callback)
        self.vel_publisher = rospy.Publisher('/vehicle/manual', ManualControl, queue_size=10)

    def odom_callback(self, odom_msg):
        self.current_x = odom_msg.pose.pose.position.x
        self.current_y = odom_msg.pose.pose.position.y
        orientation_quat = odom_msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w])
        self.current_heading = yaw

    def control(self):
        a=0.1 #punto de interes
        self.hxe=self.hxd - self.current_x
        self.hye=self.hyd - self.current_y
        he=np.array([[hxe]],[[hye]])
        J=np.array([[np.cos(phi)]])
        # Cálculo de la distancia al objetivo
        dx = 
        dy = self.target_y - self.current_y
        distance = (dx ** 2 + dy ** 2) ** 0.5

        # Cálculo del ángulo al objetivo
        target_angle = math.atan(dy / dx) if dx != 0 else math.pi / 2
        if dx < 0:
            target_angle += math.pi

        # Cálculo del error de orientación
        heading_error = target_angle - self.current_heading

        # Parámetros del controlador
        max_linear_speed = 1.0 # m/s
        max_angular_speed = 1.0 # rad/s

        # Cálculo de las velocidades lineal y angular deseadas
        print()
        v = max_linear_speed * max(dx, self.distance_tolerance)
        w = max_angular_speed * heading_error
        w = 0

        # Normalización de las velocidades entre -1000 y 1000
        normalized_v = (v / max_linear_speed) * 1000
        normalized_w = (w / max_angular_speed) * 1000

        # Creación y publicación del mensaje de velocidad
        twist_msg = ManualControl()
        twist_msg.z = normalized_v
        twist_msg.y = normalized_w
        self.vel_publisher.publish(twist_msg)

# Inicialización de ROS
rospy.init_node('position_controller')

# Ejemplo de uso
controller = PositionController(80, 0)

while not rospy.is_shutdown():
    controller.control()