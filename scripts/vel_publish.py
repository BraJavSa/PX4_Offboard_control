#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import ManualControl

class VelToManualConverter:
    def __init__(self):
        rospy.init_node('vel_to_manual_converter', anonymous=True)

        self.manual_control_pub = rospy.Publisher("vehicle/manual", ManualControl, queue_size=10)
        rospy.Subscriber("vehicle/vel_cmd", Twist, self.vel_cmd_callback)

        # Establecer la frecuencia de ejecución a 100 Hz
        self.rate = rospy.Rate(100)

        # Establecer límites de velocidad
        self.max_linear_speed = 4.0  # Velocidad lineal máxima permitida
      
        self.max_angular_speed = 2.3  # Velocidad angular máxima permitida
    

        # Establecer límites de salidas manuales
        self.max_manual_output = 1000  # Salida manual máxima permitida
 
    
    def vel_cmd_callback(self, data):
        # Implementa la lógica para procesar los datos recibidos en "vehicle/vel_cmd" y convertirlos a ManualControl
        linear_speed = data.linear.x*self.max_manual_output/self.max_linear_speed
        if linear_speed>1000:
            linear_speed=1000
        elif linear_speed<-1000:
            linear_speed=-1000
        else:
            pass


        angular_speed = data.angular.z*self.max_manual_output/self.max_angular_speed
        if angular_speed>1000:
            angular_speed=1000
        elif angular_speed<-1000:
            angular_speed=-1000
        else:
            pass
        # Crear el mensaje ManualControl y establecer los valores apropiados
        manual_control_msg = ManualControl()
        manual_control_msg.z = linear_speed
        manual_control_msg.y = -angular_speed

        # Publicar el mensaje en el topic "vehicle/manual"
        self.manual_control_pub.publish(manual_control_msg)

    def run(self):
        # Hacer que el nodo ROS siga funcionando hasta que se cierre
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        converter_node = VelToManualConverter()
        converter_node.run()
    except rospy.ROSInterruptException:
        pass
