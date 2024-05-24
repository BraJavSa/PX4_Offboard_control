#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry

def odom_callback(odom_msg, odom_pub):
    # Obtener la información de la odometría
    position = odom_msg.pose.pose.position
    orientation = odom_msg.pose.pose.orientation

    # Crear un transformador de tf
    br = tf.TransformBroadcaster()

    # Publicar el transformador de tf
    br.sendTransform(
        (position.x, position.y, position.z),
        (orientation.x, orientation.y, orientation.z, orientation.w),
        rospy.Time.now(),
        "base_link",  # Nombre del marco de referencia del robot
        "map"         # Nombre del marco de referencia del mundo
    )

    # Publicar el mensaje de odometría en un nuevo tema
    odom_pub.publish(odom_msg)

def main():
    rospy.init_node('odometry_to_tf_and_publisher')

    # Crear un publicador para el mensaje de odometría
    odom_pub = rospy.Publisher('odometry_visualization_topic', Odometry, queue_size=10)

    # Suscribirse al tema de odometría
    rospy.Subscriber('/mavros/odometry/in', Odometry, odom_callback, odom_pub)

    rospy.spin()

if __name__ == '__main__':
    main()
