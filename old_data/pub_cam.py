#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
import sys

def publish_video():
    Gst.init(None)

    # Configuración del pipeline
    pipeline_str = "v4l2src device=/dev/video0 ! videoconvert ! videoscale ! video/x-raw,width=640,height=480 ! x264enc speed-preset=superfast tune=zerolatency key-int-max=15 bitrate=500 ! h264parse ! rtph264pay config-interval=1 pt=96 ! udpsink host=192.168.88.248 port=5000"
    pipeline = Gst.parse_launch(pipeline_str)

    # Iniciar el pipeline
    pipeline.set_state(Gst.State.PLAYING)

    # Esperar a que el nodo sea detenido
    rospy.spin()

    # Detener el pipeline
    pipeline.set_state(Gst.State.NULL)

if __name__ == '__main__':
    rospy.init_node('pub_cam_node')
    try:
        publish_video()
    except rospy.ROSInterruptException:
        pass
