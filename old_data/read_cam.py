#!/usr/bin/env python3

import rospy
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
import sys

def consume_video():
    Gst.init(None)

    # Configuración del pipeline
    pipeline_str = "udpsrc port=5000 ! application/x-rtp, encoding-name=H264, payload=96 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink sync=false"
    pipeline = Gst.parse_launch(pipeline_str)

    # Iniciar el pipeline
    pipeline.set_state(Gst.State.PLAYING)

    # Esperar a que el nodo sea detenido
    rospy.spin()

    # Detener el pipeline
    pipeline.set_state(Gst.State.NULL)

if __name__ == '__main__':
    rospy.init_node('video_consumer_node')
    try:
        consume_video()
    except rospy.ROSInterruptException:
        pass
