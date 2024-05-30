#!/usr/bin/env python3

import sys
import rospy
from std_msgs.msg import Float32MultiArray
from PyQt5.QtWidgets import QMainWindow, QApplication, QWidget, QVBoxLayout, QLabel, QPushButton, QHBoxLayout, QFrame
from PyQt5.QtCore import QTimer
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GstVideo

class VideoWindow(QMainWindow):
    def __init__(self, node_name):
        super(VideoWindow, self).__init__()
        self.node_name = node_name
        self.setWindowTitle("Video Stream")
        self.resize(640, 600)

        # Widget contenedor para el área de video
        self.video_container = QWidget(self)
        self.video_layout = QVBoxLayout()
        self.video_container.setLayout(self.video_layout)

        # Área de video
        self.video_area = QWidget()
        self.video_layout.addWidget(self.video_area)

        self.setCentralWidget(self.video_container)

        # Widget contenedor para etiquetas y botones
        self.label_container = QFrame(self)
        self.label_container.setFrameShape(QFrame.StyledPanel)
        self.label_layout = QVBoxLayout()
        self.label_container.setLayout(self.label_layout)

        # Variables para almacenar los datos
        self.bateria = 0.0
        self.left_motor_current = 0.0
        self.right_motor_current = 0.0
        self.solar_current = 0.0
        self.cpu_current = 0.0

        # Widgets para mostrar los datos de la batería y corrientes
        self.label_bateria = QLabel("Battery: ")
        self.label_layout.addWidget(self.label_bateria)

        self.label_left_motor_current = QLabel("Left Motor: ")
        self.label_layout.addWidget(self.label_left_motor_current)

        self.label_right_motor_current = QLabel("Right Motor: ")
        self.label_layout.addWidget(self.label_right_motor_current)

        self.label_solar_current = QLabel("Solar: ")
        self.label_layout.addWidget(self.label_solar_current)

        self.label_cpu_current = QLabel("CPU: ")
        self.label_layout.addWidget(self.label_cpu_current)

        # Botón de cerrar
        self.button_stop = QPushButton("Close")
        self.button_stop.clicked.connect(self.close)
        self.label_layout.addWidget(self.button_stop)

        self.layout().addWidget(self.label_container)

        self.init_gstreamer()
        self.show()

    def init_gstreamer(self):
        Gst.init(None)

        # Configuración del pipeline
        pipeline_str = "udpsrc port=5000 ! application/x-rtp, encoding-name=H264, payload=96 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink sync=false"
        self.pipeline = Gst.parse_launch(pipeline_str)

        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self.on_message)

        self.pipeline.set_state(Gst.State.PLAYING)

        self.video_sink = self.pipeline.get_by_interface(GstVideo.VideoOverlay)
        if self.video_sink:
            self.video_sink.set_window_handle(self.video_area.winId())

    def on_message(self, bus, message):
        t = message.type
        if t == Gst.MessageType.EOS:
            self.pipeline.set_state(Gst.State.NULL)
        elif t == Gst.MessageType.ERROR:
            self.pipeline.set_state(Gst.State.NULL)
            err, debug = message.parse_error()
            print("Error:", err, debug)

    def closeEvent(self, event):
        self.pipeline.set_state(Gst.State.NULL)
        event.accept()

    def info_callback(self, data):
        self.left_motor_current = abs(data.data[0])
        self.right_motor_current = abs(data.data[1])
        self.solar_current = abs(data.data[2])
        self.cpu_current = abs(data.data[3])
        self.bateria = data.data[4]
        self.update_interface()

    def update_interface(self):
        # Actualizar la etiqueta de la batería y cambiar el color si es necesario
        if self.bateria < 11.8:
            self.label_bateria.setStyleSheet("color: red;")
        else:
            self.label_bateria.setStyleSheet("color: black;")

        self.label_bateria.setText("Battery: {:.2f} V".format(self.bateria))
        self.label_left_motor_current.setText("Left Motor: {:.2f} A".format(self.left_motor_current))
        self.label_right_motor_current.setText("Right Motor: {:.2f} A".format(self.right_motor_current))
        self.label_solar_current.setText("Solar: {:.2f} A".format(self.solar_current))
        self.label_cpu_current.setText("CPU: {:.2f} A".format(self.cpu_current))

def main():
    node_name = 'video_consumer_node'
    app = QApplication(sys.argv)
    video_window = VideoWindow(node_name)
    
    rospy.init_node(node_name, anonymous=True)
    
    timer = QTimer()
    timer.timeout.connect(lambda: None)
    timer.start(100)
    
    sys.exit(app.exec_())

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
