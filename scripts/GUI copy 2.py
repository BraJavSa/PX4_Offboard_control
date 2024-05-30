#!/usr/bin/env python3

import rospy
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
from std_msgs.msg import Float32MultiArray
from mavros_msgs.msg import RCIn
from PyQt5.QtWidgets import QApplication, QWidget, QHBoxLayout, QVBoxLayout, QLabel, QPushButton, QSlider
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal
import sys

class VideoThread(QThread):
    new_frame = pyqtSignal(object)

    def __init__(self):
        super().__init__()

    def run(self):
        Gst.init(None)
        pipeline_str = "udpsrc port=5000 ! application/x-rtp, encoding-name=H264, payload=96 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink"
        pipeline = Gst.parse_launch(pipeline_str)
        pipeline.set_state(Gst.State.PLAYING)

        sink = pipeline.get_by_name("appsink0")
        while True:
            sample = sink.emit("pull-sample")
            buf = sample.get_buffer()
            success, info = buf.map(Gst.MapFlags.READ)
            if success:
                data = info.data
                self.new_frame.emit(data)
                buf.unmap(info)

class IAquaBotGUI(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("IAquaBot GUI")
        self.setFixedSize(640, 600)  # Ajusta el tamaño de la ventana

        # Variables para almacenar los datos
        self.bateria = 0.0
        self.left_motor_current = 0.0
        self.right_motor_current = 0.0
        self.solar_current = 0.0
        self.cpu_current = 0.0

        # Configuración del pipeline de video
        self.video_thread = VideoThread()
        self.video_thread.new_frame.connect(self.update_video)

        # Suscripción a los tópicos de ROS
        rospy.init_node('iaquabot_gui', anonymous=True)
        rospy.Subscriber("/boat/info", Float32MultiArray, self.info_callback)
        rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_in_callback)

        # Crear layouts
        main_layout = QVBoxLayout()

        # Layout para los datos de la batería y corrientes
        battery_layout = QHBoxLayout()

        # Widgets para mostrar los datos de la batería y corrientes
        self.label_bateria = QLabel("Battery: ")
        battery_layout.addWidget(self.label_bateria)

        self.label_left_motor_current = QLabel("Left Motor: ")
        battery_layout.addWidget(self.label_left_motor_current)

        self.label_right_motor_current = QLabel("Right Motor: ")
        battery_layout.addWidget(self.label_right_motor_current)

        self.label_solar_current = QLabel("Solar: ")
        battery_layout.addWidget(self.label_solar_current)

        self.label_cpu_current = QLabel("CPU: ")
        battery_layout.addWidget(self.label_cpu_current)

        main_layout.addLayout(battery_layout)

        # Barras de desplazamiento para el throttle y el roll
        control_layout = QHBoxLayout()
        self.throttle_slider = QSlider(Qt.Vertical)
        self.roll_slider = QSlider(Qt.Horizontal)
        control_layout.addWidget(self.throttle_slider)
        control_layout.addWidget(self.roll_slider)
        main_layout.addLayout(control_layout)

        # Widget para mostrar el video
        self.video_label = QLabel()
        main_layout.addWidget(self.video_label)

        # Botón de cerrar
        self.button_stop = QPushButton("Close")
        self.button_stop.clicked.connect(self.close)
        main_layout.addWidget(self.button_stop)

        self.setLayout(main_layout)

        # Iniciar la captura de video
        self.video_thread.start()

        # Actualizar la interfaz gráfica
        self.update_interface()

    def info_callback(self, data):
        self.left_motor_current = abs(data.data[0])
        self.right_motor_current = abs(data.data[1])
        self.solar_current = abs(data.data[2])
        self.cpu_current = abs(data.data[3])
        self.bateria = data.data[4]

    def rc_in_callback(self, data):
        throttle = self.normalize(data.channels[1], 1100, 1900)
        roll = self.normalize(data.channels[3], 1900, 1100)
        self.update_manual_controls(throttle, roll)

    def normalize(self, value, min_val, max_val):
        return 2 * (value - min_val) / (max_val - min_val) - 1

    def update_manual_controls(self, throttle, roll):
        self.throttle_slider.setValue(int((throttle + 1) * 50))
        self.roll_slider.setValue(int((roll + 1) * 50))

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

        QTimer.singleShot(100, self.update_interface)  # Actualizar cada 100 ms

    def update_video(self, data):
        pixmap = QPixmap()
        pixmap.loadFromData(data)
        self.video_label.setPixmap(pixmap)

    def info_callback(self, data):
        self.left_motor_current = abs(data.data[0])
        self.right_motor_current = abs(data.data[1])
        self.solar_current = abs(data.data[2])
        self.cpu_current = abs(data.data[3])
        self.bateria = data.data[4]

    def rc_in_callback(self, data):
        throttle = self.normalize(data.channels[1], 1100, 1900)
        roll = self.normalize(data.channels[3], 1900, 1100)
        self.update_manual_controls(throttle, roll)

    def normalize(self, value, min_val, max_val):
        return 2 * (value - min_val) / (max_val - min_val) - 1

    def update_manual_controls(self, throttle, roll):
        self.throttle_slider.setValue(int((throttle + 1) * 50))
        self.roll_slider.setValue(int((roll + 1) * 50))


def main():
    app = QApplication(sys.argv)
    iaquabot_gui = IAquaBotGUI()
    iaquabot_gui.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
