#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from mavros_msgs.msg import RCIn
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from PyQt5.QtWidgets import QApplication, QWidget, QHBoxLayout, QVBoxLayout, QLabel, QPushButton, QSlider
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
import sys

class IAquaBotGUI(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("IAquaBot GUI")
        self.setFixedSize(800, 600)  # Establecer el tamaño fijo de la ventana

        # Variables para almacenar los datos
        self.bateria = 0.0
        self.left_motor_current = 0.0
        self.right_motor_current = 0.0
        self.solar_current = 0.0
        self.cpu_current = 0.0

        # Inicializar el bridge de cv
        self.bridge = CvBridge()

        # Suscripción a los tópicos de ROS
        rospy.init_node('iaquabot_gui', anonymous=True)
        rospy.Subscriber("/boat/info", Float32MultiArray, self.info_callback)
        rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_in_callback)
        rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)

        # Crear layouts
        main_layout = QVBoxLayout()
        battery_layout = QHBoxLayout()
        video_layout = QHBoxLayout()

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

        # Barras de desplazamiento para el throttle y el roll
        self.throttle_slider = QSlider(Qt.Vertical)
        self.roll_slider = QSlider(Qt.Horizontal)

        battery_layout.addWidget(self.throttle_slider)
        battery_layout.addWidget(self.roll_slider)

        # Widget para mostrar el video
        self.video_label = QLabel()
        video_layout.addWidget(self.video_label)

        # Agregar widgets a los layouts
        main_layout.addLayout(battery_layout)
        main_layout.addLayout(video_layout)

        # Botón de cerrar
        self.button_stop = QPushButton("Close")
        self.button_stop.clicked.connect(self.close)
        main_layout.addWidget(self.button_stop)

        self.setLayout(main_layout)

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

    def image_callback(self, msg):
        try:
            # Convertir el mensaje ROS Image a una imagen OpenCV en YUV
            yuyv_image = self.bridge.imgmsg_to_cv2(msg, "yuyv")
            # Convertir la imagen YUYV a BGR
            bgr_image = cv2.cvtColor(yuyv_image, cv2.COLOR_YUV2BGR_YUYV)
            # Convertir la imagen OpenCV a QImage
            height, width, channel = bgr_image.shape
            bytes_per_line = 3 * width
            q_image = QImage(bgr_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
            # Actualizar QLabel con la nueva imagen
            self.video_label.setPixmap(QPixmap.fromImage(q_image))
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

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

def main():
    app = QApplication(sys.argv)
    iaquabot_gui = IAquaBotGUI()
    iaquabot_gui.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
