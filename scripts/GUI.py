#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PyQt5.QtWidgets import QApplication, QWidget, QHBoxLayout, QVBoxLayout, QLabel, QPushButton, QSlider
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap, QIcon
import sys
import cv2
import subprocess
import datetime
import signal
import numpy as np

class IAquaBotGUI(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("IAquaBot GUI")
        self.setFixedSize(645, 600)  # Establecer el tamaño fijo de la ventana

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
        rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)

        # Bandera para controlar la grabación
        self.recording_process = None

        # Bandera para verificar si se ha recibido una imagen
        self.image_received = False

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

        # Widget para mostrar el video
        self.video_label = QLabel()
        video_layout.addWidget(self.video_label)

        # Agregar widgets a los layouts
        main_layout.addLayout(battery_layout)
        main_layout.addLayout(video_layout)

        # Crear un layout horizontal para los botones de grabación
        record_button_layout = QHBoxLayout()

        # Botones para iniciar y detener la grabación rosbag
        self.button_start_rosbag_no_cam = QPushButton("Record without Cam")
        self.button_start_rosbag_no_cam.setIcon(QIcon("record.ico"))  # Establecer el icono
        self.button_start_rosbag_no_cam.clicked.connect(self.start_rosbag_record_no_cam)
        record_button_layout.addWidget(self.button_start_rosbag_no_cam)

        self.button_start_rosbag_all = QPushButton("Record All")
        self.button_start_rosbag_all.setIcon(QIcon("start.png"))  # Establecer el icono
        self.button_start_rosbag_all.clicked.connect(self.start_rosbag_record_all)
        record_button_layout.addWidget(self.button_start_rosbag_all)

        self.button_stop_rosbag = QPushButton("Stop Record")
        self.button_stop_rosbag.setIcon(QIcon("stop.png"))  # Establecer el icono
        self.button_stop_rosbag.clicked.connect(self.stop_rosbag_record)
        self.button_stop_rosbag.setEnabled(False)
        record_button_layout.addWidget(self.button_stop_rosbag)

        # Agregar el layout de los botones de grabación al layout principal
        main_layout.addLayout(record_button_layout)

        # Botón de cerrar
        self.button_stop = QPushButton("Close")
        self.button_stop.clicked.connect(self.close)
        main_layout.addWidget(self.button_stop)

        self.setLayout(main_layout)

        # Crear la imagen "No Image"
        self.no_image = self.create_no_image()

        # Actualizar la interfaz gráfica
        self.update_interface()

    def create_no_image(self):
        # Crear una imagen negra de 640x480 con el texto "No Image"
        no_image = np.zeros((480, 640, 3), np.uint8)
        cv2.putText(no_image, 'No Image', (220, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        return no_image

    def info_callback(self, data):
        self.left_motor_current = abs(data.data[0])
        self.right_motor_current = abs(data.data[1])
        self.solar_current = abs(data.data[2])
        self.cpu_current = abs(data.data[3])
        self.bateria = data.data[4]

    def image_callback(self, msg):
        # Convertir el mensaje ROS Image a formato OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Convertir la imagen BGR a RGB
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # Convertir la imagen OpenCV a QImage
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888)

        # Actualizar QLabel con la nueva imagen
        self.video_label.setPixmap(QPixmap.fromImage(q_image))

        # Marcar que se ha recibido una imagen
        self.image_received = True

    def start_rosbag_record(self, exclude_topics=None):
        if self.recording_process is not None:
            return  # Ya hay una grabación en curso

        # Obtener la fecha y hora actual
        now = datetime.datetime.now()
        # Formatear el nombre del archivo rosbag
        rosbag_filename = "boat_{}.bag".format(now.strftime("%Y%m%d_%H%M%S"))
        command = ["rosbag", "record" ,"-a", "-O", rosbag_filename]

        if exclude_topics:
            exclude_regex = '|'.join([topic.replace('/', '\/') for topic in exclude_topics])
            command.extend(["-x", exclude_regex])

        self.recording_process = subprocess.Popen(command)

        self.button_start_rosbag_no_cam.setEnabled(False)
        self.button_start_rosbag_all.setEnabled(False)
        self.button_stop_rosbag.setEnabled(True)

    def start_rosbag_record_no_cam(self):
        self.start_rosbag_record(exclude_topics=["/usb_cam/camera_info",
                "/usb_cam/image_raw",
                "/usb_cam/image_raw/compressed",
                "/usb_cam/image_raw/compressed/parameter_descriptions",
                "/usb_cam/image_raw/compressed/parameter_updates",
                "/usb_cam/image_raw/compressedDepth",
                "/usb_cam/image_raw/compressedDepth/parameter_descriptions",
                "/usb_cam/image_raw/compressedDepth/parameter_updates",
                "/usb_cam/image_raw/theora",
                "/usb_cam/image_raw/theora/parameter_descriptions",
                "/usb_cam/image_raw/theora/parameter_updates"])

    def start_rosbag_record_all(self):
        exclude_topics = [
            "/usb_cam/image_raw",
            "/usb_cam/image_raw/compressed/parameter_descriptions",
            "/usb_cam/image_raw/compressed/parameter_updates",
            "/usb_cam/image_raw/compressedDepth",
            "/usb_cam/image_raw/compressedDepth/parameter_descriptions",
            "/usb_cam/image_raw/compressedDepth/parameter_updates",
            "/usb_cam/image_raw/theora",
            "/usb_cam/image_raw/theora/parameter_descriptions",
            "/usb_cam/image_raw/theora/parameter_updates"
        ]
        self.start_rosbag_record(exclude_topics=exclude_topics)

    def stop_rosbag_record(self):
        if self.recording_process is not None:
            self.recording_process.send_signal(signal.SIGINT)
            self.recording_process.wait()
            self.recording_process = None

        self.button_start_rosbag_no_cam.setEnabled(True)
        self.button_start_rosbag_all.setEnabled(True)
        self.button_stop_rosbag.setEnabled(False)

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

        # Mostrar la imagen "No Image" si no se ha recibido ninguna imagen
        if not self.image_received:
            height, width, channel = self.no_image.shape
            bytes_per_line = 3 * width
            q_image = QImage(self.no_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
            self.video_label.setPixmap(QPixmap.fromImage(q_image))

        QTimer.singleShot(100, self.update_interface)  # Actualizar cada 100 ms

def main():
    app = QApplication(sys.argv)
    iaquabot_gui = IAquaBotGUI()
    iaquabot_gui.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()