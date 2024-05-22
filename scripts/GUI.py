import rospy
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QHBoxLayout, QVBoxLayout, QLabel, QPushButton
from PyQt5.QtGui import QImage, QPixmap, QColor, QPainter, QFont
from PyQt5.QtCore import QTimer

import cv2
import numpy as np

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
        self.image = None

        # Suscripción a los tópicos de ROS
        rospy.init_node('iaquabot_gui', anonymous=True)
        rospy.Subscriber("/boat/info", Float32MultiArray, self.info_callback)
        rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)

        # Crear layouts
        main_layout = QVBoxLayout()
        battery_layout = QHBoxLayout()
        camera_layout = QVBoxLayout()

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

        # Widget para mostrar la cámara
        self.camera_label = QLabel()
        camera_layout.addWidget(self.camera_label)

        # Agregar widgets a los layouts
        main_layout.addLayout(battery_layout)
        main_layout.addLayout(camera_layout)

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

    def image_callback(self, data):
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(data, "bgr8")
        self.image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) if image is not None else None

    def update_interface(self):
        self.label_bateria.setText("Battery: {:.2f} V".format(self.bateria))
        self.label_left_motor_current.setText("Left Motor: {:.2f} A".format(self.left_motor_current))
        self.label_right_motor_current.setText("Right Motor: {:.2f} A".format(self.right_motor_current))
        self.label_solar_current.setText("Solar: {:.2f} A".format(self.solar_current))
        self.label_cpu_current.setText("CPU: {:.2f} A".format(self.cpu_current))

        if self.image is not None:
            height, width, channel = self.image.shape
            bytesPerLine = 3 * width
            qImg = QImage(self.image.data, width, height, bytesPerLine, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qImg)
            self.camera_label.setPixmap(pixmap)
            self.camera_label.setScaledContents(True)
        else:
            # Crear una imagen negra con texto "No Image"
            img_no_image = np.zeros((100, 200, 3), dtype=np.uint8)
            img_no_image.fill(0)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(img_no_image, 'No Image', (10, 50), font, 1, (255, 255, 255), 2, cv2.LINE_AA)

            height, width, channel = img_no_image.shape
            bytesPerLine = 3 * width
            qImg = QImage(img_no_image.data, width, height, bytesPerLine, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qImg)
            self.camera_label.setPixmap(pixmap)
            self.camera_label.setScaledContents(True)

        QTimer.singleShot(100, self.update_interface)  # Actualizar cada 100 ms

def main():
    app = QApplication(sys.argv)
    iaquabot_gui = IAquaBotGUI()
    iaquabot_gui.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
