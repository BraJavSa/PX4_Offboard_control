#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

ros::NodeHandle nh;

// Declaración de mensaje para el vector de datos
std_msgs::Float32MultiArray sensor_data_msg;
float sensor_data[5]; // Vector para almacenar los valores de los sensores
float Sensibilidad = 0.66; // Sensibilidad en Voltios/Amperio para sensor de 30A

// Declaración de publicador para el tópico único
ros::Publisher pub_sensor_data("/boat/info", &sensor_data_msg);

void setup() {
  nh.initNode();
  Serial.begin(115200);

  // Adición del publicador al nodo
  nh.advertise(pub_sensor_data);
}

void loop() {
  // Lectura de los sensores y almacenamiento en el vector
  sensor_data[0] = (analogRead(A0) * (5.0 / 1023.0) - 2.5) / Sensibilidad;  // Solar recharge
  sensor_data[1] = (analogRead(A1) * (5.0 / 1023.0) - 2.5) / Sensibilidad;  // CPU consumption
  sensor_data[2] = (analogRead(A2) * (5.0 / 1023.0) - 2.5) / Sensibilidad;  // Left motor
  sensor_data[3] = (analogRead(A3) * (5.0 / 1023.0) - 2.5) / Sensibilidad;  // Right motor
  sensor_data[4] = fmap(float(analogRead(A6)), 0, 1023, 0.0, 25.0);  // Battery voltage

  // Asignar el vector al mensaje
  sensor_data_msg.data = sensor_data;
  sensor_data_msg.data_length = 
  5;

  // Publicar el mensaje
  pub_sensor_data.publish(&sensor_data_msg);

  nh.spinOnce();
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
