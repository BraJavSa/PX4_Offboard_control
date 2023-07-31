import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion

def load_gazebo_model(model_name, model_file, model_pose):
    # Inicializa el nodo de ROS (si aún no está inicializado)
    
    # Espera a que el servicio "/gazebo/spawn_sdf_model" esté disponible
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    
    try:
        # Crea un objeto del servicio para cargar el modelo
        spawn_model_service = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        
        # Lee el contenido del archivo del modelo
        model_path = "/home/brayan/control_ws/src/px4_offboard_control/model/pos_d/" + model_file
        with open(model_path, 'r') as file:
            model_xml = file.read()
        
        # Define la posición y orientación del modelo en el mundo
        model_pose_msg = Pose(
            position=Point(x=model_pose[0], y=model_pose[1], z=model_pose[2]),
            orientation=Quaternion(x=model_pose[3], y=model_pose[4], z=model_pose[5], w=model_pose[6])
        )
        
        # Llama al servicio para cargar el modelo
        response = spawn_model_service(model_name, model_xml, "", model_pose_msg, "world")
        
        # Verifica la respuesta para asegurarse de que el modelo se cargó correctamente
        if response.success:
            print(f"Modelo '{model_name}' cargado correctamente.")
        else:
            print(f"No se pudo cargar el modelo '{model_name}'.")
    except rospy.ServiceException as e:
        print(f"Error al llamar al servicio: {e}")

if __name__ == '__main__':
    # Reemplaza 'pos_d', 'model.sdf' y los valores de posición y orientación con los adecuados
    model_name = 'pos_d'
    model_file = 'model.sdf'
    model_pose = (0, 0, 0.1, 0, 0, 0, 1)  # Posición (x, y, z) y Orientación (x, y, z, w) del modelo
    load_gazebo_model(model_name, model_file, model_pose)
