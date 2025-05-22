# Importación de librerías necesarias
import rclpy                                
from rclpy.node import Node                 
from geometry_msgs.msg import PoseWithCovarianceStamped  # Mensaje con posición y orientación (usado por AMCL)
from gazebo_msgs.msg import ModelStates      # Mensaje para obtener los estados de los modelos en Gazebo
import math                                 
from rclpy.qos import qos_profile_sensor_data            # Perfil QoS para datos tipo sensor

# Definición del nodo de comparación de navegación
class NavComparisonNode(Node):
    def __init__(self):
        # Inicializa el nodo con el nombre 'amcl'
        super().__init__('amcl')

        # Variables para almacenar las poses recibidas
        self.amcl_pose = None  # Pose estimada por AMCL
        self.gz_pose = None    # Pose real en Gazebo
        self.robot_name = 'burger'  # Nombre del modelo en Gazebo

        # Suscripción al tópico que publica la pose estimada por AMCL
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',self.amcl_callback,qos_profile_sensor_data)

        # Suscripción al tópico que publica los estados de los modelos en Gazebo
        self.create_subscription(ModelStates,'/model_states',self.gz_callback,qos_profile_sensor_data)

        self.get_logger().info('Nodo activo')  # Mensaje de activación del nodo

    # Callback que guarda la pose publicada por AMCL
    def amcl_callback(self, msg):
        self.amcl_pose = msg.pose.pose

    # Callback que extrae la pose del robot desde los datos de Gazebo
    def gz_callback(self, msg):
        idx = msg.name.index(self.robot_name)  # Encuentra el índice del modelo llamado 'burger'
        self.gz_pose = msg.pose[idx]           # Guarda la pose correspondiente

    # Método para calcular el error de posición (Distancia entre AMCL y Gazebo)
    def calculo_error_posicion(self):
        if self.amcl_pose is None or self.gz_pose is None:
            return float('nan')  # Si no hay datos aún, retorna NaN
        amcl = self.amcl_pose.position
        gz = self.gz_pose.position
        error_posicion = math.sqrt(((amcl.x - gz.x)**2) + ((amcl.y - gz.y)**2)) # Fórmula de distancia euclidiana
        return error_posicion

    # Método para calcular el error de orientación en grados
    def calcular_error_orientacion(self):
        if self.amcl_pose is None or self.gz_pose is None:
            return float('nan')
        amcl = self.amcl_pose.orientation
        gz = self.gz_pose.orientation
        theta_amcl = quaternion_to_yaw(amcl.x, amcl.y, amcl.z, amcl.w) # Conversión de cuaterniones a ángulo yaw
        theta_gz = quaternion_to_yaw(gz.x, gz.y, gz.z, gz.w)
        error = (theta_gz - theta_amcl + 180) % 360 - 180 # Diferencia angular normalizada a [-180, 180]
        return error

# Función auxiliar para convertir cuaternión a yaw (ángulo de orientación en el plano)
def quaternion_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

# Función principal
def main(args=None):
    rclpy.init(args=args)           
    node = NavComparisonNode()      

    # Callback periódico para calcular y mostrar errores cada segundo
    def timer_callback():
        error_pos = node.calculo_error_posicion()
        error_ori = node.calcular_error_orientacion()
        if math.isnan(error_pos) or math.isnan(error_ori):
            node.get_logger().warn('Mover el robot')  # Si no hay datos, se advierte mover el robot
        else:
            node.get_logger().info(f'Error posicion: {error_pos:.4f}, Error orient: {error_ori:.2f}°')

    node.create_timer(1.0, timer_callback)  # Llama al callback cada 1 segundo
    rclpy.spin(node)     
    rclpy.shutdown()     

if __name__ == '__main__':
    main()
