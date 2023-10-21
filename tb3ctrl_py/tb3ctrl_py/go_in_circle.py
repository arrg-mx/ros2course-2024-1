#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from math import pi

class TB3Controller(Node):
    # Definir constructor
    def __init__(self, node_name):
        super().__init__(node_name)
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        # Declaracion del publicador al topico '/cmd_vel'
        self.robot_ctrl_pub = self.create_publisher(Twist, '/cmd_vel', self.qos_profile)
        # Creacion del timer para el publicador
        self.timer_period = 0.01 # 1/100 segundos
        self.timer = self.create_timer(self.timer_period, self._on_robot_ctrl_clbk) # Declaracion del timer(timer_period)
        #Declracion de la variable del mensaje para el publicador
        self.ctrl_msg = Twist()
        # Definicion de constantes y variables de clase
        self.LIN_VEL = 0.07 # Velocidad lineal (m/s)
        self.ANG_VEL = 0.1  # Velocidad angular (rad/s)
        self.RADIUS = self.LIN_VEL/self.ANG_VEL
        self.start_time = self.get_clock().now()
        self.get_logger().info(f"'TB3Controller [{node_name}]' Initialized")

    # Declaracion de la funcion de callback invocada por el timer
    def _on_robot_ctrl_clbk(self):
        elpsed_time = self.get_clock().now() - self.start_time # Elemento de control para el movimiento del robot
        if elpsed_time < Duration(seconds=(2*pi/self.ANG_VEL)):
            self.ctrl_msg.linear.x = self.LIN_VEL  # Velocidad lineal del robot dif (solo se puede mover en el eje x)
            self.ctrl_msg.angular.z = self.ANG_VEL  # Velocidad angular del robot dif (solo se puede mover en el eje x)
            self.get_logger().info("Robot dibujando circulo con radio {:.4f}, elapsed time: {:.2f}".format(self.RADIUS, elpsed_time.to_msg()._sec))
            # print("Robot dibujando circulo con radio {:.4f}".format(self.RADIUS))
        else:
            self.ctrl_msg.linear.x = 0.0
            self.ctrl_msg.angular.z = 0.0 # Velocidad angular del robot dif (solo se puede mover en el eje x)

        self.robot_ctrl_pub.publish(self.ctrl_msg)


# Funcion de inicio del script
def main(args=None):
    rclpy.init(args=args) # Inicia un nuevo proceso en el gestor de la libreria de DDS
    # Comportamiento de ejecusion del nodo
    nodo = TB3Controller('tb3_node_py') # construccion
    rclpy.spin(nodo)                    # ciclo de vida (executor)
    nodo.destroy_node()                 # destruccion

    rclpy.shutdown() # Terminamos el proceso en el gestor de la libreria de DDS


# Punto de inicio del script
if __name__ == '__main()__':
    main()