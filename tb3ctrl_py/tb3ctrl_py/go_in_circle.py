#!/usr/bin/env python3

from typing import List
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class TB3Controller(Node):
    # Definir constructor
    def __init__(self, node_name):
        super().__init__(node_name)
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.robot_ctrl_pub = self.create_publisher(Twist, '/cmd_vel', self.qos_profile)
        self.timer_period = 0.01
        self.timer = self.create_timer(self.timer_period, self._on_robot_ctrl_clbk)
        self.ctrl_msg = Twist()
        self.LIN_VEL = 0.07 # Velocidad lineal (m/s)
        self.ANG_VEL = 0.1  # Velocidad angular (rad/s)

    def _on_robot_ctrl_clbk():
        pass


def main(args=None):
    node = TB3Controller('tb3_node_py')


if __name__ == '__main()__':
    main()