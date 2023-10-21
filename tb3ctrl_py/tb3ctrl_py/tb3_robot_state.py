#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
from tb3ctrl_interfaces.msg import Tb3RobotState
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
t
class TB3Satus(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self._status_msg = Tb3RobotState()
        self._robot_status_pub = self.create_publisher(Tb3RobotState, '/tb3_status', self.qos_profile)
        self._robot_odom_sub = self.create_subscription(Odometry,'/odom', self._on_odom_feedback, self.qos_profile)
        self._twist = Twist()
        self._pose = Pose()
        self._distance = 0.0
        self.get_logger().info(f"'TB3Status [{node_name}]' Initialized")


    def _on_odom_feedback(self, odom_msg):
        self._pose = odom_msg.pose.pose
        self._twist = odom_msg.twist.twist

        if (self._twist.linear.x > 0.0001) and (self._twist.angular.z > 0.001):
            self._status_msg.estado = f'En movimiento: lin {self._twist.linear.x:.4f} ang {self._twist.angular.z:.4f}'
        else:
            self._status_msg.estado = 'Detenido'

        self._robot_status_pub.publish(self._status_msg)


def main(args=None):
    rclpy.init(args=args)
    nodo = TB3Status('tb3_robot_state')
    rclpy.spin(nodo)
    rclpy.shutdown()

if __name__ == '__main__':
    main()