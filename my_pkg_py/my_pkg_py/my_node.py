#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MyNodo(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info("Nodo 'my_node' inicializado")

def main(args=None):
    rclpy.init(args=args)
    node = MyNodo()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()