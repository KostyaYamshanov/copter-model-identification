#!/usr/bin/env python3

import rclpy
from copter_model_identification.state import State
from copter_model_identification.logger_node import LoggerNode

def main(args=None):
    rclpy.init(args=args)
    logger = LoggerNode()
    rclpy.spin(logger)
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()