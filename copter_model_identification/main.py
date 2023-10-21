#!/usr/bin/env python3

import rclpy
from copter_model_identification.logger_node import LoggerNode

def main(args=None):
    rclpy.init(args=args)
    logger = LoggerNode()
    rclpy.spin(logger)
    try:
        rclpy.spin(logger)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')

    logger.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()