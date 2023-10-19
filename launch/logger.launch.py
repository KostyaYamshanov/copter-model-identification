from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Mavros 
    # setpoint_position
    # local_position
    # home_position
    # gps_status
    # global_position
    plugins = ["local_position", "param", "waypoint", "sys_status", "altitude", "command", "manual_control", "imu", "setpoint_raw", "rc_io"]
    mavros_params = [{
        "fcu_url": "udp://:14550@:18570",
        "plugin_allowlist": plugins,
        "heartbeat_mav_type": "GCS"
    }]

    return LaunchDescription([
        Node(
            package='copter_model_identification',
            executable='logger',
            name='logger'
        ),
        Node(
            package='mavros',
            executable='mavros_node',
            respawn=True,
            output='screen',
            parameters=mavros_params
        )
    ])