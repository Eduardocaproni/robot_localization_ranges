from simple_launch import SimpleLauncher
import yaml


def generate_launch_description():
    sl = SimpleLauncher()

    sl.declare_arg('config_file', default_value=sl.find('robot_localization_ranges', 'ekf.yaml'))
    
    sl.node('robot_localization_ranges', 'sim3d.py',
            parameters = sl.arg_map(('config_file')))
    
    with sl.group(ns='r2d2'):
        sl.node('slider_publisher', 'slider_publisher', arguments=[sl.find('robot_localization_ranges', 'cmd_vel.yaml')])
        
    sl.node('rviz2', 'rviz2')
        
    return sl.launch_description()
