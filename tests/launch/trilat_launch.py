from ament_index_python.packages import get_package_share_directory
from simple_launch import SimpleLauncher
from numpy import linspace
import os
import yaml

def generate_launch_description():
    sl = SimpleLauncher()
                    
    with sl.group(ns='r2d2'):

        config = os.path.join(
                 get_package_share_directory('robot_localization_ranges'),
                'tests', 'launch', 'ekf.yaml'
        )       
        
        sl.node('robot_localization_ranges', 'trilateration.py',
                parameters=[config],
                output='screen')

        # run comparison
        sl.node('tf2_ros','tf2_echo',arguments=['r2d2/base_footprint','r2d2/base_footprint_gt'],
                output='screen')
        
    return sl.launch_description()
