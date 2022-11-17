from simple_launch import SimpleLauncher
from numpy import linspace
import yaml

def generate_launch_description():
    sl = SimpleLauncher()
                    
    with sl.group(ns='r2d2'):
        
        sl.node('robot_localization_ranges', 'ekf',
                parameters=[sl.find('robot_localization_ranges', 'ekf.yaml')],
                output='screen')
        
    return sl.launch_description()