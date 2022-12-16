from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher()
                    
    with sl.group(ns='r2d2'):
        
        sl.node('robot_localization_ranges', 'trilateration.py',
                parameters=[sl.find('robot_localization_ranges', 'ekf.yaml')],
                output='screen')

        # run comparison
        sl.node('tf2_ros','tf2_echo',arguments=['r2d2/base_footprint','r2d2/base_footprint_gt'],
                output='screen')
        
    return sl.launch_description()
