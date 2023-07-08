from simple_launch import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher()
                    
    with sl.group(ns='robot'):
        
        sl.node('robot_localization_ranges', 'ekf',
                parameters=[sl.find('robot_localization_ranges', 'ekf_2d_example.yaml')],
                output='screen')

        #sl.node('tf2_ros','tf2_echo',arguments=['robot/base_footprint','robot/base_footprint_gt'],
                #output='screen')
        
    return sl.launch_description()
