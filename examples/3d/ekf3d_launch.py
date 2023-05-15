from simple_launch import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher()

    ns = 'drone'
                    
    with sl.group(ns=ns):
        
        sl.node('robot_localization_ranges', 'ekf',
                parameters=[sl.find('robot_localization_ranges', 'ekf_3d_example.yaml')],
                output='screen')

        sl.node('tf2_ros','tf2_echo',arguments=[f'{ns}/base_footprint',f'{ns}/base_footprint_gt'],
                output='screen')
        
    return sl.launch_description()
