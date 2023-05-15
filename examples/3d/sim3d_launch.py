from simple_launch import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher()

    sl.declare_arg('config_file', default_value=sl.find('robot_localization_ranges', 'ekf_3d_example.yaml'))
    
    sl.node('robot_localization_ranges', 'sim3d',
            parameters = sl.arg_map(('config_file')))
    
    with sl.group(ns='drone'):
        sl.node('slider_publisher', 'slider_publisher', arguments=[sl.find('robot_localization_ranges', 'cmd_vel_3d.yaml')])
        
    sl.rviz(sl.find('robot_localization_ranges', '3d.rviz'))
        
    return sl.launch_description()
