from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher()
                
    with sl.group(ns='r2d2'):
        sl.robot_state_publisher('map_simulator', 'r2d2.xacro')
        
        # spawn in robot namespace to get robot_description
        sl.node('map_simulator', 'spawn', 
                parameters = {'radius': 0.4, 'shape': 'square', 'force_scanner': False, 'linear_noise': 0.01, 'angular_noise': 0.01,
                              'static_tf_odom': False})
        
        sl.node('slider_publisher', 'slider_publisher', arguments=[sl.find('map_simulator', 'cmd_vel.yaml')])            
        
    return sl.launch_description()
