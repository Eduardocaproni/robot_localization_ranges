from simple_launch import SimpleLauncher
from numpy import linspace
import yaml


def generate_launch_description():
    sl = SimpleLauncher()
    
    sl.declare_arg('map', default_value=sl.find('map_simulator', 'house.yaml'))
    sl.declare_arg('max_height', default_value=800)
    sl.declare_arg('max_width', default_value=1200)
    sl.declare_arg('rate', default_value=20)
    sl.declare_arg('map_server', default_value=True)
    sl.declare_arg('display', default_value=True)
    sl.declare_arg('publish_gt',default_value=True,description='Publish real anchor position')
    
    # write trilateration file
    ekf_param_file = sl.find('robot_localization_ranges', 'ekf.yaml')
    print('Using EKF param @', ekf_param_file)

    with open(ekf_param_file) as f:
        ekf = yaml.safe_load(f)


                
    ekf_params = ekf['/**']['ros__parameters']
    ranges = [key for key in ekf_params if key.startswith('range0')]
    for key in ranges:
        ekf_params.pop(key)
    ekf_params['range0'] = 'ranges'
    ekf_params['range0_anchors'] = []
    
    xmin = -4.5
    xmax = 10.6
    ymin = -6.28
    ymax = 4.3
    steps = 2
    cov = 0.
    anchors = [(x, y) for x in linspace(xmin, xmax, steps) for y in linspace(ymin, ymax, steps)]
    
    for i,(x,y) in enumerate(anchors):
        name = f'anchor{i+1}'
        ekf_params['range0_anchors'].append(name)
        
        ekf_params['range0_' + name] = {'pose': {'x':float(x), 'y':float(y),'z':0.}}
        
        with sl.group(ns=name):
            sl.include('map_simulator', 'spawn_anchor_launch.py',
                       launch_arguments={'frame': name, 'x': x,'y': y, 'publish_gt': sl.arg('publish_gt'),
                                         'covariance': cov, 'covariance_real': cov})
            
    with open(sl.find('robot_localization_ranges', 'ekf.yaml'), 'w') as f:
        yaml.safe_dump(ekf, f)
    
    sl.node('map_simulator', 'simulator',
            parameters = sl.arg_map(('map', 'max_height', 'max_width', 'rate', 'display')))
    
    with sl.group(if_arg='map_server'):
        sl.node('nav2_map_server','map_server',name='map_server',
            parameters={'yaml_filename': sl.arg('map')})
        sl.node('nav2_lifecycle_manager','lifecycle_manager',name='lifecycle_manager_map',
        output='screen',
        parameters={'autostart': True, 'node_names': ['map_server']})

    with sl.group(ns='r2d2'):
        sl.robot_state_publisher('map_simulator', 'r2d2.xacro')

        # spawn in robot namespace to get robot_description
        sl.node('map_simulator', 'spawn',
                parameters = {'radius': 0.4, 'shape': 'square', 'force_scanner': False, 'linear_noise': 0.1, 'angular_noise': 0.1,
                              'static_tf_odom': False})

        sl.node('slider_publisher', 'slider_publisher', arguments=[sl.find('map_simulator', 'cmd_vel.yaml')])
        
    sl.node('rviz2', 'rviz2', arguments=['-d', sl.find('robot_localization_ranges', 'config.rviz')])
        
    return sl.launch_description()
