"""Full simulation: Gazebo Harmonic building + 2 TurtleBot3 + 2 elevators + Nav2 per robot + RViz.

    ros2 launch elevator_sim sim.launch.py [gui:=true] [rviz:=true]
"""
import os
import re

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'elevator_sim'
TMP = '/tmp/elevator_sim'
VMAX = {'burger': 0.20, 'waffle': 0.24, 'waffle_pi': 0.24}


def robot_sdf(model, name):
    """TurtleBot3 model SDF with every topic / frame namespaced for robot <name>."""
    src = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models', f'turtlebot3_{model}', 'model.sdf')
    with open(src) as f:
        s = f.read()
    for topic in ('imu', 'scan', 'cmd_vel', 'joint_states', 'camera/image_raw'):
        s = s.replace(f'<topic>{topic}</topic>', f'<topic>/{name}/{topic}</topic>')
    s = s.replace('<camera_info_topic>camera/camera_info</camera_info_topic>',
                  f'<camera_info_topic>/{name}/camera/camera_info</camera_info_topic>')
    s = s.replace('<odom_topic>odom</odom_topic>', f'<odom_topic>/{name}/odom</odom_topic>')
    s = s.replace('<tf_topic>/tf</tf_topic>', f'<tf_topic>/{name}/tf_gz</tf_topic>')
    s = s.replace('<frame_id>odom</frame_id>', f'<frame_id>{name}/odom</frame_id>')
    s = s.replace('<child_frame_id>base_footprint</child_frame_id>',
                  f'<child_frame_id>{name}/base_footprint</child_frame_id>')
    s = re.sub(r'<gz_frame_id>([^<]+)</gz_frame_id>', rf'<gz_frame_id>{name}/\1</gz_frame_id>', s)
    # CPU-only machine: keep the Waffle camera but make it cheap (320x240 @ 5 Hz)
    s = s.replace('<width>1920</width>', '<width>320</width>').replace('<height>1080</height>', '<height>240</height>')
    s = re.sub(r'(<sensor name="camera" type="camera">.*?<update_rate>)30(</update_rate>)', r'\g<1>5\2', s,
               flags=re.S)
    # wheel joint states at 30 Hz instead of every physics step (CPU)
    s = s.replace('<topic>/' + name + '/joint_states</topic>',
                  '<topic>/' + name + '/joint_states</topic><update_rate>30</update_rate>')
    # ground-truth model pose at 10 Hz (used only by the elevators' doorway light curtain)
    s = s.replace('</model>', '<plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">'
                  '<publish_link_pose>false</publish_link_pose><publish_model_pose>true</publish_model_pose>'
                  '<publish_sensor_pose>false</publish_sensor_pose><publish_collision_pose>false</publish_collision_pose>'
                  '<publish_visual_pose>false</publish_visual_pose><publish_nested_model_pose>false</publish_nested_model_pose>'
                  '<use_pose_vector_msg>true</use_pose_vector_msg><update_frequency>10</update_frequency>'
                  '</plugin></model>', 1)
    return s


def bridge_entries(b):
    e = [dict(ros_topic_name='/clock', gz_topic_name='/clock', ros_type_name='rosgraph_msgs/msg/Clock',
              gz_type_name='gz.msgs.Clock', direction='GZ_TO_ROS')]
    for eid in sorted(b['elevators']):
        e.append(dict(ros_topic_name=f'/{eid}/joint_states', gz_topic_name=f'/{eid}/joint_states',
                      ros_type_name='sensor_msgs/msg/JointState', gz_type_name='gz.msgs.Model', direction='GZ_TO_ROS'))
        for t in ['lift_cmd'] + [f'door_f{f}_cmd' for f in b['floors']]:
            e.append(dict(ros_topic_name=f'/{eid}/{t}', gz_topic_name=f'/{eid}/{t}', ros_type_name='std_msgs/msg/Float64',
                          gz_type_name='gz.msgs.Double', direction='ROS_TO_GZ'))
    for name, r in b['robots'].items():
        def add(ros, gz, rt, gt, d, **kw):
            e.append(dict(ros_topic_name=ros, gz_topic_name=gz, ros_type_name=rt, gz_type_name=gt, direction=d, **kw))
        add('/gz_poses', f'/model/{name}/pose', 'tf2_msgs/msg/TFMessage', 'gz.msgs.Pose_V', 'GZ_TO_ROS')
        add(f'/{name}/cmd_vel', f'/{name}/cmd_vel', 'geometry_msgs/msg/Twist', 'gz.msgs.Twist', 'ROS_TO_GZ')
        add(f'/{name}/odom', f'/{name}/odom', 'nav_msgs/msg/Odometry', 'gz.msgs.Odometry', 'GZ_TO_ROS')
        add('/tf', f'/{name}/tf_gz', 'tf2_msgs/msg/TFMessage', 'gz.msgs.Pose_V', 'GZ_TO_ROS')
        add(f'/{name}/joint_states', f'/{name}/joint_states', 'sensor_msgs/msg/JointState', 'gz.msgs.Model',
            'GZ_TO_ROS')
        add(f'/{name}/imu', f'/{name}/imu', 'sensor_msgs/msg/Imu', 'gz.msgs.IMU', 'GZ_TO_ROS')
        add(f'/{name}/scan', f'/{name}/scan', 'sensor_msgs/msg/LaserScan', 'gz.msgs.LaserScan', 'GZ_TO_ROS')
        if r['model'] != 'burger':
            add(f'/{name}/camera/image_raw', f'/{name}/camera/image_raw', 'sensor_msgs/msg/Image', 'gz.msgs.Image',
                'GZ_TO_ROS', lazy=True)
    return e


def setup(context, *_, **__):
    share = get_package_share_directory(PKG)
    cfg_path = os.path.join(share, 'config', 'building.yaml')
    with open(cfg_path) as f:
        b = yaml.safe_load(f)
    os.makedirs(TMP, exist_ok=True)
    actions = []

    # ---------------------------------------------------------------- bridge
    bridge_yaml = os.path.join(TMP, 'bridge.yaml')
    with open(bridge_yaml, 'w') as f:
        yaml.safe_dump(bridge_entries(b), f)
    actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge', output='screen',
                        parameters=[{'config_file': bridge_yaml, 'use_sim_time': True}]))

    # ------------------------------------------------------------ map server
    actions.append(Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
                        parameters=[{'yaml_filename': os.path.join(share, 'maps', 'building.yaml'),
                                     'use_sim_time': True, 'topic_name': 'map', 'frame_id': 'map'}]))
    actions.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
                        output='screen', parameters=[{'use_sim_time': True, 'autostart': True,
                                                      'node_names': ['map_server']}]))

    # --------------------------------------------------------------- robots
    template = open(os.path.join(share, 'config', 'nav2_params.template.yaml')).read()
    tb3_urdf_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'urdf')
    nav_nodes = ['amcl', 'controller_server', 'planner_server', 'behavior_server', 'bt_navigator']
    robot_actions = []
    for name, r in b['robots'].items():
        x, y, yaw = r['home']
        sdf_path = os.path.join(TMP, f'{name}.sdf')
        with open(sdf_path, 'w') as f:
            f.write(robot_sdf(r['model'], name))
        actions.append(Node(package='ros_gz_sim', executable='create', output='screen', name=f'spawn_{name}',
                            arguments=['-world', b['world_name'], '-file', sdf_path, '-name', name,
                                       '-x', str(x), '-y', str(y), '-z', '0.02', '-Y', str(yaw)]))
        with open(os.path.join(tb3_urdf_dir, f'turtlebot3_{r["model"]}.urdf')) as f:
            urdf = f.read()
        actions.append(Node(package='robot_state_publisher', executable='robot_state_publisher', namespace=name,
                            output='screen', parameters=[{'robot_description': urdf, 'use_sim_time': True,
                                                          'frame_prefix': f'{name}/'}]))
        params = (template.replace('@ROBOT@', name).replace('@RADIUS@', str(r['radius']))
                  .replace('@X@', str(x)).replace('@Y@', str(y)).replace('@YAW@', str(yaw))
                  .replace('@VMAX@', str(VMAX[r['model']])))
        params_path = os.path.join(TMP, f'nav2_{name}.yaml')
        with open(params_path, 'w') as f:
            f.write(params)
        for pkg, exe in (('nav2_amcl', 'amcl'), ('nav2_controller', 'controller_server'),
                         ('nav2_planner', 'planner_server'), ('nav2_behaviors', 'behavior_server'),
                         ('nav2_bt_navigator', 'bt_navigator')):
            robot_actions.append(Node(package=pkg, executable=exe, name=exe, namespace=name, output='screen',
                                      parameters=[params_path]))
        robot_actions.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                                  name='lifecycle_manager_navigation', namespace=name, output='screen',
                                  parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': nav_nodes,
                                               'bond_timeout': 10.0}]))
    # give Gazebo time to load the world and spawn the robots before Nav2 starts
    actions.append(TimerAction(period=float(LaunchConfiguration('nav_delay').perform(context)),
                               actions=robot_actions))

    # ------------------------------------------------------------ elevators
    for eid in sorted(b['elevators']):
        actions.append(Node(package=PKG, executable='elevator_controller', name=f'{eid}_controller', output='screen',
                            parameters=[{'elevator_id': eid, 'building_config': cfg_path, 'use_sim_time': False,
                                         'robot_names': list(b['robots'])}]))
    return actions


def generate_launch_description():
    share = get_package_share_directory(PKG)
    world = os.path.join(share, 'worlds', 'building.sdf')
    tb3_models = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models')
    gui = LaunchConfiguration('gui')
    gz_args = ['-r -v 2 ', world]
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Show the Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument('nav_delay', default_value='12.0', description='Seconds before Nav2 starts'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                               tb3_models + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch',
                                                       'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
            condition=IfCondition(gui)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch',
                                                       'gz_sim.launch.py')),
            launch_arguments={'gz_args': ['-r -s -v 2 ', world], 'on_exit_shutdown': 'true'}.items(),
            condition=UnlessCondition(gui)),
        OpaqueFunction(function=setup),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
             arguments=['-d', os.path.join(share, 'rviz', 'elevators.rviz')],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz'))),
    ])

