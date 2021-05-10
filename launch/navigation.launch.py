
import os
from launch import LaunchDescription
from launch_ros.actions import Node

# initial pose
initial_pose_x = 0
initial_pose_y = 0
initial_pose_a = 0

# frame name
global_frame = 'map'
odom_frame = 'odom'
base_frame = 'base_footprint'
base_link_frame = 'base_link'
gps_frame = 'gps'
laser_frame = 'laser'

nav2_param_file = '/home/common/ros2_ws/src/bot_navigation/param/nav2.yaml'
nav2_on = True

def generate_launch_description():

    if nav2_on:
        print("Navigation2: ON")
        return LaunchDescription([
            # lawnmower odometry
            Node(
                package='bot_navigation',
                executable='lawnmower_odometry',
                output='screen',
                parameters=[{
                    'odom_frame_id': odom_frame,
                    'base_frame_id': base_frame,
                    'print_tf': True,
                    'pub_odom_topic_name': 'odometry/wheel',
                    'sub_odom_topic_name': '/odom'
                }]
            ),
            # map_server
            Node(
                package='nav2_map_server',
                executable='map_server',
                output='screen',
                parameters=[
                    "/home/common/ros2_ws/src/bot_navigation/param/map_server_params.yaml"
                    #{
                    #'yaml_filename': 'maps/map_square/map.yaml'
                    #'yaml_filename': '~/ros2_ws/src/bot_navigation/maps/map_square/map.yaml'
                    #'yaml_filename': '/home/common/ros2_ws/src/bot_navigation/maps/map_square/map.yaml'
                #}]
                ]
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{
                    'autostart': True,
                    'node_names': ['map_server']
                }]
            ),
            # static_tf
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='world_to_map',
                arguments=['0', '0', '0', '0', '0', '0', 'world_link', global_frame]
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_to_measured',
                arguments=['0', '0', '0', '0', '0', '0', global_frame, 'base_measured']
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='foot_to_gps',
                arguments=['0.08', '0', '0.75', '0', '0', '0', base_frame, gps_frame]
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='foot_to_laser',
                arguments=['0.08', '0', '0.65', '0', '0', '0', base_frame, laser_frame]
            ),
            # map to odom (ekf)
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[
                    '/home/common/ros2_ws/src/bot_navigation/param/ekf.yaml',
                    {
                        'map_frame': global_frame,
                        'odom_frame': odom_frame,
                        'base_link_frame': base_frame,
                        'world_frame': global_frame
                        #'world_frame': odom_frame
                    }
                ],
                remappings=[('/set_pose', '/initialpose')]
            ),
            # nav2
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                parameters=[nav2_param_file],
                #remappings=remappings
                remappings=[('odom', '/odometry/filtered')]
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[nav2_param_file]
                #remappings=remappings
            ),
            Node(
                package='nav2_recoveries',
                executable='recoveries_server',
                name='recoveries_server',
                output='screen',
                parameters=[nav2_param_file],
                #remappings=remappings
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[nav2_param_file],
                #remappings=remappings
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[nav2_param_file],
                #remappings=remappings
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'autostart': True,
                    'node_names': [
                        'controller_server',
                        'planner_server',
                        'recoveries_server',
                        'bt_navigator',
                        'waypoint_follower'
                    ]
                }]
            )
        ])

    else:
        print("Navigation2: OFF")
        return LaunchDescription([
            # map_server
            Node(
                package='nav2_map_server',
                executable='map_server',
                output='screen',
                parameters=[{
                    #'yaml_filename': '~/ros2_ws/src/bot_navigation/maps/map_square/map.yaml'
                    'yaml_filename': '/home/common/ros2_ws/src/bot_navigation/maps/map_square/map.yaml'
                    #'yaml_filename': 'maps/map_square/map.yaml'
                }]
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{
                    'autostart': True,
                    'node_names': ['map_server']
                }]
            ),
            # static_tf
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='world_to_map',
                arguments=['0', '0', '0', '0', '0', '0', 'world_link', global_frame]
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_to_measured',
                arguments=['0', '0', '0', '0', '0', '0', global_frame, 'base_measured']
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='foot_to_gps',
                arguments=['0.08', '0', '0.75', '0', '0', '0', base_frame, gps_frame]
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='foot_to_laser',
                arguments=['0.08', '0', '0.65', '0', '0', '0', base_frame, laser_frame]
            ),
            # map to odom (ekf)
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[
                    '/home/common/ros2_ws/src/bot_navigation/param/ekf.yaml',
                    {
                        'map_frame': global_frame,
                        'odom_frame': odom_frame,
                        'base_link_frame': base_frame,
                        'world_frame': global_frame
                    }
                ],
                remappings=[('/set_pose', '/initialpose')]
            ),
            # lawnmower odometry
            Node(
                package='bot_navigation',
                executable='lawnmower_odometry',
                output='screen',
                parameters=[{
                    'odom_frame_id': odom_frame,
                    'base_frame_id': base_frame,
                    'print_tf': True,
                    'pub_odom_topic_name': 'odometry/wheel',
                    'sub_odom_topic_name': '/odom'
                }]
            )
        ])

    


    '''
    return LaunchDescription([
        # map_server
        Node(
            package='nav2_map_server',
            executable='map_server',
            output='screen',
            parameters=[{
                #'yaml_filename': '~/ros2_ws/src/bot_navigation/maps/map_square/map.yaml'
                'yaml_filename': '/home/common/ros2_ws/src/bot_navigation/maps/map_square/map.yaml'
                #'yaml_filename': 'maps/map_square/map.yaml'
            }]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server']
            }]
        ),
        # static_tf
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map',
            arguments=['0', '0', '0', '0', '0', '0', 'world_link', global_frame]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_measured',
            arguments=['0', '0', '0', '0', '0', '0', global_frame, 'base_measured']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='foot_to_gps',
            arguments=['0.08', '0', '0.75', '0', '0', '0', base_frame, gps_frame]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='foot_to_laser',
            arguments=['0.08', '0', '0.65', '0', '0', '0', base_frame, laser_frame]
        ),
        # map to odom (ekf)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                '/home/common/ros2_ws/src/bot_navigation/param/ekf.yaml',
                {
                    'map_frame': global_frame,
                    'odom_frame': odom_frame,
                    'base_link_frame': base_frame,
                    'world_frame': global_frame
                }
            ]
        ),
        # lawnmower odometry
        Node(
            package='bot_navigation',
            executable='lawnmower_odometry',
            output='screen',
            parameters=[{
                'odom_frame_id': odom_frame,
                'base_frame_id': base_frame,
                'print_tf': True,
                'pub_odom_topic_name': 'odometry/wheel',
                'sub_odom_topic_name': '/odom'
            }]
        ),
        # nav2
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[nav2_param_file]
            #remappings=remappings
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            #name='planner_server',
            output='screen',
            parameters=[nav2_param_file]
            #remappings=remappings
        ),
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            #name='recoveries_server',
            output='screen',
            parameters=[nav2_param_file],
            #remappings=remappings
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            #name='bt_navigator',
            output='screen',
            parameters=[nav2_param_file],
            #remappings=remappings
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            #name='waypoint_follower',
            output='screen',
            parameters=[nav2_param_file],
            #remappings=remappings
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'recoveries_server',
                    'bt_navigator',
                    'waypoint_follower'
                ]
            }]
        )
    ])
    '''