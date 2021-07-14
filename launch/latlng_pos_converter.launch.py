
from launch import LaunchDescription
from launch_ros.actions import Node

#offset_param_file = '/home/common/ros2_ws/src/bot_navigation/param/gnss_offset.yaml'

origin_lat = 33.595649
origin_lng = 130.219383
angle_offset = 42.85
scale_offset = 1.0

fix_value = 3
float_value = 2

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bot_navigation',
            executable='latlng2pos',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'map_frame_name': 'map',
                'odom_frame_name': 'odom',

                'visualization_marker': False,
                'marker_topic_name': 'gnss_position_marker',

                'fix_value': fix_value,
                'float_value': float_value,

                'origin_lat': origin_lat,
                'origin_lng': origin_lng,
                'angle_offset': angle_offset,
                'scale_offset': scale_offset,

                'solutions_flag': 'all',
                
                'nav_sat_fix_topic_name': 'fix',
                'odom_topic_name': 'odometry/gnss'
            }]
        ),
        Node(
            package='bot_navigation',
            executable='pos2latlng',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'origin_lat': origin_lat,
                'origin_lng': origin_lng,
                'angle_offset': angle_offset,
                'scale_offset': scale_offset,

                'pub_topic_name': 'filtered_latlng',
                'sub_topic_name': 'odometry/filtered',
            }]
        )
    ])
