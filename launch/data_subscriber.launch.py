import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node
        (
            package='data_subscriber',
            executable='data_subscriber_node',
            name='data_subscriber_node',
            remappings=[('/image','/camera/color/image_raw'),
                        ('/depth','/camera/aligned_depth_to_color/image_raw'),
                        ('/camera_info', '/camera/color/camera_info'),
                        ('/laser', '/processed_scan')],
            # remappings=[('/image','/rgb_image_sync'),
            #             ('/depth','/estimated_depth'),
            #             ('/camera_info', '/camera/color/camera_info'),
            #             ('/scan', '/scan')]        
        )
    ])