from launch import LaunchDescription

from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():


    resize_color_node = Node(
         package='image_proc',
         executable='resize_node',
         name='resize_node_color',
         output='screen',
                remappings=[
                    ('image/image_raw', '/color/image_raw'),
                    ('image/camera_info', '/color/camera_info'),
                    # ('/color/image_raw', 'image/image_raw'),
                    # ('/color/camera_info', 'image/camera_info'),
                    ('resize/image_raw', '/resize/image_raw'),
                    ('resize/camera_info', '/resize/camera_info'),
                ],
                parameters=[{
                    'use_scale ': False,
                    'height ': 480.0,
                    'width  ': 640.0,
                    'qos_overrides': {
                        '/resize/image_raw': {
                            'publisher': {
                                'reliability': 'best_effort',
                                'history': 'keep_last',
                                'depth': 10,
                            }
                        },
                                                
                        '/resize/camera_info': {
                            'publisher': {
                                'reliability': 'best_effort',
                                'history': 'keep_last',
                                'depth': 10,
                            }
                        }

                    },                    
                }],
    )

    depth_image_proc = Node(
         package='depth_image_proc',
         executable='resize_node',
         name='resize_node_color',
         output='screen',
                remappings=[
                    ('image/image_raw', '/color/image_raw'),
                    ('image/camera_info', '/color/camera_info'),
                    # ('/color/image_raw', 'image/image_raw'),
                    # ('/color/camera_info', 'image/camera_info'),
                    ('resize/image_raw', '/resize/image_raw'),
                    ('resize/camera_info', '/resize/camera_info'),
                ],
                parameters=[{
                    'use_scale ': False,
                    'height ': 480.0,
                    'width  ': 640.0,
                    'qos_overrides': {
                        '/resize/image_raw': {
                            'publisher': {
                                'reliability': 'best_effort',
                                'history': 'keep_last',
                                'depth': 10,
                            }
                        },
                                                
                        '/resize/camera_info': {
                            'publisher': {
                                'reliability': 'best_effort',
                                'history': 'keep_last',
                                'depth': 10,
                            }
                        }

                    },                    
                }],
    )

#     container = ComposableNodeContainer(
#         name='depth_image_proc_container',
#         package='rclcpp_components',
#         executable='component_container',
#                 namespace='',
#         composable_node_descriptions=[
#             ComposableNode(
#                 package='image_proc',
#                 plugin='image_proc::ResizeNode',
#                 name='resize_node_color',
#                 remappings=[
#                     ('/color/image_raw', '/image/image_raw'),
#                     ('/color/camera_info', '/image/camera_info'),
#                         # ('resized/image_raw', 'resized/image_raw'),
#                         # ('resized/camera_info', 'resized/camera_info'),
#                 ],
#                 parameters=[{'use_scale ': False,
#                     'height ': 480.0,
#                     'width  ': 640.0,
#             'qos_overrides./tf_static.publisher.durability': 'transient_local',                    
#                     }],
    
#             ),            ComposableNode(
#                 package='image_proc',
#                 plugin='image_proc::RectifyNode',
#                 name='rectify_node_color',
#                 remappings=[
#                     ('resized/image_raw', 'image'),
#                     ('resized/camera_info', 'camera_info' ),
#                     ('image_rect', 'color/image_rect')
#                 ],
#             ),
#             ComposableNode(
#                 package='image_proc',
#                 plugin='image_proc::RectifyNode',
#                 name='rectify_node_depth',
#                 remappings=[
#                     ('depth/image_raw', 'image' ),
#                     ('depth/camera_info', 'camera_info'),
#                     ('image_rect', 'depth/image_rect')
#                 ],
#             ),
#             ComposableNode(
#                 package='depth_image_proc',
#                 plugin='depth_image_proc::PointCloudXyzrgbNode',
#                 name='point_cloud_xyzrgb_node',
#                 remappings=[
#                     ('color/image_rect', 'rgb/image_rect_color'),
#                     ('depth/image_rect', 'depth_registered/image_rect'),
#                     ('rgb/camera_info', 'depth/camera_info'),
# #                    ('points', 'depth_registered/points'),
#                     #                     ('rgb/image_rect_color', 'rgb/image'),
#                     # ('depth_registered/image_rect', 'depth_registered/image'),
#                     ('points', 'depth_registered/points'),
#                 ],
#             ),
#         ]
#     )

    return LaunchDescription([resize_color_node])