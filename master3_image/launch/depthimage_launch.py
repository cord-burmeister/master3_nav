from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer , Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    container = ComposableNodeContainer(
        name='depth_image_proc_container',
        package='rclcpp_components',
        executable='component_container',
                namespace='',
        composable_node_descriptions=[
            ComposableNode(
                package='image_proc',
                plugin='image_proc::ResizeNode',
                name='resize_node_color',
                remappings=[
                    ('image/image_raw', '/color/opt/image_raw'),
                    ('image/camera_info', '/color/opt/camera_info'),
                    ('resize/image_raw', '/resize/opt/image_raw'),
                    ('resize/camera_info', '/resize/opt/camera_info'),
                ],
                parameters=[{
                        'use_scale': False,
                        'height': 480,
                        'width': 640,
                        'scale_height': 2.0,
                        'scale_width': 2.0,
                        'qos_overrides': {
                            '/resize/opt/image_raw': {
                                'publisher': {
                                    'reliability': 'best_effort',
                                    'history': 'keep_last',
                                    'depth': 10,
                                }
                            },
                            # '/resize/opt/camera_info': {
                            #     'publisher': {
                            #         'reliability': 'best_effort',
                            #         'history': 'keep_last',
                            #         'depth': 10,
                            #     }
                            # }
                        },                    
                    }],
    
            ),
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_node_color',
                remappings=[
                    ('image', '/resize/opt/image_raw'),
                    ('camera_info', '/resize/opt/camera_info' ),
                    ('image_rect', '/color_rect/opt/image_rect')
                ],
                parameters=[{
                        'qos_overrides': {
                            '/resize/opt/image_raw': {
                                '   ': {
                                    'reliability': 'best_effort',
                                    'history': 'keep_last',
                                    'depth': 10,
                                }
                            },
                            # '/resize/opt/camera_info': {
                            #     'publisher': {
                            #         'reliability': 'best_effort',
                            #         'history': 'keep_last',
                            #         'depth': 10,
                            #     }
                            # },
                            '/color_rect/opt/image_rect': {
                                'publisher': {
                                    'reliability': 'best_effort',
                                    'history': 'keep_last',
                                    'depth': 10,
                                }
                            }
                        },
                }
                ]                
            ),
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_node_depth',
                remappings=[
                    ('image', '/depth/opt/image_raw' ),
                    ('camera_info', '/depth/opt/camera_info'),
                    ('image_rect', '/depth_rect/opt/image_rect')
                ],
                parameters=[{
                        'qos_overrides': {
                            '/depth/opt/image_raw': {
                                'publisher': {
                                    'reliability': 'best_effort',
                                    'history': 'keep_last',
                                    'depth': 10,
                                }
                            },
                            # '/depth/opt/camera_info': {
                            #     'publisher': {
                            #         'reliability': 'best_effort',
                            #         'history': 'keep_last',
                            #         'depth': 10,
                            #     }
                            # },
                            '/depth_rect/opt/image_rect': {
                                'publisher': {
                                    'reliability': 'best_effort',
                                    'history': 'keep_last',
                                    'depth': 10,
                                }
                            }
                        },
                }
                ]                

            ),
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzrgbNode',
                name='point_cloud_xyzrgb_node',
                remappings=[
                    ('rgb/camera_info', '/resize/opt/camera_info'),
                    ('rgb/image_rect_color', '/color_rect/opt/image_rect'),
                    ('depth_registered/image_rect', '/depth_rect/opt/image_rect'),
                    ('points', '/depth_registered/points')],
                    parameters=[{
                        'qos_overrides': {
                            # '/resize/opt/camera_info': {
                            #     'subscription': {
                            #         'reliability': 'best_effort',
                            #         'history': 'keep_last',
                            #         'depth': 10,
                            #     }
                            # },                            
                            '/color_rect/opt/image_rect': {
                                'publisher': {
                                    'reliability': 'best_effort',
                                    'history': 'keep_last',
                                    'depth': 10,
                                }
                            },
                            '/depth_rect/opt/image_rect': {
                                'publisher': {
                                    'reliability': 'best_effort',
                                    'history': 'keep_last',
                                    'depth': 10,
                                }
                            },
                            '/depth_registered/points': {
                                'publisher': {
                                    'reliability': 'best_effort',
                                    'history': 'keep_last',
                                    'depth': 10,
                                }
                            }
                        },
                }
                ]                                                    
#                 remappings=[
#                     ('color/image_rect', 'rgb/image_rect_color'),
#                     ('depth/image_rect', 'depth_registered/image_rect'),
#                     ('rgb/camera_info', 'depth/camera_info'),
# #                    ('points', 'depth_registered/points'),
#                     #                     ('rgb/image_rect_color', 'rgb/image'),
#                     # ('depth_registered/image_rect', 'depth_registered/image'),
#                     ('points', 'depth_registered/points'),
#                 ],
            ),
        ]
    )

   
    # rviz_node = Node(
    #         package='rviz2', node_executable='rviz2', output='screen',
    #         arguments=['--display-config', default_rviz]),

    return LaunchDescription([container])