# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""This is all-in-one launch script intended for use by nav2 developers."""

import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


#region Start Simulation

def launch_setup(context, *args, **kwargs):
    """
    The Method is starting the simulation. This is for gazebo harmonic and higher.
    This is implemented as OpaqueFunction to process and control the simulation start. 
    """
    world = LaunchConfiguration("world").perform(context)
    headless = LaunchConfiguration("headless").perform(context)
    gz_args = f"--headless-rendering -s -v 4 -r {world}" if eval(headless) else f"-r {world}"
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            )
        ),
        launch_arguments={
            "gz_args": gz_args,
            "on_exit_shutdown": "True",
        }.items(),
    )
    return [gz_sim]

#endregion

#region Convert XACRO file

def evaluate_xacro(context, *args, **kwargs):
    """
    Evaluates LaunchConfigurations in context for use with xacro.process_file(). Returns a list of launch actions to be
     ncluded in launch description    
    Method is converting the XACRO  description into a URDF file format. 
    XARCO Allows some paramters and programming in the robot desccription
    This is implemented as OpaqueFunction to process description ad publish it. 
    """

    # Get possible paramter driving the xacro description from context.
    mecanum = LaunchConfiguration('mecanum').perform(context)
    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory('master3_description'), 'urdf', 'yahboomcar_X3.urdf.xacro')

    #robot_description_config = xacro.process_file(xacro_file)
    robot_description_config = xacro.process_file(xacro_file, 
            mappings={  
                "mecanum": mecanum
                }).toxml()

    robot_state_publisher_node = Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      name='robot_state_publisher',
        output='both',
      parameters=[{
        'robot_description': robot_description_config
      }])

    return [robot_state_publisher_node]

#endregion

def generate_launch_description():
#region  Get the launch directories
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    pkg_home = get_package_share_directory('master3_nav2')
    pkg_bringup = get_package_share_directory('master3_bringup')
    world_package = get_package_share_directory("aws_robomaker_small_house_world")
    world_file = PathJoinSubstitution([world_package, "worlds", "small_house.world"])
#endregion 

#region  Create the launch configuration variables
    slam = LaunchConfiguration('slam')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    # args that can be set from the command line or a default will be used
    mecanum_launch_value = LaunchConfiguration('mecanum')
#endregion

#region  Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    pose = {'x': LaunchConfiguration('x_pose', default='-2.00'),
            'y': LaunchConfiguration('y_pose', default='-0.50'),
            'z': LaunchConfiguration('z_pose', default='0.01'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}
    # robot_name = LaunchConfiguration('robot_name')
    # robot_sdf = LaunchConfiguration('robot_sdf')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    # remappings = [('/tf', 'tf'),
    #               ('/tf_static', 'tf_static')]
#endregion

#region  Declare the launch arguments

    declare_mecanum_cmd = DeclareLaunchArgument(
        "mecanum", 
        default_value="True", 
        description="Flag indicating to use mecanum wheels; skid drive otherwise")

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            world_package, 'maps', 'map.yaml'),
        description='Full path to map file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # declare_params_file_cmd = DeclareLaunchArgument(
    #     'params_file',
    #     default_value=os.path.join(pkg_home, 'config', 'nav2_params-DWBLocalPlanner.yaml'),
    #     description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_home, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    # declare_params_file_cmd = DeclareLaunchArgument(
    #     'params_file',
    #     default_value=os.path.join(pkg_home, 'config', 'nav2_params-SmacStateLattice.yaml'),
    #     description='Full path to the ROS2 parameters file to use for all launched nodes')




    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(pkg_home, 'config', 'nav2_default_view.rviz') ,
        description='Full path to the RVIZ config file to use')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='True',
        description='Whether to execute gazebo Client UI)')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Full path to world model file to load')

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(world_package, 'maps', 'map.yaml'),
        description='Full path to the ROS2 map file to use for navigation')


    # declare_robot_name_cmd = DeclareLaunchArgument(
    #     'robot_name',
    #     default_value='turtlebot3_waffle',
    #     description='name of the robot')

    # declare_robot_sdf_cmd = DeclareLaunchArgument(
    #     'robot_sdf',
    #     default_value=os.path.join(bringup_dir, 'worlds', 'waffle.model'),
    #     description='Full path to robot sdf file to spawn the robot in gazebo')

#endregion


    # Specify the actions
    # start_gazebo_server_cmd = ExecuteProcess(
    #     condition=IfCondition(use_simulator),
    #     cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
    #          '-s', 'libgazebo_ros_factory.so', world],
    #     cwd=[launch_dir], output='screen')

    # start_gazebo_client_cmd = ExecuteProcess(
    #     condition=IfCondition(PythonExpression(
    #         [use_simulator, ' and not ', headless])),
    #     cmd=['gzclient'],
    #     cwd=[launch_dir], output='screen')

    # urdf = os.path.join(bringup_dir, 'urdf', 'turtlebot3_waffle.urdf')
    # with open(urdf, 'r') as infp:
    #     robot_description = infp.read()

    # start_robot_state_publisher_cmd = Node(
    #     condition=IfCondition(use_robot_state_pub),
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     namespace=namespace,
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time,
    #                  'robot_description': robot_description}],
    #     remappings=remappings)

    # start_gazebo_spawner_cmd = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     output='screen',
    #     arguments=[
    #         '-entity', robot_name,
    #         '-file', robot_sdf,
    #         '-robot_namespace', namespace,
    #         '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
    #         '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']])

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_bringup, 'config', 'master3_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )


    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "master3_drive",
            "-allow_renaming",
            "false",
            "-topic",
            "robot_description",
            "-x",
            LaunchConfiguration("x", default="0.00"),
            "-y",
            LaunchConfiguration("y", default="0.00"),
            "-z",
            LaunchConfiguration("z", default="0.00"),
            "-R",
            LaunchConfiguration("roll", default="0.00"),
            "-P",
            LaunchConfiguration("pitch", default="0.00"),
            "-Y",
            LaunchConfiguration("yaw", default="0.00"),
        ],
        output="screen",
        namespace="",
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'rviz_config': rviz_config_file}.items())


    robot_localization_cmd = Node(
         package='robot_localization',
         executable='ekf_node',
         name='ekf_filter_node',
         output='screen',
         parameters=[os.path.join(pkg_home, 'config/ekf.yaml'), {'use_sim_time': use_sim_time}]
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'slam': slam,
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'autostart': autostart,
                          'use_composition': use_composition,
                          'use_respawn': use_respawn}.items())

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_mecanum_cmd)

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_map_cmd)
            

    # ld.add_action(declare_robot_name_cmd)
    # ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(declare_use_respawn_cmd)

    # Add any simulation  actions
    ld.add_action(OpaqueFunction(function=evaluate_xacro))
    ld.add_action(OpaqueFunction(function=launch_setup))
    ld.add_action(gz_spawn_entity)
    ld.add_action(bridge_cmd)
    # ld.add_action(start_gazebo_server_cmd)

    # Add the actions to launch all of the navigation nodes
    # ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(robot_localization_cmd)
    ld.add_action(bringup_cmd)
    

    return ld
