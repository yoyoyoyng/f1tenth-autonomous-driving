from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'yaml_filename': map_yaml}],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_min',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl', 'planner_server']
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', description='Full path to map yaml'),
        DeclareLaunchArgument('params_file', description='Full path to Nav2 params yaml'),
        map_server,
        amcl,
        planner_server,
        lifecycle_manager,
    ])
    
    
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node


# def generate_launch_description():
#     use_sim_time = LaunchConfiguration('use_sim_time')
#     map_yaml = LaunchConfiguration('map')
#     params_file = LaunchConfiguration('params_file')

#     map_server = Node(
#         package='nav2_map_server',
#         executable='map_server',
#         name='map_server',
#         output='screen',
#         parameters=[{'use_sim_time': use_sim_time},
#                     {'yaml_filename': map_yaml}],
#     )

#     amcl = Node(
#         package='nav2_amcl',
#         executable='amcl',
#         name='amcl',
#         output='screen',
#         parameters=[params_file, {'use_sim_time': use_sim_time}],
#     )

#     # ⭐ Global Costmap 노드 추가!
#     costmap_node = Node(
#         package='nav2_costmap_2d',
#         executable='nav2_costmap_2d',
#         name='global_costmap',
#         output='screen',
#         parameters=[params_file, {'use_sim_time': use_sim_time}],
#         remappings=[
#             ('costmap', 'global_costmap/costmap'),
#             ('costmap_updates', 'global_costmap/costmap_updates'),
#         ]
#     )

#     lifecycle_manager = Node(
#         package='nav2_lifecycle_manager',
#         executable='lifecycle_manager',
#         name='lifecycle_manager_min',
#         output='screen',
#         parameters=[{
#             'use_sim_time': use_sim_time,
#             'autostart': True,
#             'node_names': ['map_server', 'amcl', 'global_costmap']  # ⭐ 추가
#         }],
#     )

#     return LaunchDescription([
#         DeclareLaunchArgument('use_sim_time', default_value='false'),
#         DeclareLaunchArgument('map', description='Full path to map yaml'),
#         DeclareLaunchArgument('params_file', description='Full path to Nav2 params yaml'),
#         map_server,
#         amcl,
#         costmap_node,  # ⭐ 추가
#         lifecycle_manager,
#     ])
