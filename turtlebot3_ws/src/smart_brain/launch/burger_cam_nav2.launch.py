import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. Get Directories
    smart_brain_dir = get_package_share_directory('smart_brain')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # 2. Define Input Files
    # We explicitly define the paths to prevent the system from guessing wrong
    map_file_path = os.path.join(smart_brain_dir, 'config', 'map.yaml')
    params_file_path = os.path.join(smart_brain_dir, 'config', 'burger_cam.yaml')

    # 3. Create Launch Configuration Variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_file = LaunchConfiguration('map', default=map_file_path)
    params_file = LaunchConfiguration('params_file', default=params_file_path)

    # 4. Define Actions
    
    # Action A: Launch Localization (Map Server + AMCL)
    localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items()
    )

    # Action B: Launch Navigation (Planners + Controllers + BT)
    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items()
    )

    # Action C: Launch RViz2 (Standard Nav2 Config)
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': '',
        }.items()
    )

    # 5. Create Launch Description
    ld = LaunchDescription()

    # Add Arguments
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))
    ld.add_action(DeclareLaunchArgument('map', default_value=map_file_path))
    ld.add_action(DeclareLaunchArgument('params_file', default_value=params_file_path))

    # Add Nodes
    ld.add_action(localization_cmd)
    ld.add_action(navigation_cmd)
    ld.add_action(rviz_cmd)

    return ld
