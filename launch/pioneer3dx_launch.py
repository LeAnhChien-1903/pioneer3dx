import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.utils import controller_url_prefix

def generate_launch_description():
    package_dir = get_package_share_directory('pioneer3dx')

    robot_description = pathlib.Path(os.path.join(package_dir,'resource', 'pioneer3dx_webots.urdf')).read_text()

    webots = WebotsLauncher(world = os.path.join(package_dir, 'worlds', 'pioneer3dx_world.wbt'))

    ros2_supervisor = Ros2SupervisorLauncher()

    default_model_path = os.path.join(package_dir, "models", "pioneer3dx.urdf")
    default_rviz_config_path = os.path.join(package_dir, "rviz", "urdf_config.rviz")
    robot_localization_file_path = os.path.join(package_dir, "config", "ekf.yaml")
     # Launch configuration variables specific to simulation
    gui = LaunchConfiguration('gui')
    model = LaunchConfiguration('model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
     # Declare the launch arguments  
    declare_model_path_cmd = DeclareLaunchArgument(
        name='model', 
        default_value=default_model_path, 
        description='Absolute path to robot urdf file')
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')
    
    declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
        name='gui',
        default_value='True',
        description='Flag to enable joint_state_publisher_gui')
  
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RVIZ')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
     # Publish the joint state values for the non-fixed joints in the URDF file.
    start_joint_state_publisher_cmd = Node(
        condition=UnlessCondition(gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher')

    # A GUI to manipulate the joint state values
    start_joint_state_publisher_gui_node = Node(
        condition=IfCondition(gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui')

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 
        'robot_description': Command(['xacro ', model])}],
        arguments=[default_model_path])
    # Start robot localization using an Extended Kalman filter
    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file_path, 
        {'use_sim_time': use_sim_time}])
    # Launch RViz
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file])
    controller = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env = {'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'pioneer3dx'},
        parameters=[
            {'robot_description': robot_description},
        ] 
    )
    launch_cmd = launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ld = LaunchDescription()
    ld.add_action(webots)
    ld.add_action(controller)
    ld.add_action(ros2_supervisor)
    ld.add_action(launch_cmd)
    # Declare the launch options
    ld.add_action(declare_model_path_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_joint_state_publisher_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)  
    ld.add_action(declare_use_rviz_cmd) 
    ld.add_action(declare_use_sim_time_cmd)

    # Add any actions
    ld.add_action(start_robot_localization_cmd)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_gui_node)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)

    return ld