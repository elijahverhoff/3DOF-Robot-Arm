from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('my_arm_description')
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'my_arm.urdf.xacro'])

    # Run xacro to generate the URDF XML
    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        xacro_file
    ])

    # Wrap as a string so YAML parsing doesn't choke
    robot_description = ParameterValue(robot_description_content, value_type=str)

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])