import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.actions import Node

launch_params = yaml.safe_load(open(os.path.join(
    get_package_share_directory('rm_vision_bringup'), 'config', 'launch_params.yaml')))

robot_description = Command(['xacro ', os.path.join(
    get_package_share_directory('rm_gimbal_description'), 'urdf', 'rm_gimbal.urdf.xacro'),
    ' xyz:=', launch_params['base_link2camera']['xyz'], ' rpy:=', launch_params['base_link2camera']['rpy']])

robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': robot_description,
                 'publish_frequency': 1000.0}]
)

node_params = os.path.join(
    get_package_share_directory('rm_vision_bringup'), 'config', 'node_params.yaml')

pose_solver_node = Node(
        package='pose_solver',
        executable='pose_solver_node',
        name='pose_solver',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
        ros_arguments=['--log-level', 'pose_solver:='+launch_params['tracker_log_level']],
        )
aimer_node = Node(
        package='aimer',
        executable='aimer_node',
        name='rm_aimer',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
        ros_arguments=['--log-level', 'rm_aimer:='+launch_params['tracker_log_level']],
        )
