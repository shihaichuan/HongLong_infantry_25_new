from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os.path

def generate_launch_description():
    ld = LaunchDescription()

    params_file = os.path.join(get_package_share_directory('pose_solver'),'config','pose_params.yaml')

    use_sim_time = False

    pose_solver_left = Node(
        package="pose_solver",
        executable="pose_solver",
        name='pose_solver_left',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=
        [
          ('/Top_status' , "/left/Top_status"),
          ('/setGimbalAngle' , "/left/setGimbalAngle"),
          ('/shootSome' , "/left/shootSome")
        ],
        # output='screen'
    )

    pose_solver_right = Node(
        package="pose_solver",
        executable="pose_solver",
        name='pose_solver_right',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=
        [
          ('/Top_status' , "/right/Top_status"),
          ('/setGimbalAngle' , "/right/setGimbalAngle"),
          ('/shootSome' , "/right/shootSome")
        ],
        # output='screen'
    )

    ld.add_action(pose_solver_left)
    ld.add_action(pose_solver_right)
    return ld

