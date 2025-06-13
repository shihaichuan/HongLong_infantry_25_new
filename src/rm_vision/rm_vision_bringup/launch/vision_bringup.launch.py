import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable
from launch.substitutions import Command

sys.path.append(
    os.path.join(get_package_share_directory("rm_vision_bringup"), "launch")
)


def generate_launch_description():

    from common import (
        node_params,
        launch_params,
        robot_state_publisher,
        pose_solver_node,
        aimer_node,
    )
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node
    from launch.actions import TimerAction, Shutdown
    from launch import LaunchDescription

    def get_params(name):
        return os.path.join(
            get_package_share_directory("rm_vision_bringup"),
            "config",
            "node_params",
            "{}_params.yaml".format(name),
        )

    # 创建融合了相机和检测功能的节点
    detector_node = ComposableNode(
        package="armor_detector",
        plugin="rm_auto_aim::ArmorDetectorNode",
        name="armor_detector",
        parameters=[node_params],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    # 创建节点容器
    detector_container = ComposableNodeContainer(
        name="detector_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            detector_node
        ],
        output="both",
        emulate_tty=True,
        ros_arguments=[
            "--ros-args",
            "--log-level",
            "armor_detector:=" + launch_params["detector_log_level"],
        ],
        on_exit=Shutdown(),
    )

    serial_driver_node = Node(
        package="rm_serial_driver",
        executable="rm_serial_driver_node",
        name="serial_driver",
        output="both",
        emulate_tty=True,
        parameters=[node_params],
        on_exit=Shutdown(),
        ros_arguments=[],
    )

    delay_serial_node = TimerAction(
        period=1.5,
        actions=[serial_driver_node],
    )
    ######################
    delay_pose_solver = TimerAction(
        period=2.0,
        actions=[pose_solver_node],
    )
    delay_cam_detector = TimerAction(
        period=2.0,
        actions=[detector_container],
    )
    delay_aimer = TimerAction(
        period=2.0,
        actions=[aimer_node],
    )
    # set_env = SetEnvironmentVariable(
    #     name='ROS_LOCALHOST_ONLY',
    #     value='1'
    # )



    return LaunchDescription(
        [
            # set_env,
            robot_state_publisher,
            # # # cam_detector,
            # delay_serial_node,
            delay_cam_detector,
        #    delay_pose_solver,
        #     delay_aimer
        ]
    )
