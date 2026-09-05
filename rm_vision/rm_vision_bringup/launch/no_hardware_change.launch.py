import os
import sys
from ament_index_python.packages import get_package_share_directory

sys.path.append(os.path.join(get_package_share_directory('rm_vision_bringup'), 'launch'))

def generate_launch_description():
    from common import launch_params, robot_state_publisher, node_params, tracker_node
    from launch_ros.actions import Node
    from launch.actions import ExecuteProcess, TimerAction
    from launch import LaunchDescription

    # TCP 图像收图节点：接收 Unity(Windows) TCP 直发的 raw RGB 帧，发布为 /image_raw + /camera_info。
    # 绕开 rosbridge JSON，支持 15fps+ 低延迟图像流（对应 Unity 侧 CameraToROS_TCP 组件）
    tcp_bridge = ExecuteProcess(
        cmd=['python3', os.path.join(get_package_share_directory('rm_vision_bringup'), 'scripts', 'tcp_image_bridge.py')],
        output='both',
    )

    # 你的原本 detector 启动逻辑（不变）
    detector_node = Node(
        package='armor_detector',
        executable='armor_detector_node',
        emulate_tty=True,
        output='both',
        parameters=[node_params],
        arguments=['--ros-args', '--log-level',
                   'armor_detector:=' + launch_params['detector_log_level']],
    )


    # 从第一个启动文件移植：伪造 TF (odom -> gimbal_link)
    # 用于闭合 odom -> gimbal_link -> camera_link -> camera_optical_frame 的 TF 链
    fake_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'gimbal_link'],
        output='screen'
    )

    # 从第一个启动文件移植：延时启动 tracker（等待 TF / detector 输出就绪）
    delay_tracker_node = TimerAction(
        period=2.0,
        actions=[tracker_node],
    )

    return LaunchDescription([
        robot_state_publisher,
        fake_tf_publisher,   # 新增：TF 链闭合
        tcp_bridge,          # 新增：Unity 图像 TCP 收图节点
        detector_node,
        delay_tracker_node,  # 新增：延迟启动 tracker
    ])
