# cara_stack.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
# run it with ros2 launch cara_bringup cara_stack.launch.py use_servos:=true

def generate_launch_description():
    use_servos = LaunchConfiguration("use_servos")
    camera_id  = LaunchConfiguration("camera_id")
    img_w      = LaunchConfiguration("img_w")
    img_h      = LaunchConfiguration("img_h")

    gaze_pkg_share = get_package_share_directory("cara_gaze_control")
    gaze_launch = os.path.join(gaze_pkg_share, "launch", "model_gaze.launch.py")

    return LaunchDescription([
        DeclareLaunchArgument("use_servos", default_value="false"),
        DeclareLaunchArgument("camera_id", default_value="0"),
        DeclareLaunchArgument("img_w", default_value="640"),
        DeclareLaunchArgument("img_h", default_value="480"),

        # 1) Include your existing camera+yunet+gaze pipeline
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gaze_launch),
            launch_arguments={
                "use_servos": use_servos,
                "camera_id": camera_id,
                "img_w": img_w,
                "img_h": img_h,
            }.items(),
        ),

        # 2) Emotion node (publishes /cara/emotion, subscribes /cara/feedback)
        Node(
            package="cara_vision_control",
            executable="emotion_node",
            name="emotion_node",
            output="screen",
        ),

        # 3) (Optional, later) Head/Ears expression node
        # Node(
        #     package="cara_motion_control",
        #     executable="head_expression_node",
        #     name="head_expression_node",
        #     output="screen",
        #     parameters=[{"serial_port": "/dev/ttyAMA0"}],
        # ),

        # 4) (Optional, later) IMU node
        # Node(
        #     package="cara_motion_control",
        #     executable="imu_node",
        #     name="imu_node",
        #     output="screen",
        # ),
    ])

