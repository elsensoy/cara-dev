 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    camera_id = LaunchConfiguration('camera_id')
    img_w = LaunchConfiguration('img_w')
    img_h = LaunchConfiguration('img_h')

    # Resolve model paths at launch-generation time (normal Python is fine here)
    share = get_package_share_directory('cara_vision_control')
    yunet = os.path.join(share, 'models', 'yunet.onnx')
    arc   = os.path.join(share, 'models', 'arcface.onnx')
    db    = os.path.expanduser('~/.cara/face_db.yaml')

    # Build /dev/videoX using a substitution (no .perform calls)
    video_dev = PythonExpression(["'/dev/video' + str(", camera_id, ")"])

    return LaunchDescription([
        DeclareLaunchArgument('camera_id', default_value='0'),
        DeclareLaunchArgument('img_w', default_value='640'),
        DeclareLaunchArgument('img_h', default_value='480'),

        # 1) Camera (headless-friendly)
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera',
            output='screen',
            parameters=[{
                'video_device': video_dev,
                 # THIS IS THE FIX
                'image_size': PythonExpression(['[', 'int(', img_w, '), ', 'int(', img_h, ')]']),
                'pixel_format': 'YUYV',
                'output_encoding': 'rgb8',
            }],
            # If you’re on SSH and don’t need GUI windows:
            additional_env={'QT_QPA_PLATFORM': 'offscreen', 'DISPLAY': ''},
        ),

        # 2) Face detection/recognition (ONNX paths live in cara_vision_control/share)
        Node(
            package='face_id_ros',
            executable='face_id_node',
            name='face_id_node',
            output='screen',
            parameters=[{
                'camera_topic': '/image_raw',
                'detector_model': yunet,
                'embedder_model': arc,
                'db_path': db,
                'det_thresh': 0.6,
                'recog_thresh': 0.40,
                'use_gpu': False,  # flip later when adding CUDA EP
            }],
        ),


        Node(
            package='cara_vision_control',
            executable='emotion_node',
            name='emotion_node',
            output='screen',
            parameters=[{
                'camera_id': camera_id,
                'use_gstreamer': True,
                'img_w': img_w,
                'img_h': img_h
            }]
        ),


        # Uncomment to drive servos via PCA9685
        Node(
            package='cara_vision_control',
            executable='servo_pca9685',
            name='servo_driver',
            output='screen',
            parameters=[{
                'i2c_addr': 0x40,
                'channels': [0, 1],
                'startup_deg': [90.0, 90.0],
                'neutral_deg: [90.0, 90.0],
                'min_deg': [30.0, 40.0],
                'max_deg': [150.0, 130.0],
            }],
            condition=IfCondition(use_servos),
        ),
    
