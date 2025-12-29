from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # If you want the models to live inside cara_vision_control:
    share = get_package_share_directory('cara_vision_control')
    yunet = os.path.join(share, 'models', 'yunet.onnx')
    arc   = os.path.join(share, 'models', 'arcface.onnx')
    db    = os.path.join(os.path.expanduser('~'), '.cara', 'face_db.yaml')

    face_node = Node(
        package='face_id_ros',
        executable='face_id_node',
        name='face_id_node',
        parameters=[{
            'camera_topic': '/image_raw',
            'detector_model': yunet,        # or "package://face_id_ros/models/yunet.onnx"
            'embedder_model': arc,
            'db_path': db,
            'det_thresh': 0.6,
            'recog_thresh': 0.40,
            'use_gpu': False,
        }],
        remappings=[('/camera/image', '/image_raw')],   
        output='screen',
    )

    return LaunchDescription([face_node])
