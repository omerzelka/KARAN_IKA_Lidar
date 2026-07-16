# RPLIDAR A2M12 sürücüsünü parametreleriyle başlatan launch dosyası
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rplidar_a2m12_driver',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'baud_rate': 256000,      # A2M12 varsayılanı; A2M8 için 115200
                'frame_id': 'laser_link',
                'range_min': 0.15,        # metre
                'range_max': 12.0,        # metre (A2M12 nominal menzili)
                'num_bins': 720,          # 0.5 derece açısal çözünürlük
            }],
        ),
    ])
