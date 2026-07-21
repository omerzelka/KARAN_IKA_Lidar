# YDLIDAR T-mini Plus sürücüsünü parametreleriyle başlatan launch dosyası
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ydlidar_tmini_driver',
            executable='ydlidar_node',
            name='ydlidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'baud_rate': 230400,        # T-mini Plus varsayılanı
                'frame_id': 'laser_link',
                'range_min': 0.02,          # metre (T-mini Plus min menzil)
                'range_max': 12.0,          # metre (T-mini Plus nominal menzil)
                'num_bins': 360,            # 1 derece açısal çözünürlük
                # --- Donanımda doğrulanması gereken sabitler (bkz. ACIKLAMA) ---
                'sample_bytes': 2,          # 2 = mesafe-only; veri gelmezse 3 dene
                'distance_scale_mm': 0.25,  # ham -> mm (standart = 1/4)
                'angle_correction': False,  # ToF: kapalı (üçgenlemeli modellerde True)
                'verify_checksum': False,   # donanımda test edip True yapabilirsin
                'invert_angle': True,       # CW(lidar) -> CCW(REP-103)
                'use_dtr_motor': False,     # motor SCAN ile döner; gerekirse True
                'health_check': True,
            }],
        ),
    ])
