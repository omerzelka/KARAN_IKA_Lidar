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
                'range_min': 0.05,          # metre; çok yakın parazit okumaları ele
                                            # (0.05 yetmezse 0.10'a çıkar)
                'range_max': 12.0,          # metre (T-mini Plus nominal menzil)
                'num_bins': 360,            # 1 derece açısal çözünürlük
                # --- Donanımda DOĞRULANDI (lidar_probe.py, bkz. ACIKLAMA) ---
                'sample_bytes': 3,          # T-mini Plus intensity'li: 3 bayt/örnek
                                            # (probe: 385/385 paket 3B ölçüldü)
                'distance_scale_mm': 0.25,  # ham -> mm = 1/4 (probe ile doğrulandı)
                'angle_correction': False,  # ToF: kapalı (üçgenlemeli modellerde True)
                'verify_checksum': True,    # intensity-dahil XOR %100 eşleşti;
                                            # bozuk paketler artık otomatik atılır
                'invert_angle': True,       # CW(lidar) -> CCW(REP-103)
                'use_dtr_motor': False,     # motor SCAN ile döner; gerekirse True
                'health_check': True,
            }],
        ),
    ])
