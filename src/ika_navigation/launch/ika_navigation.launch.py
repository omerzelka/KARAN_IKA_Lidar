# İKA navigasyon düğümünü (Stanley + VFH+ + Kalman) parametreleriyle başlatır.
# Not: Lidar sürücüsü ayrıca çalışıyor olmalı:
#   ros2 launch rplidar_a2m12_driver rplidar.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ika_navigation',
            executable='ika_navigation_node',
            name='ika_navigation',
            output='screen',
            parameters=[{
                # --- Genel hareket sınırları ---
                'max_linear_speed': 0.6,      # m/s
                'max_angular_speed': 1.2,     # rad/s
                'slow_down_dist': 1.5,        # m — engel bu mesafede yavaşla
                'stop_dist': 0.5,             # m — bu mesafede tam dur
                'scan_angle_offset': 0.0,     # lidar montaj açısı düzeltmesi (rad)
                'cluster_threshold': 0.3,     # engel kümeleme eşiği (m)
                'angular_gain': 1.5,          # direksiyon -> açısal hız kazancı
                'odom_frame': 'odom',

                # --- Referans yol: [x0,y0, x1,y1, ...] (odom çerçevesi, metre) ---
                # Boş bırakılırsa yalnızca engelden kaçınma (VFH+) çalışır.
                'waypoints': [0.0, 0.0,  3.0, 0.0,  3.0, 3.0,  0.0, 3.0],

                # --- Stanley Controller ---
                'stanley.k': 1.5,             # çapraz hata kazancı
                'stanley.k_soft': 0.5,        # düşük hız yumuşatma (m/s)
                'stanley.max_steer': 1.0,     # direksiyon sınırı (rad)
                'stanley.wheel_base': 0.5,    # sanal aks mesafesi (m)
                'stanley.goal_tolerance': 0.3,

                # --- VFH+ (engelden kaçınma) ---
                'vfh.num_sectors': 72,        # 5° çözünürlük
                'vfh.robot_radius': 0.35,     # tank yarıçapı (m)
                'vfh.safety_distance': 0.25,  # güvenlik payı (m)
                'vfh.max_range': 4.0,         # dikkate alınan azami engel mesafesi
                'vfh.threshold_low': 3.0,
                'vfh.threshold_high': 8.0,
                'vfh.mu1_target': 5.0,        # hedefe yönelim ağırlığı
                'vfh.mu2_current': 2.0,
                'vfh.mu3_previous': 2.0,

                # --- Kalman engel takibi ---
                'tracker.process_noise': 1.0,
                'tracker.measurement_noise': 0.05,
                'tracker.association_dist': 0.6,   # gözlem-iz eşleştirme (m)
                'tracker.max_misses': 8,
            }],
            # Lidar 'scan' ve aracın 'odom' topic'lerini burada yeniden eşleyebilirsiniz:
            # remappings=[('scan', '/scan'), ('odom', '/odom'), ('cmd_vel', '/cmd_vel')],
        ),
    ])
