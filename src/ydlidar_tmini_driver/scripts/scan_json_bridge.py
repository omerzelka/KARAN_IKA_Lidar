#!/usr/bin/env python3
# ============================================================================
# scan_json_bridge.py  —  JETSON'da çalışır
#
# /scan (sensor_msgs/LaserScan) -> JSON satırları -> TCP ile uzaktaki bilgisayara.
# Her tam tur (360°) TEK satır JSON olarak gönderilir (JSON Lines / NDJSON):
#   {"stamp":..., "angle_min":..., "angle_increment":..., "ranges":[...], "pose":{...}}\n
#
# Tasarım:
#   - Jetson TCP SUNUCU'dur (0.0.0.0:<port> dinler); uzaktaki bilgisayar
#     istemci olarak bağlanır. Birden çok izleyici aynı anda bağlanabilir.
#   - Yayın Best-Effort olduğu için abonelik qos_profile_sensor_data ile açılır.
#   - inf/nan -> null (JSON Infinity kabul etmez).
#   - Opsiyonel /odom: her tura pose{x,y,yaw} eklenir (gerçek haritalama için).
#
# Çalıştırma (Jetson):
#   source ~/lidar/KARAN_IKA_Lidar/install/setup.bash
#   python3 scan_json_bridge.py
#   # parametreyle:
#   python3 scan_json_bridge.py --ros-args -p port:=9000 -p odom_topic:=/odom
# ============================================================================

import json
import math
import socket
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

# nav_msgs her sistemde kurulu olmayabilir; odom kapalıysa gerekmez
try:
    from nav_msgs.msg import Odometry
    HAVE_ODOM = True
except ImportError:
    HAVE_ODOM = False


def clean(v):
    """JSON Infinity/NaN kabul etmez -> geçersiz değerleri null (None) yap."""
    return v if math.isfinite(v) else None


class ScanJsonBridge(Node):
    def __init__(self):
        super().__init__('scan_json_bridge')

        self.port             = int(self.declare_parameter('port', 9000).value)
        self.send_intensities = bool(self.declare_parameter('send_intensities', False).value)
        # '' => odom kapalı. Haritalama için '/odom' ver.
        self.odom_topic       = str(self.declare_parameter('odom_topic', '').value)

        # DİKKAT: rclpy.Node'un kendi 'self._clients' özelliği var (ROS servis
        # client listesi). Aynı ismi kullanırsak executor soketi ROS client
        # sanıp çöker -> ayrı isim kullan.
        self._conns = []
        self._lock = threading.Lock()
        self._pose = None  # en son {x, y, yaw}

        # /scan — DİKKAT: sürücü Best-Effort yayınlar, QoS uyumlu olmalı
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)

        if self.odom_topic:
            if HAVE_ODOM:
                self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
                self.get_logger().info(f"Odometri dinleniyor: {self.odom_topic}")
            else:
                self.get_logger().warn("nav_msgs yok; odom_topic yok sayıldı")

        threading.Thread(target=self._serve, daemon=True).start()
        self.get_logger().info(f"JSON köprüsü 0.0.0.0:{self.port} portunda dinliyor")

    # --- TCP sunucu: arka planda istemci bağlantılarını kabul eder ---
    def _serve(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(('0.0.0.0', self.port))
        srv.listen(5)
        while rclpy.ok():
            try:
                conn, addr = srv.accept()
            except OSError:
                break
            conn.settimeout(2.0)  # yavaş istemci tüm akışı kilitlemesin
            with self._lock:
                self._conns.append(conn)
            self.get_logger().info(f"İstemci bağlandı: {addr[0]}:{addr[1]}")

    def on_odom(self, msg):
        p = msg.pose.pose
        q = p.orientation
        # Quaternion -> yaw (2B düzlemde yeterli)
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._pose = {'x': p.position.x, 'y': p.position.y, 'yaw': yaw}

    def on_scan(self, m):
        rec = {
            'stamp':           m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
            'frame_id':        m.header.frame_id,
            'angle_min':       m.angle_min,
            'angle_max':       m.angle_max,
            'angle_increment': m.angle_increment,
            'range_min':       m.range_min,
            'range_max':       m.range_max,
            'scan_time':       m.scan_time,
            'ranges':          [clean(r) for r in m.ranges],
        }
        if self.send_intensities and m.intensities:
            rec['intensities'] = [clean(i) for i in m.intensities]
        if self._pose is not None:
            rec['pose'] = self._pose

        line = (json.dumps(rec, separators=(',', ':')) + '\n').encode('utf-8')
        self._broadcast(line)

    def _broadcast(self, data):
        dead = []
        with self._lock:
            for c in self._conns:
                try:
                    c.sendall(data)
                except OSError:
                    dead.append(c)
            for c in dead:
                self._conns.remove(c)
                try:
                    c.close()
                except OSError:
                    pass
        if dead:
            self.get_logger().warn(f"{len(dead)} istemci düştü, akıştan çıkarıldı")


def main():
    rclpy.init()
    node = ScanJsonBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
