#!/usr/bin/env python3
# ============================================================================
# scan_serial_bridge.py  —  JETSON'da çalışır
#
# /scan (LaserScan) -> KOMPAKT string -> seri port (STM32'ye) @ 57600.
# STM32 bu baytları olduğu gibi PC'ye aktarır (dumb relay), PC haritalar.
#
# CÜMLE FORMATI (NMEA benzeri, string, checksum'lı):
#   $L,<seq>,<N>,<mm0>,<mm1>,...,<mm_{N-1}>*<XOR>\r\n
#       seq  : 0-65535 artan sayaç (tur kaybı tespiti için)
#       N    : nokta sayısı (seyreltilmiş)
#       mm_i : i. sektörün mesafesi, MİLİMETRE tamsayı; 0 = geçersiz/menzil dışı
#       XOR  : '$' ile '*' arasındaki karakterlerin XOR'u (NMEA usulü)
#   $P,<x_mm>,<y_mm>,<yaw_cdeg>*<XOR>\r\n   (opsiyonel, odom varsa; haritalama için)
#       yaw_cdeg = yaw derece * 100 (santiderece), tamsayı
#
# !!! 57600 BAUD DARBOĞAZI !!!
#   57600 8N1 ≈ 5760 byte/s. N=180 @ 6 Hz ≈ ~5 KB/s -> sığar (dar marj).
#   PC/STM32'de veri düşerse: num_points'i (N) düşür ya da publish_every'yi artır.
#
# Bağımlılık:  pip3 install pyserial
# Çalıştırma:
#   source ~/lidar/KARAN_IKA_Lidar/install/setup.bash
#   python3 scan_serial_bridge.py --ros-args \
#       -p port:=/dev/ttyTHS1 -p num_points:=180 -p odom_topic:=/odom
# ============================================================================

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

try:
    import serial  # pyserial
except ImportError:
    raise SystemExit("pyserial gerekli: pip3 install pyserial")

try:
    from nav_msgs.msg import Odometry
    HAVE_ODOM = True
except ImportError:
    HAVE_ODOM = False


def nmea_checksum(payload: str) -> int:
    """'$' ile '*' arasındaki gövdenin XOR checksum'ı."""
    c = 0
    for ch in payload:
        c ^= ord(ch)
    return c


class ScanSerialBridge(Node):
    def __init__(self):
        super().__init__('scan_serial_bridge')

        self.port_name  = str(self.declare_parameter('port', '/dev/ttyTHS1').value)
        self.baud       = int(self.declare_parameter('baud', 57600).value)
        self.num_points = int(self.declare_parameter('num_points', 180).value)   # seyreltme
        self.publish_every = int(self.declare_parameter('publish_every', 1).value)  # her k. tur
        self.odom_topic = str(self.declare_parameter('odom_topic', '').value)

        try:
            self.ser = serial.Serial(self.port_name, self.baud, timeout=0)
        except serial.SerialException as e:
            raise SystemExit(f"Seri port açılamadı ({self.port_name}): {e}")

        self.seq = 0
        self.scan_count = 0
        self.pose = None

        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        if self.odom_topic and HAVE_ODOM:
            self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
            self.get_logger().info(f"Odometri: {self.odom_topic}")

        self.get_logger().info(
            f"Seri köprü açık: {self.port_name} @ {self.baud}, N={self.num_points}, "
            f"her {self.publish_every}. tur")

    def on_odom(self, msg):
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (p.position.x, p.position.y, yaw)

    def _downsample(self, m: LaserScan):
        """Ham taramayı N sektöre indir: her sektörde EN YAKIN geçerli mesafe (mm)."""
        n = self.num_points
        out = [0] * n  # 0 = geçersiz
        two_pi = 2.0 * math.pi
        a = m.angle_min
        for r in m.ranges:
            if math.isfinite(r) and m.range_min <= r <= m.range_max:
                # açıyı [0,2pi) normalize et, sektöre ata
                ang = (a - m.angle_min) % two_pi
                k = int(ang / two_pi * n) % n
                mm = int(round(r * 1000.0))
                if out[k] == 0 or mm < out[k]:   # en yakın (min) tut
                    out[k] = mm
            a += m.angle_increment
        return out

    def _send(self, sentence_body: str):
        cs = nmea_checksum(sentence_body)
        line = f"${sentence_body}*{cs:02X}\r\n"
        try:
            self.ser.write(line.encode('ascii'))
        except serial.SerialException as e:
            self.get_logger().error(f"Seri yazma hatası: {e}")

    def on_scan(self, m):
        self.scan_count += 1
        if self.scan_count % self.publish_every != 0:
            return  # hız düşürme (57600'e sığdırmak için turların bir kısmını atla)

        # Poz cümlesi (varsa) — haritalama için turları dünya çerçevesinde birleştirir
        if self.pose is not None:
            x_mm  = int(round(self.pose[0] * 1000.0))
            y_mm  = int(round(self.pose[1] * 1000.0))
            yaw_c = int(round(math.degrees(self.pose[2]) * 100.0))
            self._send(f"P,{x_mm},{y_mm},{yaw_c}")

        # Lidar cümlesi
        dists = self._downsample(m)
        body = f"L,{self.seq},{len(dists)}," + ",".join(str(d) for d in dists)
        self._send(body)
        self.seq = (self.seq + 1) & 0xFFFF


def main():
    rclpy.init()
    node = ScanSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
