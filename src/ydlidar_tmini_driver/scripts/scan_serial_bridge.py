#!/usr/bin/env python3
# ============================================================================
# scan_serial_bridge.py  —  JETSON'da çalışır
#
# /scan (LaserScan) -> KOMPAKT BİNARY çerçeve -> seri port (STM32'ye) @ 57600.
# STM32 bu baytları olduğu gibi PC'ye aktarır (dumb relay), PC haritalar.
#
# BİNARY ÇERÇEVE (little-endian):
#   [0xAA 0x55] [type:u8] [len:u16] [payload:len bayt] [crc:u16]
#     sync word                                          CRC-16/CCITT-FALSE
#     type : 'L'(0x4C)=lidar, 'P'(0x50)=poz
#     len  : payload uzunluğu (bayt)
#     crc  : type + len + payload üstünde (init=0xFFFF, poly=0x1021)
#   L payload: seq(u16) + N(u16) + N × u16   (mesafe mm, 0 = geçersiz/menzil dışı)
#   P payload: x_mm(i32) + y_mm(i32) + yaw_cdeg(i32)   (santiderece = derece*100)
#
#   NEDEN BİNARY? Eski ASCII format ~767 B/tur (57600 hattının %80'i, dar marj).
#   Binary u16 ~360 B/tur (%38) — mesafeyi 5 haneli metin yerine 2 bayta koyar,
#   bilgi kaybı yok, N=180 @ 6 Hz artık rahat sığar. Baytlar '$','\r','\n'
#   içerebildiği için satır çerçevelemesi yerine sync+len+CRC kullanılır; STM32
#   relay byte-transparan olduğundan sorun olmaz.
#
# !!! 57600 BAUD BÜTÇESİ !!!
#   57600 8N1 ≈ 5760 byte/s. Binary N=180 @ 6 Hz ≈ 2160 B/s -> bol pay.
#   Yine de düşerse: num_points (N) düşür ya da publish_every artır.
#
# Bağımlılık:  pip3 install pyserial
# Çalıştırma:
#   source ~/lidar/KARAN_IKA_Lidar/install/setup.bash
#   python3 scan_serial_bridge.py --ros-args \
#       -p port:=/dev/ttyTHS1 -p num_points:=180 -p odom_topic:=/odom
# ============================================================================

import math
import struct

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

SYNC = b'\xAA\x55'
TYPE_L = 0x4C  # ord('L')
TYPE_P = 0x50  # ord('P')


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE (init=0xFFFF, poly=0x1021, yansıtma yok)."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def build_frame(type_byte: int, payload: bytes) -> bytes:
    """[AA 55][type][len:u16][payload][crc:u16] çerçevesini üretir."""
    body = struct.pack('<BH', type_byte, len(payload)) + payload
    return SYNC + body + struct.pack('<H', crc16_ccitt(body))


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
            f"Seri köprü açık (binary): {self.port_name} @ {self.baud}, "
            f"N={self.num_points}, her {self.publish_every}. tur")

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
                if mm > 65535:
                    mm = 65535                    # u16 taşmasına karşı kırp
                if out[k] == 0 or mm < out[k]:    # en yakın (min) tut
                    out[k] = mm
            a += m.angle_increment
        return out

    def _write(self, frame: bytes):
        try:
            self.ser.write(frame)
        except serial.SerialException as e:
            self.get_logger().error(f"Seri yazma hatası: {e}")

    def on_scan(self, m):
        self.scan_count += 1
        if self.scan_count % self.publish_every != 0:
            return  # hız düşürme (57600'e sığdırmak için turların bir kısmını atla)

        # Poz çerçevesi (varsa) — haritalama için turları dünya çerçevesinde birleştirir
        if self.pose is not None:
            x_mm  = int(round(self.pose[0] * 1000.0))
            y_mm  = int(round(self.pose[1] * 1000.0))
            yaw_c = int(round(math.degrees(self.pose[2]) * 100.0))
            self._write(build_frame(TYPE_P, struct.pack('<iii', x_mm, y_mm, yaw_c)))

        # Lidar çerçevesi
        dists = self._downsample(m)
        payload = struct.pack('<HH', self.seq, len(dists)) + \
            struct.pack(f'<{len(dists)}H', *dists)
        self._write(build_frame(TYPE_L, payload))
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
