#!/usr/bin/env python3
# ============================================================================
# scan_serial_receiver.py  —  PC'de çalışır (Windows/Linux, ROS GEREKMEZ)
#
# STM32'nin PC'ye bağlı 57600 seri portundan $L/$P cümlelerini okur,
# checksum'ı doğrular, her turu (x, y) nokta bulutuna çevirir ve haritalama
# fonksiyonuna verir.
#
# Bağımlılık:  pip install pyserial
# Çalıştırma:
#   Windows:  python scan_serial_receiver.py COM5
#   Linux:    python3 scan_serial_receiver.py /dev/ttyUSB0
# ============================================================================

import math
import sys

try:
    import serial  # pyserial
except ImportError:
    raise SystemExit("pyserial gerekli: pip install pyserial")

PORT = sys.argv[1] if len(sys.argv) > 1 else ('COM5' if sys.platform == 'win32'
                                              else '/dev/ttyUSB0')
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 57600


def nmea_checksum(payload: str) -> int:
    c = 0
    for ch in payload:
        c ^= ord(ch)
    return c


def parse_line(line: str):
    """'$<body>*<XX>' -> ('L'|'P', [alanlar]) ya da checksum/format hatasında None."""
    if not line.startswith('$') or '*' not in line:
        return None
    body, _, cs_hex = line[1:].partition('*')
    try:
        if int(cs_hex[:2], 16) != nmea_checksum(body):
            return None  # checksum tutmadı -> gürültülü satır, at
    except ValueError:
        return None
    fields = body.split(',')
    return fields[0], fields[1:]


_last_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}


def handle_L(fields):
    """$L: seq, N, mm0..mm_{N-1}  -> (x, y) noktaları (varsa dünya çerçevesinde)."""
    if len(fields) < 2:
        return
    seq = int(fields[0]); n = int(fields[1])
    mm = fields[2:2 + n]
    pts = []
    two_pi = 2.0 * math.pi
    px, py, yaw = _last_pose['x'], _last_pose['y'], _last_pose['yaw']
    c, s = math.cos(yaw), math.sin(yaw)
    for i, v in enumerate(mm):
        d = int(v)
        if d > 0:                                   # 0 = geçersiz
            r = d / 1000.0                          # mm -> m
            ang = i / n * two_pi                    # sektör -> açı
            x = r * math.cos(ang); y = r * math.sin(ang)
            # poz varsa sensör -> dünya (odom) çerçevesi
            x, y = px + c * x - s * y, py + s * x + c * y
            pts.append((x, y))
    handle_scan(seq, pts)


def handle_P(fields):
    """$P: x_mm, y_mm, yaw_cdeg -> en son pozu güncelle."""
    if len(fields) < 3:
        return
    _last_pose['x']   = int(fields[0]) / 1000.0
    _last_pose['y']   = int(fields[1]) / 1000.0
    _last_pose['yaw'] = int(fields[2]) / 100.0 * math.pi / 180.0


def handle_scan(seq, pts):
    """>>> HARİTALAMA KODUNU BURAYA BAĞLA <<<  pts = [(x, y), ...] metre."""
    print(f"tur seq={seq:5d}  nokta={len(pts):4d}  poz=({_last_pose['x']:.2f},"
          f"{_last_pose['y']:.2f},{math.degrees(_last_pose['yaw']):.0f}°)")


def main():
    print(f"Seri açılıyor: {PORT} @ {BAUD} ...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except serial.SerialException as e:
        raise SystemExit(f"Port açılamadı: {e}")
    print("Bağlandı. Ctrl+C ile çık.")
    try:
        while True:
            raw = ser.readline()               # '\n' sonuna kadar oku
            if not raw:
                continue
            line = raw.decode('ascii', errors='ignore').strip()
            parsed = parse_line(line)
            if parsed is None:
                continue
            kind, fields = parsed
            if kind == 'L':
                handle_L(fields)
            elif kind == 'P':
                handle_P(fields)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == '__main__':
    main()
