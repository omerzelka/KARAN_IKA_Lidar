#!/usr/bin/env python3
# ============================================================================
# scan_serial_receiver.py  —  PC'de çalışır (Windows/Linux, ROS GEREKMEZ)
#
# STM32'nin PC'ye bağlı 57600 seri portundan BİNARY çerçeveleri okur, CRC'yi
# doğrular, her turu (x, y) nokta bulutuna çevirir ve haritalama fonksiyonuna
# verir.
#
# BİNARY ÇERÇEVE (scan_serial_bridge.py ile aynı, little-endian):
#   [0xAA 0x55] [type:u8] [len:u16] [payload:len bayt] [crc:u16]
#     type 'L'(0x4C): seq(u16) + N(u16) + N × u16 mm  (0 = geçersiz)
#     type 'P'(0x50): x_mm(i32) + y_mm(i32) + yaw_cdeg(i32)
#     crc : type+len+payload üstünde CRC-16/CCITT-FALSE
#
# Bağımlılık:  pip install pyserial
# Çalıştırma:
#   Windows:  python scan_serial_receiver.py COM5
#   Linux:    python3 scan_serial_receiver.py /dev/ttyUSB0
# ============================================================================

import math
import struct
import sys

try:
    import serial  # pyserial
except ImportError:
    raise SystemExit("pyserial gerekli: pip install pyserial")

PORT = sys.argv[1] if len(sys.argv) > 1 else ('COM5' if sys.platform == 'win32'
                                              else '/dev/ttyUSB0')
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 57600

SYNC0, SYNC1 = 0xAA, 0x55
TYPE_L = 0x4C
TYPE_P = 0x50


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE — köprüdeki ile aynı olmalı."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


_last_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}


def handle_L(payload: bytes):
    """L payload: seq(u16) N(u16) N×u16 mm -> (x, y) noktaları (dünya çerçevesinde)."""
    if len(payload) < 4:
        return
    seq, n = struct.unpack_from('<HH', payload, 0)
    if len(payload) < 4 + 2 * n:
        return  # eksik/bozuk
    mm = struct.unpack_from(f'<{n}H', payload, 4)
    pts = []
    two_pi = 2.0 * math.pi
    px, py, yaw = _last_pose['x'], _last_pose['y'], _last_pose['yaw']
    c, s = math.cos(yaw), math.sin(yaw)
    for i, d in enumerate(mm):
        if d > 0:                                   # 0 = geçersiz
            r = d / 1000.0                          # mm -> m
            ang = i / n * two_pi                    # sektör -> açı
            x = r * math.cos(ang); y = r * math.sin(ang)
            # poz varsa sensör -> dünya (odom) çerçevesi
            x, y = px + c * x - s * y, py + s * x + c * y
            pts.append((x, y))
    handle_scan(seq, pts)


def handle_P(payload: bytes):
    """P payload: x_mm(i32) y_mm(i32) yaw_cdeg(i32) -> en son pozu güncelle."""
    if len(payload) < 12:
        return
    x_mm, y_mm, yaw_c = struct.unpack_from('<iii', payload, 0)
    _last_pose['x']   = x_mm / 1000.0
    _last_pose['y']   = y_mm / 1000.0
    _last_pose['yaw'] = yaw_c / 100.0 * math.pi / 180.0


def handle_scan(seq, pts):
    """>>> HARİTALAMA KODUNU BURAYA BAĞLA <<<  pts = [(x, y), ...] metre."""
    print(f"tur seq={seq:5d}  nokta={len(pts):4d}  poz=({_last_pose['x']:.2f},"
          f"{_last_pose['y']:.2f},{math.degrees(_last_pose['yaw']):.0f}°)")


def parse_buffer(buf: bytearray):
    """Tampondaki tüm tam çerçeveleri çözer, tüketilenleri buf'tan atar.
    Eksik çerçeve kalırsa bir sonraki okumayı bekler."""
    i = 0
    n = len(buf)
    while True:
        # sync word ara
        while i + 1 < n and not (buf[i] == SYNC0 and buf[i + 1] == SYNC1):
            i += 1
        if i + 5 > n:                 # sync + type + len için yeterli bayt yok
            break
        length = buf[i + 3] | (buf[i + 4] << 8)
        frame_end = i + 5 + length + 2   # sync(2)+type(1)+len(2)+payload+crc(2)
        if frame_end > n:             # payload+crc henüz gelmedi -> bekle
            break
        body = bytes(buf[i + 2:i + 5 + length])          # type+len+payload
        crc_rx = buf[i + 5 + length] | (buf[i + 6 + length] << 8)
        if crc16_ccitt(body) == crc_rx:
            type_byte = buf[i + 2]
            payload = bytes(buf[i + 5:i + 5 + length])
            if type_byte == TYPE_L:
                handle_L(payload)
            elif type_byte == TYPE_P:
                handle_P(payload)
            i = frame_end             # çerçeveyi tükettik
        else:
            i += 2                    # CRC tutmadı -> sync'ten sonra yeniden ara
    del buf[:i]                       # işlenen/atılan baytları çıkar


def main():
    print(f"Seri açılıyor: {PORT} @ {BAUD} ...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except serial.SerialException as e:
        raise SystemExit(f"Port açılamadı: {e}")
    print("Bağlandı. Ctrl+C ile çık.")
    buf = bytearray()
    try:
        while True:
            chunk = ser.read(512)         # timeout=1s içinde gelen baytlar
            if chunk:
                buf.extend(chunk)
                parse_buffer(buf)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == '__main__':
    main()
