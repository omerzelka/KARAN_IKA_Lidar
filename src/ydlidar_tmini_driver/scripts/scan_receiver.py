#!/usr/bin/env python3
# ============================================================================
# scan_receiver.py  —  UZAKTAKI BİLGİSAYARDA çalışır (ROS GEREKMEZ)
#
# Jetson'daki scan_json_bridge'e TCP ile bağlanır, JSON satırlarını okur ve
# her turu (x, y) nokta bulutuna çevirir. Haritalama kodunu handle_scan()
# içine bağlarsın. Sadece Python standart kütüphanesi kullanır.
#
# Çalıştırma:
#   python3 scan_receiver.py           # varsayılan IP/port
#   python3 scan_receiver.py 10.201.219.31 9000
# ============================================================================

import json
import math
import socket
import sys

JETSON_IP = sys.argv[1] if len(sys.argv) > 1 else '10.201.219.31'
PORT      = int(sys.argv[2]) if len(sys.argv) > 2 else 9000


def scan_to_points(rec):
    """Bir tur JSON kaydını (x, y) noktalarına çevirir (sensör çerçevesinde).
    pose varsa noktaları dünya çerçevesine taşır (basit 2B dönüşüm)."""
    pts = []
    a = rec['angle_min']
    inc = rec['angle_increment']
    pose = rec.get('pose')

    for r in rec['ranges']:
        if r is not None:                    # null = geçersiz / menzil dışı
            x = r * math.cos(a)
            y = r * math.sin(a)
            if pose:                         # sensör -> dünya (odom) çerçevesi
                c, s = math.cos(pose['yaw']), math.sin(pose['yaw'])
                x, y = pose['x'] + c * x - s * y, pose['y'] + s * x + c * y
            pts.append((x, y))
        a += inc
    return pts


def handle_scan(rec, pts):
    """>>> HARİTALAMA KODUNU BURAYA BAĞLA <<<
    Şimdilik sadece özet basar. pts = [(x, y), ...] (metre)."""
    print(f"tur t={rec['stamp']:.2f}  nokta={len(pts):4d}  "
          f"pose={rec.get('pose')}")


def main():
    print(f"Bağlanılıyor: {JETSON_IP}:{PORT} ...")
    sock = socket.create_connection((JETSON_IP, PORT))
    print("Bağlandı. Ctrl+C ile çık.")
    buf = b''
    try:
        while True:
            chunk = sock.recv(65536)
            if not chunk:
                print("Bağlantı kapandı (Jetson tarafı durdu).")
                break
            buf += chunk
            # Satır satır işle (NDJSON): her satır bir tur
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                line = line.strip()
                if not line:
                    continue
                try:
                    rec = json.loads(line)
                except json.JSONDecodeError:
                    continue  # yarım/bozuk satır atla
                handle_scan(rec, scan_to_points(rec))
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()


if __name__ == '__main__':
    main()
