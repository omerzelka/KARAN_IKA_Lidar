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
# Haritalama: gelen her tur log-odds occupancy grid'e işlenir (sensörden her
# noktaya ışın izlenir; yol boyu hücreler BOŞ, uç nokta DOLU) ve matplotlib ile
# canlı çizilir. Poz ('P') geliyorsa turlar dünya (odom) çerçevesinde birikir.
#
# Bağımlılık:  pip install pyserial numpy matplotlib
# Çalıştırma:
#   Windows:  python scan_serial_receiver.py COM5
#   Linux:    python3 scan_serial_receiver.py /dev/ttyUSB0
#   Harita kapalı (sadece log):  python scan_serial_receiver.py COM5 57600 --nomap
# ============================================================================

import math
import struct
import sys
import time

try:
    import serial  # pyserial
except ImportError:
    raise SystemExit("pyserial gerekli: pip install pyserial")

_args = [a for a in sys.argv[1:] if not a.startswith('--')]
_flags = [a for a in sys.argv[1:] if a.startswith('--')]

PORT = _args[0] if len(_args) > 0 else ('COM5' if sys.platform == 'win32'
                                        else '/dev/ttyUSB0')
BAUD = int(_args[1]) if len(_args) > 1 else 57600
MAP_ENABLED = '--nomap' not in _flags

# --- Harita parametreleri (gerekirse burayı ayarla) ------------------------
GRID_RES_M    = 0.05    # hücre boyutu (m) — 5 cm
GRID_HALF_M   = 15.0    # harita yarı-genişliği (m) -> 30x30 m alan
L_OCC         = 0.85    # dolu ölçüm için log-odds artışı
L_FREE        = 0.40    # boş (ışın yolu) için log-odds azalışı
L_CLAMP       = 6.0     # log-odds doygunluk sınırı (±)
REDRAW_PERIOD = 0.25    # ekran yenileme aralığı (s) — okuma döngüsünü yormasın
FREE_RAY_M    = 11.5    # dönüşsüz (d=0) sektör: 'buraya kadar boş' varsayımı (m).
                        # Atlanırsa o doğrultular haritada sonsuza dek gri kalır.
                        # --freeray=0 ile kapatılır.
NEAR_OCCLUSION_M = 1.0  # d=0 sektörün ±2 komşusunda bundan yakın ölçüm varsa
                        # 'örtülme' (el/gövde/<range_min) -> boş-ışın İŞLENMEZ.
for _f in _flags:
    if _f.startswith('--freeray='):
        FREE_RAY_M = float(_f.split('=', 1)[1])

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


# ===========================================================================
# Occupancy grid (log-odds) — sensörden ışın izleyerek boş/dolu hücre günceller
# ===========================================================================
class OccupancyGrid:
    def __init__(self, res=GRID_RES_M, half=GRID_HALF_M):
        import numpy as np
        self.np = np
        self.res = res
        self.half = half
        self.n = int(round(2 * half / res))          # kenar başına hücre sayısı
        self.log = np.zeros((self.n, self.n), dtype=np.float32)  # log-odds

    def w2c(self, x, y):
        """Dünya (m) -> hücre (col, row). Sınır dışıysa None döner."""
        cx = int((x + self.half) / self.res)
        cy = int((y + self.half) / self.res)
        if 0 <= cx < self.n and 0 <= cy < self.n:
            return cx, cy
        return None

    def _ray_cells(self, x0, y0, x1, y1):
        """(x0,y0)->(x1,y1) hücreleri arası Bresenham; uç HARİÇ hücreleri verir."""
        dx = abs(x1 - x0); dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        cells = []
        while not (x0 == x1 and y0 == y1):
            cells.append((x0, y0))
            e2 = 2 * err
            if e2 > -dy:
                err -= dy; x0 += sx
            if e2 < dx:
                err += dx; y0 += sy
        return cells  # uç noktayı (dolu) içermez

    def _clip(self, ox, oy, px, py):
        """Harita dışına taşan ışın ucunu sınıra kırpar (yön korunur)."""
        lim = self.half * 0.999
        t = 1.0
        dx, dy = px - ox, py - oy
        if abs(dx) > 1e-12:
            t = min(t, ((lim if dx > 0 else -lim) - ox) / dx)
        if abs(dy) > 1e-12:
            t = min(t, ((lim if dy > 0 else -lim) - oy) / dy)
        t = max(t, 0.0)
        return ox + dx * t, oy + dy * t

    def _mark_free_ray(self, oc, ec, include_end):
        cells = self._ray_cells(oc[0], oc[1], ec[0], ec[1])
        if include_end:
            cells.append(ec)
        for (cx, cy) in cells:
            self.log[cy, cx] = max(-L_CLAMP, self.log[cy, cx] - L_FREE)

    def integrate(self, ox, oy, points):
        """ox,oy = sensör orijini (m, dünya); points = [(x,y)] uç noktalar (m).
        Uç harita dışındaysa ışının içerideki kısmı yine boş işaretlenir."""
        oc = self.w2c(ox, oy)
        if oc is None:
            return
        for (px, py) in points:
            ec = self.w2c(px, py)
            if ec is None:
                clipped = self.w2c(*self._clip(ox, oy, px, py))
                if clipped is not None:
                    self._mark_free_ray(oc, clipped, include_end=True)
                continue
            # ışın yolu: boş hücreler (log-odds azalt)
            self._mark_free_ray(oc, ec, include_end=False)
            # uç nokta: dolu (log-odds artır)
            self.log[ec[1], ec[0]] = min(L_CLAMP, self.log[ec[1], ec[0]] + L_OCC)

    def integrate_free(self, ox, oy, points):
        """Dönüşsüz ışınlar: uç DAHİL tüm yol boş (engel işaretlenmez)."""
        oc = self.w2c(ox, oy)
        if oc is None:
            return
        for (px, py) in points:
            ec = self.w2c(px, py)
            if ec is None:
                ec = self.w2c(*self._clip(ox, oy, px, py))
                if ec is None:
                    continue
            self._mark_free_ray(oc, ec, include_end=True)

    def prob_image(self):
        """Görselleştirme için olasılık [0..1] görüntüsü (bilinmeyen=0.5)."""
        return 1.0 / (1.0 + self.np.exp(-self.log))


_grid = None      # OccupancyGrid örneği (main'de kurulur)
_viz = None       # matplotlib durum sözlüğü
_last_draw = 0.0  # son yenileme zamanı


def _occluded(mm, i, n):
    """d=0 sektörün ±2 komşusunda NEAR_OCCLUSION_M'den yakın geçerli ölçüm
    varsa True: 0 'menzil dışı' değil 'örtülme' (el/gövde/çok yakın engel)."""
    lim = NEAR_OCCLUSION_M * 1000.0
    for off in (-2, -1, 1, 2):
        d = mm[(i + off) % n]
        if 0 < d < lim:
            return True
    return False


def handle_L(payload: bytes):
    """L payload: seq(u16) N(u16) N×u16 mm -> (x, y) noktaları (dünya çerçevesinde)."""
    if len(payload) < 4:
        return
    seq, n = struct.unpack_from('<HH', payload, 0)
    if len(payload) < 4 + 2 * n:
        return  # eksik/bozuk
    mm = struct.unpack_from(f'<{n}H', payload, 4)
    pts, free_pts = [], []
    two_pi = 2.0 * math.pi
    px, py, yaw = _last_pose['x'], _last_pose['y'], _last_pose['yaw']
    c, s = math.cos(yaw), math.sin(yaw)
    for i, d in enumerate(mm):
        ang = i / n * two_pi                        # sektör -> açı
        if d > 0:
            r = d / 1000.0                          # mm -> m
            x = r * math.cos(ang); y = r * math.sin(ang)
            # poz varsa sensör -> dünya (odom) çerçevesi
            x, y = px + c * x - s * y, py + s * x + c * y
            pts.append((x, y))
        elif FREE_RAY_M > 0.0 and not _occluded(mm, i, n):
            # dönüşsüz + örtülü DEĞİL: 'FREE_RAY_M'ye kadar engel yok' ışını —
            # işlenmezse bu doğrultular haritada sonsuza dek gri kalır.
            # (Örtülüyse — komşuda yakın ölçüm — ışın işlenmez: el/gövde.)
            fx = FREE_RAY_M * math.cos(ang); fy = FREE_RAY_M * math.sin(ang)
            free_pts.append((px + c * fx - s * fy, py + s * fx + c * fy))
    handle_scan(seq, pts, free_pts)


def handle_P(payload: bytes):
    """P payload: x_mm(i32) y_mm(i32) yaw_cdeg(i32) -> en son pozu güncelle."""
    if len(payload) < 12:
        return
    x_mm, y_mm, yaw_c = struct.unpack_from('<iii', payload, 0)
    _last_pose['x']   = x_mm / 1000.0
    _last_pose['y']   = y_mm / 1000.0
    _last_pose['yaw'] = yaw_c / 100.0 * math.pi / 180.0


def handle_scan(seq, pts, free_pts=()):
    """Bir tur geldi: occupancy grid'i güncelle + canlı çizimi yenile.
    pts = engel uçları, free_pts = dönüşsüz boş-ışın uçları (m, dünya)."""
    print(f"tur seq={seq:5d}  nokta={len(pts):4d}  poz=({_last_pose['x']:.2f},"
          f"{_last_pose['y']:.2f},{math.degrees(_last_pose['yaw']):.0f}°)")

    if _grid is None:
        return  # harita kapalı (--nomap) -> sadece log

    # Sensör orijini = en son poz (yoksa 0,0). Işınlar buradan çizilir.
    _grid.integrate(_last_pose['x'], _last_pose['y'], pts)
    if free_pts:
        _grid.integrate_free(_last_pose['x'], _last_pose['y'], free_pts)

    global _last_draw
    now = time.monotonic()
    if now - _last_draw >= REDRAW_PERIOD:
        _last_draw = now
        _redraw()


def _init_viz():
    """matplotlib penceresini kurar; başarısızsa None döner (harita devre dışı)."""
    try:
        import numpy  # noqa: F401  (grid için de gerekli)
        import matplotlib
        import matplotlib.pyplot as plt
    except ImportError:
        print("[uyarı] numpy/matplotlib yok -> harita kapalı "
              "(pip install numpy matplotlib). Sadece log basılacak.")
        return None

    plt.ion()
    fig, ax = plt.subplots(figsize=(7, 7))
    extent = [-GRID_HALF_M, GRID_HALF_M, -GRID_HALF_M, GRID_HALF_M]
    # gray_r: olasılık yüksek (dolu) -> koyu, düşük (boş) -> açık, 0.5 -> gri
    im = ax.imshow(_grid.prob_image(), cmap='gray_r', origin='lower',
                   extent=extent, vmin=0.0, vmax=1.0, interpolation='nearest')
    robot, = ax.plot([0], [0], 'r^', markersize=9, label='araç')
    ax.set_title('Canlı Occupancy Grid  (koyu=engel, açık=boş)')
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    ax.set_aspect('equal'); ax.legend(loc='upper right')
    fig.tight_layout()
    return {'plt': plt, 'fig': fig, 'ax': ax, 'im': im, 'robot': robot}


def _redraw():
    if _viz is None:
        return
    _viz['im'].set_data(_grid.prob_image())
    _viz['robot'].set_data([_last_pose['x']], [_last_pose['y']])
    _viz['fig'].canvas.draw_idle()
    _viz['plt'].pause(0.001)   # GUI olaylarını işle (pencere donmasın)


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
        if length > 4096:             # bozuk len baytı -> sahte sync, takılma
            i += 2
            continue
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
    global _grid, _viz

    # Harita altyapısını kur (numpy/matplotlib yoksa nazikçe kapanır)
    if MAP_ENABLED:
        try:
            _grid = OccupancyGrid()
            _viz = _init_viz()
            if _viz is None:
                _grid = None      # matplotlib yoksa grid'i de bırak
        except ImportError:
            print("[uyarı] numpy yok -> harita kapalı (pip install numpy matplotlib).")
            _grid = None

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
            elif _viz is not None:
                _viz['plt'].pause(0.001)  # veri yokken de pencereyi canlı tut
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        if _viz is not None:
            print("Harita penceresi açık kalsın diye bekleniyor (kapatınca çıkar)...")
            _viz['plt'].ioff()
            _viz['plt'].show()


if __name__ == '__main__':
    main()
