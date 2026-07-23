#!/usr/bin/env python3
# ============================================================================
# scan_map_viewer.py  —  PC'de çalışır (Windows/Linux, ROS GEREKMEZ)
#
# STM32 relay hattından (57600) gelen binary lidar çerçevelerinin
# DOĞRULUĞUNU sınamak + 2B haritalamak için canlı uygulama:
#
#   SOL   : birikimli occupancy grid (log-odds, sensörden ışın izleme)
#   SAĞ ÜST: anlık tarama (sensör çerçevesi) — lidarın "o an gördüğü"
#   SAĞ ALT: canlı istatistik paneli:
#            - CRC hatası        (hat bozulması -> çerçeve atılır)
#            - kayıp tur         (seq boşluğu -> bant/tampon sorunu)
#            - tur hızı (Hz)     (T-mini Plus ~6.1 Hz beklenir)
#            - bant genişliği    (57600 bütçesinin yüzdesi)
#            - menzil dışı sayaç (>12 m 'şüpheli' — T-mini Plus üst sınırı)
#
# Çerçeve formatı scan_serial_bridge.py ile birebir aynı (little-endian):
#   [0xAA 0x55][type:u8][len:u16][payload][crc:u16]  crc=CRC-16/CCITT-FALSE
#   'L'(0x4C): seq(u16)+N(u16)+N×u16 mm (0=geçersiz)
#   'P'(0x50): x_mm(i32)+y_mm(i32)+yaw_cdeg(i32)
#
# Klavye:  c = haritayı temizle    s = PNG kaydet    q = çık
#
# Bağımlılık:  pip install pyserial numpy matplotlib
# Çalıştırma:  python scan_map_viewer.py COM6            (Windows)
#              python3 scan_map_viewer.py /dev/ttyACM0   (Linux)
# ============================================================================

import math
import struct
import sys
import time
from collections import deque

try:
    import serial  # pyserial
except ImportError:
    raise SystemExit("pyserial gerekli: pip install pyserial")

# --- Ayarlar ----------------------------------------------------------------
GRID_RES_M    = 0.05     # hücre boyutu (m)
GRID_HALF_M   = 15.0     # harita yarı-genişliği (m) -> 30x30 m
L_OCC         = 0.85     # dolu ölçüm log-odds artışı
L_FREE        = 0.40     # boş (ışın yolu) log-odds azalışı
L_CLAMP       = 6.0      # log-odds doygunluk (±)
REDRAW_PERIOD = 0.25     # ekran yenileme aralığı (s)
MAX_PAYLOAD   = 4096     # bozuk len baytına karşı üst sınır (L: 4+2*N)
RANGE_MAX_MM  = 12000    # T-mini Plus üst menzil — üstü 'şüpheli' sayılır
NO_RETURN_FREE_M = 11.5  # dönüşsüz (d=0) sektör: 'buraya kadar boş' varsayılır (m).
                         # 0 gönderilen sektörler atlanırsa o doğrultular haritada
                         # sonsuza dek gri kalır! --freeray=0 ile kapatılabilir.
NEAR_OCCLUSION_M = 1.0   # ÖRTÜLME KAPISI: d=0 sektörün ±2 komşusunda bundan
                         # YAKIN geçerli ölçüm varsa 0 'çok uzak' değil 'örtülü'
                         # demektir (el/gövde, <range_min) -> boş-ışın İŞLENMEZ,
                         # sektör bilinmeyen kalır. Yoksa lidara el koyunca
                         # elin arkasına sahte beyaz ışınlar çizilir.

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


# ===========================================================================
# Çerçeve çözücü — CRC/resync sayaçlarıyla (doğruluk analizi için)
# ===========================================================================
class FrameParser:
    def __init__(self):
        self.buf = bytearray()
        self.bytes_total = 0     # okunan toplam bayt (bant genişliği için)
        self.frames_ok = 0       # CRC'si tutan çerçeve
        self.crc_errors = 0      # CRC'si tutmayan çerçeve (atıldı)

    def feed(self, chunk: bytes):
        """Baytları tampona ekler, çözülen (type, payload) listesini döner."""
        self.bytes_total += len(chunk)
        self.buf.extend(chunk)
        out = []
        buf = self.buf
        i, n = 0, len(buf)
        while True:
            while i + 1 < n and not (buf[i] == SYNC0 and buf[i + 1] == SYNC1):
                i += 1
            if i + 5 > n:                 # sync+type+len için yeterli bayt yok
                break
            length = buf[i + 3] | (buf[i + 4] << 8)
            if length > MAX_PAYLOAD:      # bozuk len -> sahte sync, yeniden ara
                i += 2
                continue
            end = i + 5 + length + 2
            if end > n:                   # payload+crc henüz gelmedi
                break
            body = bytes(buf[i + 2:i + 5 + length])
            crc_rx = buf[i + 5 + length] | (buf[i + 6 + length] << 8)
            if crc16_ccitt(body) == crc_rx:
                out.append((buf[i + 2], bytes(buf[i + 5:i + 5 + length])))
                self.frames_ok += 1
                i = end
            else:
                self.crc_errors += 1
                i += 2                    # sync'ten sonra yeniden ara
        del buf[:i]
        return out


# ===========================================================================
# İstatistik — veri doğruluğunun canlı özeti
# ===========================================================================
class Stats:
    def __init__(self):
        self.t0 = time.monotonic()
        self.frames_l = 0
        self.frames_p = 0
        self.last_seq = None
        self.lost = 0            # seq boşluğu toplamı (kayıp tur)
        self.out_of_order = 0    # geriye giden seq (sıra dışı/tekrar)
        self.l_times = deque(maxlen=32)   # Hz hesabı için son L zamanları
        self.oor_total = 0       # menzil dışı (>12 m) ölçüm toplamı
        # son tur özeti
        self.seq = 0
        self.n = 0
        self.valid = 0
        self.noret = 0           # son turda dönüşsüz sektör (d=0 toplamı)
        self.freect = 0          # bunlardan boş-ışın işlenen (örtülü = fark)
        self.rmin = self.ravg = self.rmax = 0.0

    def on_l(self, seq, n, valid, oor, noret, freect, rmin, ravg, rmax):
        now = time.monotonic()
        self.frames_l += 1
        self.l_times.append(now)
        if self.last_seq is not None:
            gap = (seq - self.last_seq - 1) & 0xFFFF
            if gap < 32768:
                self.lost += gap
            else:
                self.out_of_order += 1
        self.last_seq = seq
        self.oor_total += oor
        self.seq, self.n, self.valid, self.noret = seq, n, valid, noret
        self.freect = freect
        self.rmin, self.ravg, self.rmax = rmin, ravg, rmax

    def hz(self):
        if len(self.l_times) < 2:
            return 0.0
        dt = self.l_times[-1] - self.l_times[0]
        return (len(self.l_times) - 1) / dt if dt > 0 else 0.0

    def health(self):
        """Kaba sağlık özeti — doğrulama testinin 'sonuç' satırı."""
        if self.frames_l == 0:
            return "VERİ BEKLENİYOR..."
        bad = []
        if self.crc_ref.crc_errors:
            bad.append(f"CRC hatası={self.crc_ref.crc_errors}")
        if self.lost:
            bad.append(f"kayıp tur={self.lost}")
        if self.oor_total:
            bad.append(f"menzil dışı={self.oor_total}")
        return "VERİ TEMİZ ✓" if not bad else "SORUN: " + ", ".join(bad)


# ===========================================================================
# Occupancy grid (log-odds) — sensörden ışın izleyerek boş/dolu günceller
# ===========================================================================
class OccupancyGrid:
    def __init__(self, res=GRID_RES_M, half=GRID_HALF_M):
        import numpy as np
        self.np = np
        self.res, self.half = res, half
        self.n = int(round(2 * half / res))
        self.log = np.zeros((self.n, self.n), dtype=np.float32)

    def clear(self):
        self.log.fill(0.0)

    def w2c(self, x, y):
        cx = int((x + self.half) / self.res)
        cy = int((y + self.half) / self.res)
        if 0 <= cx < self.n and 0 <= cy < self.n:
            return cx, cy
        return None

    def _ray_cells(self, x0, y0, x1, y1):
        """Bresenham; uç HARİÇ hücreler."""
        dx, dy = abs(x1 - x0), abs(y1 - y0)
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
        return cells

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
        """Engel uçlu ışınlar: yol=boş, uç=dolu. Uç harita dışındaysa ışının
        harita içindeki kısmı yine boş işaretlenir (gri boşluk kalmasın)."""
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
            self._mark_free_ray(oc, ec, include_end=False)
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
        return 1.0 / (1.0 + self.np.exp(-self.log))


# ===========================================================================
# L payload çözümü — sensör noktaları + dünya noktaları + doğruluk sayaçları
# ===========================================================================
def _occluded(mm, i, n):
    """d=0 sektörün ±2 komşusunda NEAR_OCCLUSION_M'den yakın geçerli ölçüm
    varsa True: 0 'menzil dışı' değil 'örtülme' (el/gövde/çok yakın engel)."""
    lim = NEAR_OCCLUSION_M * 1000.0
    for off in (-2, -1, 1, 2):
        d = mm[(i + off) % n]
        if 0 < d <= RANGE_MAX_MM and d < lim:
            return True
    return False


def decode_l(payload, pose, free_range_m=NO_RETURN_FREE_M):
    """(seq, n, sensor_pts, world_pts, free_pts, valid, oor, noret, rmin, ravg,
    rmax) döner; payload eksikse None. free_pts = dönüşsüz (d=0) sektörlerde
    'buraya kadar engel yok' varsayılan ışın uçları (dünya çerçevesi) — bunlar
    haritaya boş işlenmezse o doğrultular sonsuza dek gri kalır."""
    if len(payload) < 4:
        return None
    seq, n = struct.unpack_from('<HH', payload, 0)
    if n == 0 or len(payload) < 4 + 2 * n:
        return None
    mm = struct.unpack_from(f'<{n}H', payload, 4)
    two_pi = 2.0 * math.pi
    px, py, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    sensor_pts, world_pts, free_pts, rs = [], [], [], []
    oor = noret = 0
    for i, d in enumerate(mm):
        ang = i / n * two_pi
        if d == 0:                        # dönüş yok: uzak MI, örtülü MÜ?
            noret += 1
            # örtülme kapısı: yakın komşusu varsa boş-ışın İŞLEME (el/gövde)
            if free_range_m > 0.0 and not _occluded(mm, i, n):
                fx = free_range_m * math.cos(ang)
                fy = free_range_m * math.sin(ang)
                free_pts.append((px + c * fx - s * fy, py + s * fx + c * fy))
            continue
        if d > RANGE_MAX_MM:
            oor += 1                      # T-mini üst menzili aşan 'şüpheli'
            continue
        r = d / 1000.0
        x, y = r * math.cos(ang), r * math.sin(ang)
        sensor_pts.append((x, y))
        world_pts.append((px + c * x - s * y, py + s * x + c * y))
        rs.append(r)
    if rs:
        rmin, ravg, rmax = min(rs), sum(rs) / len(rs), max(rs)
    else:
        rmin = ravg = rmax = 0.0
    return (seq, n, sensor_pts, world_pts, free_pts,
            len(rs), oor, noret, rmin, ravg, rmax)


# ===========================================================================
# Canlı görselleştirme
# ===========================================================================
class Viewer:
    def __init__(self, grid, baud):
        import matplotlib.pyplot as plt
        self.plt = plt
        self.grid = grid
        self.baud = baud
        plt.ion()
        self.fig = plt.figure(figsize=(12, 7))
        self.fig.canvas.manager.set_window_title('KARAN İKA — Lidar Doğrulama & Harita')
        gs = self.fig.add_gridspec(2, 2, width_ratios=[1.4, 1.0],
                                   height_ratios=[2.2, 1.0])

        # Sol: birikimli occupancy grid
        self.ax_map = self.fig.add_subplot(gs[:, 0])
        ext = [-grid.half, grid.half, -grid.half, grid.half]
        self.im = self.ax_map.imshow(grid.prob_image(), cmap='gray_r',
                                     origin='lower', extent=ext,
                                     vmin=0.0, vmax=1.0, interpolation='nearest')
        self.robot, = self.ax_map.plot([0], [0], 'r^', ms=9, label='araç')
        self.ax_map.set_title('Birikimli Occupancy Grid  (koyu=engel, açık=boş)')
        self.ax_map.set_xlabel('x [m]'); self.ax_map.set_ylabel('y [m]')
        self.ax_map.set_aspect('equal'); self.ax_map.legend(loc='upper right')

        # Sağ üst: anlık tarama (sensör çerçevesi)
        self.ax_scan = self.fig.add_subplot(gs[0, 1])
        self.scan_dots, = self.ax_scan.plot([], [], '.', ms=3, color='tab:blue')
        self.ax_scan.plot([0], [0], 'r^', ms=8)
        lim = 12.5
        self.ax_scan.set_xlim(-lim, lim); self.ax_scan.set_ylim(-lim, lim)
        self.ax_scan.set_aspect('equal'); self.ax_scan.grid(True, alpha=0.3)
        self.ax_scan.set_title('Anlık Tarama (sensör çerçevesi)')

        # Sağ alt: istatistik paneli
        ax_txt = self.fig.add_subplot(gs[1, 1]); ax_txt.axis('off')
        self.txt = ax_txt.text(0.0, 1.0, '', va='top', ha='left',
                               family='monospace', fontsize=9,
                               transform=ax_txt.transAxes)

        self.fig.tight_layout()
        self.fig.canvas.mpl_connect('key_press_event', self._on_key)
        self._rate_t = time.monotonic()
        self._rate_b = 0
        self.rate_bps = 0.0

    def _on_key(self, ev):
        if ev.key == 'c':
            self.grid.clear()
            print("[viewer] harita temizlendi")
        elif ev.key == 's':
            name = time.strftime('harita_%Y%m%d_%H%M%S.png')
            self.fig.savefig(name, dpi=150)
            print(f"[viewer] kaydedildi: {name}")
        elif ev.key == 'q':
            self.plt.close(self.fig)

    def alive(self):
        return self.plt.fignum_exists(self.fig.number)

    def update(self, stats: Stats, parser: FrameParser, sensor_pts, pose):
        # bant genişliği (kayan pencere)
        now = time.monotonic()
        dt = now - self._rate_t
        if dt >= 1.0:
            self.rate_bps = (parser.bytes_total - self._rate_b) / dt
            self._rate_t, self._rate_b = now, parser.bytes_total

        self.im.set_data(self.grid.prob_image())
        self.robot.set_data([pose[0]], [pose[1]])
        if sensor_pts:
            self.scan_dots.set_data([p[0] for p in sensor_pts],
                                    [p[1] for p in sensor_pts])

        budget = self.baud / 10.0          # 8N1: baud/10 = byte/s
        pct = 100.0 * self.rate_bps / budget if budget else 0.0
        el = now - stats.t0
        lines = [
            f"SÜRE     : {el:7.1f} s   BANT: {self.rate_bps:5.0f} B/s (%{pct:.0f} @ {self.baud})",
            f"TUR (L)  : {stats.frames_l:6d}  @ {stats.hz():4.1f} Hz   POZ (P): {stats.frames_p}",
            f"CRC HATA : {parser.crc_errors:6d}   KAYIP TUR: {stats.lost}   SIRA DIŞI: {stats.out_of_order}",
            f"SON TUR  : seq={stats.seq}  N={stats.n}  geçerli={stats.valid}"
            f"  dönüşsüz={stats.noret} (örtülü={stats.noret - stats.freect})",
            f"MESAFE   : min {stats.rmin:.2f}  ort {stats.ravg:.2f}  maks {stats.rmax:.2f} m"
            f"   MENZİL DIŞI: {stats.oor_total}",
            f"POZ      : x={pose[0]:.2f}  y={pose[1]:.2f}  yaw={math.degrees(pose[2]):.0f}°",
            "",
            f">>> {stats.health()} <<<",
            "klavye: c=temizle  s=PNG kaydet  q=çık",
        ]
        self.txt.set_text("\n".join(lines))
        self.fig.canvas.draw_idle()
        self.plt.pause(0.001)


# ===========================================================================
def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    port = args[0] if args else ('COM6' if sys.platform == 'win32' else '/dev/ttyACM0')
    baud = int(args[1]) if len(args) > 1 else 57600
    free_m = NO_RETURN_FREE_M
    for a in sys.argv[1:]:
        if a.startswith('--freeray='):
            free_m = float(a.split('=', 1)[1])   # --freeray=0 -> kapalı

    try:
        import numpy  # noqa: F401
        import matplotlib  # noqa: F401
    except ImportError:
        raise SystemExit("numpy+matplotlib gerekli: pip install numpy matplotlib\n"
                         "(msys2: pacman -S mingw-w64-ucrt-x86_64-python-numpy "
                         "mingw-w64-ucrt-x86_64-python-matplotlib)")

    grid = OccupancyGrid()
    viewer = Viewer(grid, baud)
    parser = FrameParser()
    stats = Stats()
    stats.crc_ref = parser          # health() CRC sayacına buradan bakar

    print(f"Seri açılıyor: {port} @ {baud} ...")
    try:
        ser = serial.Serial(port, baud, timeout=0.05)
    except serial.SerialException as e:
        raise SystemExit(f"Port açılamadı: {e}")
    print("Bağlandı. Pencereyi kapat ya da q ile çık.")

    pose = (0.0, 0.0, 0.0)
    last_sensor_pts = []
    last_draw = 0.0
    try:
        while viewer.alive():
            chunk = ser.read(4096)
            if chunk:
                for tb, payload in parser.feed(chunk):
                    if tb == TYPE_P and len(payload) >= 12:
                        x_mm, y_mm, yaw_c = struct.unpack_from('<iii', payload, 0)
                        pose = (x_mm / 1000.0, y_mm / 1000.0,
                                yaw_c / 100.0 * math.pi / 180.0)
                        stats.frames_p += 1
                    elif tb == TYPE_L:
                        dec = decode_l(payload, pose, free_m)
                        if dec is None:
                            continue
                        (seq, n, spts, wpts, fpts,
                         valid, oor, noret, rmin, ravg, rmax) = dec
                        stats.on_l(seq, n, valid, oor, noret, len(fpts),
                                   rmin, ravg, rmax)
                        grid.integrate(pose[0], pose[1], wpts)
                        grid.integrate_free(pose[0], pose[1], fpts)
                        last_sensor_pts = spts
            now = time.monotonic()
            if now - last_draw >= REDRAW_PERIOD:
                last_draw = now
                viewer.update(stats, parser, last_sensor_pts, pose)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        print(f"\nÖZET: L={stats.frames_l} tur, CRC hata={parser.crc_errors}, "
              f"kayıp={stats.lost}, menzil dışı={stats.oor_total}")


if __name__ == '__main__':
    main()
