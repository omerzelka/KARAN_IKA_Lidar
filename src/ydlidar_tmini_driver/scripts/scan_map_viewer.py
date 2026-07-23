#!/usr/bin/env python3
# ============================================================================
# scan_map_viewer.py  —  PC'de çalışır (Windows/Linux, ROS GEREKMEZ)
#
# STM32 relay hattından (57600) gelen binary lidar çerçevelerinin
# DOĞRULUĞUNU sınamak + 2B haritalamak için canlı uygulama:
#
#   SOL   : birikimli occupancy grid (log-odds, NumPy vektörize ışın izleme)
#   SAĞ ÜST: anlık tarama (sensör çerçevesi, mesafeye göre renkli)
#   SAĞ ALT: canlı istatistik paneli (CRC, kayıp tur, Hz, bant, spike, örtülme)
#
# Çerçeve formatı scan_serial_bridge.py ile birebir aynı (little-endian):
#   [0xAA 0x55][type:u8][len:u16][payload][crc:u16]  crc=CRC-16/CCITT-FALSE
#   'L'(0x4C): seq(u16)+N(u16)+N×u16 mm (0=geçersiz)
#   'P'(0x50): x_mm(i32)+y_mm(i32)+yaw_cdeg(i32)
#
# VERİ TEMİZLİĞİ (bu sürümde eklenen korumalar):
#   * TUR-SEVİYESİ ÖRTÜLME KAPISI: bir turda dönüşsüz (d=0) sektör oranı
#     OCCLUDED_TURN_FRAC'ı aşarsa sensör örtülü kabul edilir (el/battaniye) ->
#     o turda HİÇ boş-ışın işlenmez. Eskiden lidar elle tamamen kapatılınca
#     tüm yönlere 11.5 m sahte "boş" yazılıp harita bozuluyordu.
#   * SPIKE FİLTRESİ: SPIKE_NEAR_M'den yakın, iki yanı da onu doğrulamayan
#     İZOLE tek-sektör okumalar gürültü sayılıp atılır (panelde sayılır).
#     Gerçek yakın engel (el/duvar) komşu sektörlerde de görünür, etkilenmez.
#
# Klavye:  c = haritayı temizle    s = PNG kaydet    q = çık
#
# Bağımlılık:  pip install pyserial numpy matplotlib
# Çalıştırma:  python scan_map_viewer.py COM6            (Windows)
#              python3 scan_map_viewer.py /dev/ttyACM0   (Linux)
#              ... [baud]  [--freeray=M]  (--freeray=0 boş-ışını kapatır)
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
REDRAW_PERIOD = 0.20     # ekran yenileme aralığı (s)
MAX_PAYLOAD   = 4096     # bozuk len baytına karşı üst sınır (L: 4+2*N)
RANGE_MAX_MM  = 12000    # T-mini Plus üst menzil — üstü 'şüpheli' sayılır
NO_RETURN_FREE_M = 11.5  # dönüşsüz (d=0) sektör: 'buraya kadar boş' varsayılır (m).
                         # 0 gönderilen sektörler atlanırsa o doğrultular haritada
                         # sonsuza dek gri kalır! --freeray=0 ile kapatılabilir.
NEAR_OCCLUSION_M = 1.0   # SEKTÖR ÖRTÜLME KAPISI: d=0 sektörün ±2 komşusunda
                         # bundan yakın geçerli ölçüm varsa 0 'çok uzak' değil
                         # 'örtülü' demektir -> boş-ışın işlenmez.
OCCLUDED_TURN_FRAC = 0.35  # TUR ÖRTÜLME KAPISI: dönüşsüz oran bunu aşarsa
                           # sensör kapalı kabul edilir, TÜM boş-ışınlar atlanır.
SPIKE_NEAR_M  = 0.5      # bundan yakın İZOLE tek-sektör okuma 'spike' sayılır
SPIKE_RATIO   = 3.0      # komşu, spike adayından bu çarpandan uzaksa doğrulamaz

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
        self.spikes_total = 0    # atılan izole yakın-gürültü (spike) toplamı
        self.covered_turns = 0   # tur-seviyesi örtülme kapısına takılan turlar
        # son tur özeti
        self.seq = 0
        self.n = 0
        self.valid = 0
        self.noret = 0           # son turda dönüşsüz sektör (d=0 toplamı)
        self.freect = 0          # bunlardan boş-ışın işlenen (örtülü = fark)
        self.covered = False     # son tur örtülü müydü?
        self.rmin = self.ravg = self.rmax = 0.0

    def on_l(self, seq, n, valid, oor, noret, freect, spikes, covered,
             rmin, ravg, rmax):
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
        self.spikes_total += spikes
        if covered:
            self.covered_turns += 1
        self.seq, self.n, self.valid, self.noret = seq, n, valid, noret
        self.freect = freect
        self.covered = covered
        self.rmin, self.ravg, self.rmax = rmin, ravg, rmax

    def hz(self):
        if len(self.l_times) < 2:
            return 0.0
        dt = self.l_times[-1] - self.l_times[0]
        return (len(self.l_times) - 1) / dt if dt > 0 else 0.0

    def health(self):
        """Kaba sağlık özeti — doğrulama testinin 'sonuç' satırı."""
        if self.frames_l == 0:
            return "VERİ BEKLENİYOR...", 'wait'
        if self.covered:
            return "SENSÖR ÖRTÜLÜ — harita korunuyor", 'warn'
        bad = []
        if self.crc_ref.crc_errors:
            bad.append(f"CRC hatası={self.crc_ref.crc_errors}")
        if self.lost:
            bad.append(f"kayıp tur={self.lost}")
        if self.oor_total:
            bad.append(f"menzil dışı={self.oor_total}")
        if not bad:
            return "VERİ TEMİZ ✓", 'ok'
        return "SORUN: " + ", ".join(bad), 'bad'


# ===========================================================================
# Occupancy grid (log-odds) — NumPy vektörize ışın izleme.
# Işınlar res*0.6 adımlarla örneklenir; bir turda her hücre EN FAZLA BİR KEZ
# güncellenir (np.unique) -> hem hızlı hem sensöre yakın hücrelerin yüzlerce
# ışınla aşırı-güvenli hale gelmesini önler (eski Bresenham per-ışın idi).
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

    # --- ortak yardımcılar (vektörize) -------------------------------------
    def _clip_to_map(self, ox, oy, xs, ys):
        """Işın uçlarını harita sınırına kırpar (yön korunur)."""
        np = self.np
        lim = self.half * 0.999
        dx, dy = xs - ox, ys - oy
        t = np.ones_like(dx)
        with np.errstate(divide='ignore', invalid='ignore'):
            tx = np.where(dx > 0, (lim - ox) / dx,
                          np.where(dx < 0, (-lim - ox) / dx, np.inf))
            ty = np.where(dy > 0, (lim - oy) / dy,
                          np.where(dy < 0, (-lim - oy) / dy, np.inf))
        t = np.minimum(t, np.minimum(tx, ty))
        t = np.clip(t, 0.0, 1.0)
        return ox + dx * t, oy + dy * t

    def _flat_cells_along(self, ox, oy, xs, ys, shorten_m=0.0):
        """Tüm ışınlar boyunca örnekleme noktalarının benzersiz düz hücre
        indekslerini döner. shorten_m: ucu bu kadar KISALT (engel hücresine
        boş yazmamak için)."""
        np = self.np
        dx, dy = xs - ox, ys - oy
        r = np.hypot(dx, dy)
        keep = r > 1e-9
        if not keep.any():
            return None
        dx, dy, r = dx[keep], dy[keep], r[keep]
        if shorten_m > 0.0:
            scale = np.maximum(r - shorten_m, 0.0) / r
            dx, dy, r = dx * scale, dy * scale, r * scale
        step = self.res * 0.6
        nsteps = np.maximum((r / step).astype(np.int64), 1)
        total = int(nsteps.sum())
        if total == 0:
            return None
        ray_idx = np.repeat(np.arange(len(r)), nsteps)
        starts = np.cumsum(nsteps) - nsteps
        si = np.arange(total) - np.repeat(starts, nsteps)
        t = (si + 0.5) / nsteps[ray_idx]
        px = ox + dx[ray_idx] * t
        py = oy + dy[ray_idx] * t
        cx = ((px + self.half) / self.res).astype(np.int64)
        cy = ((py + self.half) / self.res).astype(np.int64)
        ok = (cx >= 0) & (cx < self.n) & (cy >= 0) & (cy < self.n)
        if not ok.any():
            return None
        return np.unique(cy[ok] * self.n + cx[ok])

    def _end_cells(self, xs, ys):
        np = self.np
        cx = ((np.asarray(xs) + self.half) / self.res).astype(np.int64)
        cy = ((np.asarray(ys) + self.half) / self.res).astype(np.int64)
        ok = (cx >= 0) & (cx < self.n) & (cy >= 0) & (cy < self.n)
        if not ok.any():
            return None
        return np.unique(cy[ok] * self.n + cx[ok])

    # --- tur entegrasyonu ---------------------------------------------------
    def integrate_turn(self, ox, oy, occ_pts, free_pts):
        """occ_pts: ucu engel olan ışınlar (yol=boş, uç=dolu).
        free_pts: dönüşsüz ışınlar (uç DAHİL tüm yol boş, engel yazılmaz)."""
        np = self.np
        oc_in = (-self.half < ox < self.half) and (-self.half < oy < self.half)
        if not oc_in:
            return

        free_sets = []
        if occ_pts:
            xs = np.array([p[0] for p in occ_pts]); ys = np.array([p[1] for p in occ_pts])
            cxs, cys = self._clip_to_map(ox, oy, xs, ys)
            # yolun boşu: uçtan bir hücre geride dur (engel hücresini silme)
            f = self._flat_cells_along(ox, oy, cxs, cys, shorten_m=self.res)
            if f is not None:
                free_sets.append(f)
        if free_pts:
            xs = np.array([p[0] for p in free_pts]); ys = np.array([p[1] for p in free_pts])
            cxs, cys = self._clip_to_map(ox, oy, xs, ys)
            f = self._flat_cells_along(ox, oy, cxs, cys, shorten_m=0.0)
            if f is not None:
                free_sets.append(f)

        flat = self.log.reshape(-1)
        if free_sets:
            fidx = np.unique(np.concatenate(free_sets))
            flat[fidx] = np.maximum(flat[fidx] - L_FREE, -L_CLAMP)
        if occ_pts:
            # harita içindeki GERÇEK uçlara dolu yaz (kırpılmışlar engel değil)
            inb = [(p[0], p[1]) for p in occ_pts
                   if -self.half < p[0] < self.half and -self.half < p[1] < self.half]
            if inb:
                oidx = self._end_cells([p[0] for p in inb], [p[1] for p in inb])
                if oidx is not None:
                    flat[oidx] = np.minimum(flat[oidx] + L_OCC, L_CLAMP)

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


def _is_spike(mm, i, n):
    """İZOLE çok yakın okuma mı? SPIKE_NEAR_M'den yakın ve her iki komşusu da
    onu doğrulamıyorsa (geçersiz ya da SPIKE_RATIO kat uzaksa) gürültüdür.
    Gerçek yakın engeller (el, duvar) birden çok komşu sektörde görünür."""
    d = mm[i]
    if d == 0 or d > SPIKE_NEAR_M * 1000.0:
        return False
    for off in (-1, 1):
        nb = mm[(i + off) % n]
        if 0 < nb <= RANGE_MAX_MM and nb < d * SPIKE_RATIO:
            return False    # komşu doğruluyor -> gerçek engel
    return True


def decode_l(payload, pose, free_range_m=NO_RETURN_FREE_M):
    """(seq, n, sensor_pts, sensor_rs, world_pts, free_pts, valid, oor, noret,
    spikes, covered, rmin, ravg, rmax) döner; payload eksikse None.

    covered=True -> tur-seviyesi örtülme: dönüşsüz oran OCCLUDED_TURN_FRAC'ı
    aştı (sensör elle/örtüyle kapalı). Bu turda free_pts BOŞ döner ki haritaya
    sahte 'boş' yazılmasın."""
    if len(payload) < 4:
        return None
    seq, n = struct.unpack_from('<HH', payload, 0)
    if n == 0 or len(payload) < 4 + 2 * n:
        return None
    mm = struct.unpack_from(f'<{n}H', payload, 4)
    two_pi = 2.0 * math.pi
    px, py, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    sensor_pts, sensor_rs, world_pts, free_pts, rs = [], [], [], [], []
    oor = noret = spikes = 0

    # Tur-seviyesi örtülme kapısı: önce dönüşsüz oranı say
    noret_all = sum(1 for d in mm if d == 0)
    covered = (noret_all / n) >= OCCLUDED_TURN_FRAC

    for i, d in enumerate(mm):
        ang = i / n * two_pi
        if d == 0:                        # dönüş yok: uzak MI, örtülü MÜ?
            noret += 1
            if covered:
                continue                  # tur örtülü: boş-ışın YOK
            # sektör örtülme kapısı: yakın komşusu varsa boş-ışın İŞLEME
            if free_range_m > 0.0 and not _occluded(mm, i, n):
                fx = free_range_m * math.cos(ang)
                fy = free_range_m * math.sin(ang)
                free_pts.append((px + c * fx - s * fy, py + s * fx + c * fy))
            continue
        if d > RANGE_MAX_MM:
            oor += 1                      # T-mini üst menzili aşan 'şüpheli'
            continue
        if _is_spike(mm, i, n):
            spikes += 1                   # izole yakın gürültü -> atıldı
            continue
        r = d / 1000.0
        x, y = r * math.cos(ang), r * math.sin(ang)
        sensor_pts.append((x, y))
        sensor_rs.append(r)
        world_pts.append((px + c * x - s * y, py + s * x + c * y))
        rs.append(r)
    if rs:
        rmin, ravg, rmax = min(rs), sum(rs) / len(rs), max(rs)
    else:
        rmin = ravg = rmax = 0.0
    return (seq, n, sensor_pts, sensor_rs, world_pts, free_pts,
            len(rs), oor, noret, spikes, covered, rmin, ravg, rmax)


# ===========================================================================
# Canlı görselleştirme — koyu tema, mesafe-renkli anlık tarama, hizalı panel
# ===========================================================================
class Viewer:
    BG = '#14161a'; FG = '#d7dae0'; DIM = '#5c6370'
    OK = '#5fd07a'; WARN = '#e5c07b'; BAD = '#e06c75'

    def __init__(self, grid, baud):
        import matplotlib.pyplot as plt
        self.plt = plt
        self.grid = grid
        self.baud = baud
        plt.ion()
        self.fig = plt.figure(figsize=(12.5, 7), facecolor=self.BG)
        self.fig.canvas.manager.set_window_title('KARAN İKA — Lidar Doğrulama & Harita')
        gs = self.fig.add_gridspec(2, 2, width_ratios=[1.45, 1.0],
                                   height_ratios=[2.1, 1.0],
                                   left=0.055, right=0.985, top=0.94, bottom=0.07,
                                   wspace=0.16, hspace=0.22)

        # Sol: birikimli occupancy grid
        self.ax_map = self.fig.add_subplot(gs[:, 0], facecolor=self.BG)
        ext = [-grid.half, grid.half, -grid.half, grid.half]
        self.im = self.ax_map.imshow(grid.prob_image(), cmap='gray_r',
                                     origin='lower', extent=ext,
                                     vmin=0.0, vmax=1.0, interpolation='nearest')
        self.robot, = self.ax_map.plot([0], [0], marker=(3, 0, 0), ms=11,
                                       color=self.BAD, ls='none', label='araç')
        self.ax_map.set_title('Occupancy Grid  (koyu=engel, açık=boş)',
                              color=self.FG, fontsize=11)
        self._style_axes(self.ax_map)
        self.ax_map.set_xlabel('x [m]'); self.ax_map.set_ylabel('y [m]')
        self.ax_map.set_aspect('equal')

        # Sağ üst: anlık tarama (mesafeye göre renkli)
        self.ax_scan = self.fig.add_subplot(gs[0, 1], facecolor=self.BG)
        self.scan_sc = self.ax_scan.scatter([], [], s=6, c=[], cmap='turbo',
                                            vmin=0.0, vmax=12.0)
        self.ax_scan.plot([0], [0], marker=(3, 0, 0), ms=9, color=self.BAD)
        for rr in (3, 6, 9, 12):     # menzil halkaları
            self.ax_scan.add_patch(self.plt.Circle((0, 0), rr, fill=False,
                                                   color=self.DIM, lw=0.5, alpha=0.5))
        lim = 12.5
        self.ax_scan.set_xlim(-lim, lim); self.ax_scan.set_ylim(-lim, lim)
        self.ax_scan.set_aspect('equal')
        self._style_axes(self.ax_scan)
        self.ax_scan.set_title('Anlık Tarama (renk = mesafe)', color=self.FG, fontsize=11)

        # Sağ alt: istatistik paneli
        ax_txt = self.fig.add_subplot(gs[1, 1]); ax_txt.axis('off')
        self.txt = ax_txt.text(0.0, 1.0, '', va='top', ha='left',
                               family='monospace', fontsize=9, color=self.FG,
                               transform=ax_txt.transAxes)
        self.health_txt = ax_txt.text(0.0, 0.06, '', va='bottom', ha='left',
                                      family='monospace', fontsize=11,
                                      fontweight='bold', color=self.OK,
                                      transform=ax_txt.transAxes)

        self.fig.canvas.mpl_connect('key_press_event', self._on_key)
        self._rate_t = time.monotonic()
        self._rate_b = 0
        self.rate_bps = 0.0

    def _style_axes(self, ax):
        ax.tick_params(colors=self.DIM, labelsize=8)
        for sp in ax.spines.values():
            sp.set_color(self.DIM)
        ax.xaxis.label.set_color(self.DIM)
        ax.yaxis.label.set_color(self.DIM)
        ax.grid(True, color=self.DIM, alpha=0.15, lw=0.5)

    def _on_key(self, ev):
        if ev.key == 'c':
            self.grid.clear()
            print("[viewer] harita temizlendi")
        elif ev.key == 's':
            name = time.strftime('harita_%Y%m%d_%H%M%S.png')
            self.fig.savefig(name, dpi=150, facecolor=self.BG)
            print(f"[viewer] kaydedildi: {name}")
        elif ev.key == 'q':
            self.plt.close(self.fig)

    def alive(self):
        return self.plt.fignum_exists(self.fig.number)

    def update(self, stats: Stats, parser: FrameParser, sensor_pts, sensor_rs, pose):
        import numpy as np
        # bant genişliği (kayan pencere)
        now = time.monotonic()
        dt = now - self._rate_t
        if dt >= 1.0:
            self.rate_bps = (parser.bytes_total - self._rate_b) / dt
            self._rate_t, self._rate_b = now, parser.bytes_total

        self.im.set_data(self.grid.prob_image())
        self.robot.set_data([pose[0]], [pose[1]])
        if sensor_pts:
            self.scan_sc.set_offsets(np.asarray(sensor_pts))
            self.scan_sc.set_array(np.asarray(sensor_rs))
        else:
            self.scan_sc.set_offsets(np.empty((0, 2)))

        budget = self.baud / 10.0          # 8N1: baud/10 = byte/s
        pct = 100.0 * self.rate_bps / budget if budget else 0.0
        el = now - stats.t0
        lines = [
            f"SÜRE  {el:7.1f} s    BANT {self.rate_bps:5.0f} B/s  %{pct:3.0f} @ {self.baud}",
            f"TUR L {stats.frames_l:7d}    HIZ {stats.hz():4.1f} Hz     POZ P {stats.frames_p}",
            f"CRC   {parser.crc_errors:7d}    KAYIP {stats.lost:<6d} SIRA DIŞI {stats.out_of_order}",
            f"SPIKE {stats.spikes_total:7d}    MENZİL DIŞI {stats.oor_total:<5d} ÖRTÜLÜ TUR {stats.covered_turns}",
            f"SON   seq={stats.seq:<5d} N={stats.n} geçerli={stats.valid} "
            f"dönüşsüz={stats.noret}",
            f"MESAFE min {stats.rmin:5.2f}  ort {stats.ravg:5.2f}  maks {stats.rmax:5.2f} m",
            f"POZ    x={pose[0]:6.2f}  y={pose[1]:6.2f}  yaw={math.degrees(pose[2]):5.0f}°",
            "klavye: c=temizle  s=PNG  q=çık",
        ]
        self.txt.set_text("\n".join(lines))

        msg, level = stats.health()
        color = {'ok': self.OK, 'warn': self.WARN,
                 'bad': self.BAD, 'wait': self.DIM}[level]
        self.health_txt.set_text(f">>> {msg} <<<")
        self.health_txt.set_color(color)

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
    last_sensor_rs = []
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
                        (seq, n, spts, srs, wpts, fpts, valid, oor,
                         noret, spikes, covered, rmin, ravg, rmax) = dec
                        stats.on_l(seq, n, valid, oor, noret, len(fpts),
                                   spikes, covered, rmin, ravg, rmax)
                        grid.integrate_turn(pose[0], pose[1], wpts, fpts)
                        last_sensor_pts = spts
                        last_sensor_rs = srs
            now = time.monotonic()
            if now - last_draw >= REDRAW_PERIOD:
                last_draw = now
                viewer.update(stats, parser, last_sensor_pts, last_sensor_rs, pose)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        print(f"\nÖZET: L={stats.frames_l} tur, CRC hata={parser.crc_errors}, "
              f"kayıp={stats.lost}, menzil dışı={stats.oor_total}, "
              f"spike={stats.spikes_total}, örtülü tur={stats.covered_turns}")


if __name__ == '__main__':
    main()
