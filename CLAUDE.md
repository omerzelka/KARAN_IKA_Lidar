# CLAUDE.md — KARAN_IKA_Lidar

İKA (İnsansız Kara Aracı) için 2D lidar tabanlı algılama + navigasyon çalışma
alanı (ROS 2). Bu dosya, kod tabanında çalışan Claude/geliştirici için hızlı
referanstır. Yorumlar ve doküman **Türkçe**; sürücüler **SDK'sız, sıfırdan**
yazılmıştır.

## Depo yapısı

```
KARAN_IKA_Lidar/                 # colcon workspace kökü (repo = workspace)
├── src/
│   ├── ydlidar_tmini_driver/    # YDLIDAR T-mini Plus sürücüsü (C++, ana lidar)
│   │   ├── src/ + include/       #   protocol, serial_transport, driver_node
│   │   ├── launch/ydlidar.launch.py
│   │   └── scripts/              #   /scan tüketen yardımcı Python araçları (aşağıda)
│   └── ika_navigation/          # VFH+ / Kalman / Stanley navigasyon düğümü (C++)
│       └── launch/ika_navigation.launch.py
├── firmware/
│   └── stm32f4_uart_relay/      # STM32F4 UART köprü firmware'i (Jetson->PC relay)
└── CLAUDE.md
```

> Not: Eski `rplidar_a2m12_driver` paketi kaldırıldı; lidar artık YDLIDAR T-mini
> Plus. Git geçmişinde RPLidar sürücüsü referans olarak bulunabilir.

## Derleme & çalıştırma (Jetson / Ubuntu, ROS 2)

Jetson'da repo yolu: `~/lidar/KARAN_IKA_Lidar` (workspace = repo kökü).

```bash
cd ~/lidar/KARAN_IKA_Lidar
colcon build --packages-select ydlidar_tmini_driver ika_navigation
source install/setup.bash

# 1. terminal: lidar sürücüsü  -> /scan
ros2 launch ydlidar_tmini_driver ydlidar.launch.py
# 2. terminal: navigasyon      -> /cmd_vel  (odom + /scan tüketir)
ros2 launch ika_navigation ika_navigation.launch.py
```

**Deploy akışı:** Kod Windows'ta (`C:\Users\ZELKA\ika_ws\KARAN_IKA_Lidar`)
düzenlenir → `git push origin main` → Jetson'da `git pull` → `colcon build`.
Paket silindiyse Jetson'da `rm -rf build install log` ile temiz derle.

## Lidar sürücüsü (`ydlidar_tmini_driver`)

YDLIDAR T-mini Plus (**ToF**, 360°, ~0.02–12 m, 6–12 Hz, ölçüldü ~6.1 Hz).
Seri: **230400 baud, 8N1**. `sensor_msgs/LaserScan` olarak `/scan`'e yayınlar.

**Protokol (Slamtec'ten farkı):** komutlar `0xA5 0x60`(SCAN)/`0x65`(STOP)/
`0x91`(HEALTH); veri paketleri `0xAA 0x55` başlıklı, çok noktalı; açı FSA/LSA
arası interpolasyon. Ayrıntı: `src/ydlidar_tmini_driver/ACIKLAMA`.

**Doğrulanmış/kritik parametreler** (launch'ta):

| Parametre | Değer | Not |
|-----------|-------|-----|
| `baud_rate` | 230400 | T-mini Plus sabiti |
| `distance_scale_mm` | 0.25 | ham/4 = mm — **donanımda doğrulandı** |
| `sample_bytes` | 2 | mesafe-only; "Bozuk paket" akarsa 3 dene |
| `range_min` / `range_max` | 0.05 / 12.0 | sürücü bu aralık dışını **eler** (+inf) |
| `angle_correction` | false | ToF'ta kapalı |
| `invert_angle` | true | CW(lidar) → CCW(REP-103) |

⚠️ **`/scan` Best-Effort (SensorDataQoS) yayınlar.** Abone olan HER düğüm
(rclpy dahil) `qos_profile_sensor_data` / Best-Effort kullanmalı; yoksa mesaj
GELMEZ. `ros2 topic echo/hz` otomatik uyumlanır.

## Navigasyon (`ika_navigation`)

`/scan` + `/odom` → `/cmd_vel`. Üç parça: **VFH+** (engelden kaçınma),
**Kalman** (sabit-hız 2B, engel takibi), **Stanley** (waypoint takibi). Odom
yoksa otomatik sadece VFH+'ya düşer. Kalibrasyon: `scan_angle_offset` (fiziksel
ön ile ışın 0'ını hizala), `waypoints`, hız/dönüş limitleri — launch'tan.

## Veri dışa aktarma / uzaktan haritalama

Lidar verisini uzaktaki bilgisayara taşımanın 3 yolu (`scripts/` altında):

1. **ROS 2 native (en temiz, uzak makine ROS ise):** iki makinede aynı
   `ROS_DOMAIN_ID` → `/scan` uzakta görünür → `slam_toolbox` native tüketir.
   Kod gerekmez.
2. **TCP + JSON (uzak makine ROS'suz):** `scan_json_bridge.py` (Jetson TCP
   sunucu :9000, NDJSON) + `scan_receiver.py` (uzak, ROS'suz). Yüksek bant
   genişliği (LAN/WiFi).
3. **Seri → STM32 → PC @ 57600 (telemetri/gömülü hat):**
   `scan_serial_bridge.py` (Jetson) → **STM32F4 relay** (`firmware/`) → 57600
   UART → `scan_serial_receiver.py` (PC).
   - **Binary çerçeve** (little-endian, satır değil sync+len+CRC ile çerçevelenir):
     `[0xAA 0x55][type:u8][len:u16][payload][crc:u16]`. `crc` = type+len+payload
     üstünde **CRC-16/CCITT-FALSE**. `type 'L'(0x4C)`: `seq:u16 + N:u16 + N×u16`
     (mesafe **mm**, 0 = geçersiz). `type 'P'(0x50)`: `x_mm:i32 + y_mm:i32 +
     yaw_cdeg:i32` (santiderece = derece·100). Bridge ↔ receiver `crc16_ccitt`'i
     aynı olmalı.
   - **57600 bütçesi: ≈5760 byte/s.** Bridge 360° → `num_points` (varsayılan
     180 = 2°) sektöre seyreltir; her sektörde en yakın geçerli mesafe tutulur.
     Binary N=180 @ 6 Hz ≈ **~2.3 KB/s (%41)** — tam çözünürlük artık sığar
     (eski ASCII format %80 idi). Yine de düşerse `num_points`↓ ya da
     `publish_every`↑.
   - Not: STM32 relay **byte-transparan** (halka tampon, içeriği yorumlamaz);
     binary baytlar (`$`, `\r`, `\n` dahil) sorunsuz geçer.

`scan_serial_receiver.py` artık gelen turları **log-odds occupancy grid**'e
işleyip (sensörden ışın izleme: yol=boş, uç=dolu) matplotlib ile **canlı** çizer;
poz ('P') varsa turları dünya çerçevesinde birleştirir. Harita paramları dosya
başında (`GRID_RES_M`, `GRID_HALF_M`, `L_OCC/L_FREE/L_CLAMP`). `--nomap` ile
haritasız sadece log.

`scripts/scan_check.py`: GUI'siz canlı doğrulama (en yakın engel + açı yazar).
Tüm Python abone araçları Best-Effort QoS kullanır. Bağımlılık: `pyserial`
(seri araçlar); `numpy`+`matplotlib` (`scan_serial_receiver.py` canlı haritası).

## Konvansiyonlar & tuzaklar

- **Dil:** kod yorumları, log'lar, doküman Türkçe. Kod stili: C++17, ament_cmake,
  `-Wall -Wextra -Wpedantic -O2`. Sürücüler SDK'sız, POSIX `termios2` seri katmanı
  (isteğe bağlı baud için).
- **CRLF:** dosyalar Windows'ta üretiliyor. Jetson'da launch/py hatası →
  `sudo apt install dos2unix && find src -type f | xargs dos2unix`.
- **Seri izin:** `sudo usermod -aG dialout $USER` (oturum kapat-aç).
- **Lidar portu:** `/dev/ttyUSB0` (CP210x/CH340). Motor SCAN komutuyla döner
  (DTR gerekmez; `use_dtr_motor` istisna).
- **Push/commit:** kullanıcı onaylamadan push edilmez (dışa gönderim).
- Maintainer: Omer-Metehan <promerz10@gmail.com>. Lisans: MIT.
```
