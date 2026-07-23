# KARAN_IKA_Lidar

İKA (İnsansız Kara Aracı) için **YDLIDAR T-mini Plus** tabanlı 2D algılama +
navigasyon çalışma alanı (ROS 2). Sürücüler SDK'sız, sıfırdan yazılmıştır;
tüm kod yorumları ve doküman Türkçedir.

```
Lidar (T-mini Plus) ──► Jetson (ROS 2) ──► STM32 relay ──► PC (harita + doğrulama)
        USB              /scan → binary      UART 57600        canlı occupancy grid
```

## İçerik

| Bileşen | Ne yapar |
|---|---|
| `src/ydlidar_tmini_driver/` | T-mini Plus sürücüsü (C++): seri protokolü çözer, `/scan` yayınlar |
| `src/ika_navigation/` | VFH+ (engel kaçınma) + Kalman (engel takibi) + Stanley (waypoint) → `/cmd_vel` |
| `firmware/stm32f4_uart_relay/` | STM32 UART köprüsü: Jetson→PC byte-transparan relay (Nucleo-F446RE'de doğrulandı) |
| `scripts/` (sürücü içinde) | `/scan` tüketen PC/Jetson yardımcı araçları (aşağıda) |

## Hızlı başlangıç — script'lerle (önerilen)

Yeni bir makinede üç komut yeter:

```bash
# JETSON (Ubuntu + ROS 2 kurulu olmalı)
./setup.sh --nvgetty    # tek seferlik: apt paketleri + dialout + nvgetty kapat
./build.sh              # derleme (git pull sonrası da bunu çalıştır)
./start.sh              # lidar + STM32 köprüsü (Ctrl+C hepsini durdurur)
```

`./start.sh` modları: `lidar` | `bridge` | `nav` | `all` (varsayılan) | `all-nav`.
Ayarlar ortam değişkeniyle: `ODOM_TOPIC=/odom BRIDGE_PORT=/dev/ttyTHS1 ./start.sh`

```bat
:: WINDOWS PC (harita/doğrulama)
viewer.bat COM6
```

> Not: `.gitattributes` `*.sh`/`*.py` dosyalarını her zaman **LF** ile checkout
> eder — Jetson'da artık `dos2unix` gerekmez.

## Hızlı başlangıç — elle (Jetson, ROS 2)

```bash
cd ~/lidar/KARAN_IKA_Lidar
source /opt/ros/<distro>/setup.bash        # örn. jazzy
colcon build --packages-select ydlidar_tmini_driver ika_navigation
source install/setup.bash

# Terminal 1: lidar sürücüsü -> /scan
ros2 launch ydlidar_tmini_driver ydlidar.launch.py
# Terminal 2: navigasyon -> /cmd_vel
ros2 launch ika_navigation ika_navigation.launch.py
```

Lidar: `/dev/ttyUSB0`, **230400 8N1**. Seri izin: `sudo usermod -aG dialout $USER`.

> ⚠️ `/scan` **Best-Effort (SensorDataQoS)** yayınlar — abone olan her düğüm de
> Best-Effort kullanmalı, yoksa mesaj gelmez.

## Lidar verisini PC'ye taşıma (3 yol)

1. **ROS 2 native** — iki makine aynı `ROS_DOMAIN_ID` → `/scan` uzakta görünür.
2. **TCP + JSON** — `scan_json_bridge.py` (Jetson, :9000) + `scan_receiver.py` (PC, ROS'suz).
3. **UART → STM32 → PC** — düşük bant, gömülü telemetri hattı (aşağıda).

### Yol 3: STM32 relay zinciri

```
Jetson ttyTHS1 TX ──► PA10 (USART1_RX)  STM32  PA2 (USART2_TX) ──► PC
                        Nucleo-F446RE'de USART2 = ST-Link USB (VCP) → ekstra adaptör gerekmez
GND ── GND ── GND   (ortak GND şart!)
```

- **Jetson:** `python3 scripts/scan_serial_bridge.py --ros-args -p port:=/dev/ttyTHS1 -p num_points:=180`
- **STM32:** `firmware/stm32f4_uart_relay/` — kurulum için içindeki README.
- **PC:** `python scripts/scan_map_viewer.py COM6` (aşağıda).

Hat **binary çerçeve** taşır (JSON değil — 57600'e sığması için):
`[0xAA 0x55][type:u8][len:u16][payload][crc16:CCITT-FALSE]`,
`'L'`=lidar turu (seq + N×u16 mm), `'P'`=poz (x,y,yaw). N=180 @ 6 Hz ≈ %41 bant.

## PC araçları (ROS gerekmez, `pip install pyserial numpy matplotlib`)

| Araç | Ne yapar |
|---|---|
| **`scan_map_viewer.py`** | **Doğrulama + haritalama uygulaması:** canlı occupancy grid, anlık tarama görünümü ve veri sağlığı paneli (CRC hatası, kayıp tur/seq boşluğu, tur Hz'i, bant genişliği %, menzil dışı ölçüm). Klavye: `c` temizle, `s` PNG kaydet, `q` çık. |
| `scan_serial_receiver.py` | Sade alıcı + canlı harita; `--nomap` ile yalnız log. Kendi haritalama kodunu bağlamak için temel. |
| `scan_check.py` | GUI'siz hızlı kontrol (en yakın engel + açı). |

```bash
cd src/ydlidar_tmini_driver/scripts
python scan_map_viewer.py COM6        # Windows (STLink VCP portu)
python3 scan_map_viewer.py /dev/ttyACM0   # Linux
```

İstatistik panelindeki **"VERİ TEMİZ ✓"** satırı; CRC hatası, kayıp tur ve
menzil dışı sayaçlarının tümü sıfırsa görünür — STM32 hattının doğruluğunu
bu panelden okuyabilirsin.

> **Not:** Dönüş alınamayan doğrultular (menzil >12 m, cam vb.) haritada
> "buraya kadar boş" olarak işlenir (`--freeray=<m>` ile ayarlanır,
> `--freeray=0` kapatır) — aksi halde o doğrultular hiç veri gelmemiş gibi
> gri kalırdı.

## Sorun giderme

| Belirti | Çözüm |
|---|---|
| Jetson'da launch/py sözdizimi hatası | CRLF: `sudo apt install dos2unix && find src -type f \| xargs dos2unix` |
| `/dev/ttyTHS1` açılamıyor | `sudo systemctl stop nvgetty && sudo systemctl disable nvgetty`, `dialout` grubu |
| PC'ye hiç veri gelmiyor | Ortak GND? TX→RX çapraz mı? Üç tarafta da 57600 mü? USART1 kesmesi açık mı? |
| CRC hataları akıyor | Kablo boyu/paraziti azalt; baud toleransını (clock) kontrol et |
| rclpy aracına `/scan` gelmiyor | QoS Best-Effort mu? (`qos_profile_sensor_data`) |

## Lisans & iletişim

MIT — Maintainer: Omer-Metehan-Alperen <promerz10@gmail.com>
