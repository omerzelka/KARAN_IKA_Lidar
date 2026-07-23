#!/usr/bin/env bash
# ============================================================================
# start.sh — JETSON çalıştırma: lidar sürücüsü + STM32 seri köprüsü (+ nav)
#
# Kullanım:
#   ./start.sh              # lidar + köprü (varsayılan; Ctrl+C hepsini durdurur)
#   ./start.sh lidar        # yalnız lidar sürücüsü (/scan)
#   ./start.sh bridge       # yalnız STM32 seri köprüsü (/scan -> ttyTHS1)
#   ./start.sh nav          # yalnız navigasyon (/scan+/odom -> /cmd_vel)
#   ./start.sh all-nav      # lidar + köprü + navigasyon
#
# Ortam değişkenleriyle ayar (varsayılanlar parantezde):
#   BRIDGE_PORT=/dev/ttyTHS1   köprünün STM32'ye yazdığı port
#   NUM_POINTS=180             köprü sektör sayısı (2 derece çözünürlük)
#   ODOM_TOPIC=                boş değilse köprü poz ('P') çerçevesi de yollar
#     örn: ODOM_TOPIC=/odom ./start.sh
# ============================================================================
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ROS + workspace ortamı
if [ -z "$ROS_DISTRO" ]; then
    for d in /opt/ros/*/setup.bash; do
        [ -f "$d" ] && source "$d" && break
    done
fi
if [ ! -f "$ROOT/install/setup.bash" ]; then
    echo "HATA: install/ yok — önce ./build.sh çalıştır."
    exit 1
fi
source "$ROOT/install/setup.bash"

BRIDGE_PORT="${BRIDGE_PORT:-/dev/ttyTHS1}"
NUM_POINTS="${NUM_POINTS:-180}"
SCRIPTS="$ROOT/src/ydlidar_tmini_driver/scripts"

run_lidar()  { ros2 launch ydlidar_tmini_driver ydlidar.launch.py; }
run_nav()    { ros2 launch ika_navigation ika_navigation.launch.py; }
run_bridge() {
    local extra=()
    [ -n "$ODOM_TOPIC" ] && extra+=(-p "odom_topic:=$ODOM_TOPIC")
    python3 "$SCRIPTS/scan_serial_bridge.py" --ros-args \
        -p "port:=$BRIDGE_PORT" -p "num_points:=$NUM_POINTS" "${extra[@]}"
}

# Çoklu mod: hepsini arka planda başlat, Ctrl+C ile tümünü öldür
run_group() {
    trap 'echo; echo "Durduruluyor..."; kill 0' INT TERM
    for job in "$@"; do
        "$job" &
        sleep 2   # sürücünün porta bağlanmasına fırsat ver
    done
    wait
}

MODE="${1:-all}"
case "$MODE" in
    lidar)   run_lidar ;;
    bridge)  run_bridge ;;
    nav)     run_nav ;;
    all)     echo "[start] lidar + köprü (port=$BRIDGE_PORT N=$NUM_POINTS)"
             run_group run_lidar run_bridge ;;
    all-nav) echo "[start] lidar + köprü + navigasyon"
             run_group run_lidar run_bridge run_nav ;;
    *)  echo "Kullanım: $0 [lidar|bridge|nav|all|all-nav]"; exit 1 ;;
esac
