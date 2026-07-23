#!/usr/bin/env bash
# ============================================================================
# build.sh — JETSON/Ubuntu derleme (her git pull sonrası çalıştır)
#
# Kullanım:  ./build.sh           # iki paketi derle
#            ./build.sh --clean   # build/install/log silip temiz derle
#                                  (paket silindi/yeniden adlandıysa gerekir)
# ============================================================================
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

# ROS ortamı
if [ -z "$ROS_DISTRO" ]; then
    for d in /opt/ros/*/setup.bash; do
        [ -f "$d" ] && source "$d" && break
    done
fi
if [ -z "$ROS_DISTRO" ]; then
    echo "HATA: ROS 2 bulunamadı. Önce ./setup.sh çalıştır."
    exit 1
fi
echo "ROS 2 $ROS_DISTRO ile derleniyor..."

if [ "$1" = "--clean" ]; then
    echo "Temiz derleme: build/ install/ log/ siliniyor"
    rm -rf build install log
fi

colcon build --packages-select ydlidar_tmini_driver ika_navigation

echo
echo "=== Derleme tamam ==="
echo "Çalıştırmak için:  ./start.sh"
echo "(elle kullanım için: source install/setup.bash)"
