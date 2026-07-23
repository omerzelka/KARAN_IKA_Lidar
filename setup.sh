#!/usr/bin/env bash
# ============================================================================
# setup.sh — JETSON/Ubuntu TEK SEFERLİK kurulum (yeni makine hazırlama)
#
# Yaptıkları:
#   1. ROS 2 kurulumunu doğrular (kurmaz — dağıtıma göre elle kurulmalı)
#   2. Gerekli apt paketlerini kurar (colcon, pyserial)
#   3. Seri port izni verir (dialout grubu)
#   4. İsteğe bağlı: /dev/ttyTHS1'i meşgul eden nvgetty'yi kapatır (Jetson)
#
# Kullanım:  ./setup.sh            # temel kurulum
#            ./setup.sh --nvgetty  # + nvgetty'yi kapat (STM32 köprüsü için)
# ============================================================================
set -e

echo "=== KARAN_IKA_Lidar kurulum ==="

# --- 1) ROS 2 var mı? -------------------------------------------------------
ROS_SETUP=""
for d in /opt/ros/*/setup.bash; do
    [ -f "$d" ] && ROS_SETUP="$d" && break
done
if [ -z "$ROS_SETUP" ]; then
    echo "HATA: /opt/ros altında ROS 2 bulunamadı."
    echo "Önce ROS 2 kur (örn. Jazzy): https://docs.ros.org/en/jazzy/Installation.html"
    exit 1
fi
echo "ROS 2 bulundu: $ROS_SETUP"

# --- 2) Paketler ------------------------------------------------------------
# Ubuntu 24.04'te pip sistem kurulumira kapalı (PEP 668) -> apt kullanılır.
echo "apt paketleri kuruluyor (sudo ister)..."
sudo apt-get update -qq
sudo apt-get install -y python3-colcon-common-extensions python3-serial

# --- 3) Seri port izni ------------------------------------------------------
if groups "$USER" | grep -q '\bdialout\b'; then
    echo "dialout grubu: zaten üye ✓"
else
    sudo usermod -aG dialout "$USER"
    echo "dialout grubuna eklendi — ETKİLİ OLMASI İÇİN OTURUMU KAPAT-AÇ!"
fi

# --- 4) nvgetty (isteğe bağlı, yalnız Jetson'da ttyTHS1 için) ----------------
if [ "$1" = "--nvgetty" ]; then
    if systemctl list-unit-files 2>/dev/null | grep -q nvgetty; then
        sudo systemctl stop nvgetty || true
        sudo systemctl disable nvgetty || true
        echo "nvgetty kapatıldı (/dev/ttyTHS1 artık serbest)"
    else
        echo "nvgetty servisi yok (sorun değil)"
    fi
fi

echo
echo "=== Kurulum tamam. Sıradaki adımlar ==="
echo "  1) (dialout yeni eklendiyse) oturumu kapat-aç"
echo "  2) ./build.sh          # derle"
echo "  3) ./start.sh          # lidar + köprüyü başlat"
