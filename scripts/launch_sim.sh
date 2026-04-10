#!/bin/bash

# TEKNOFEST 2026 Sürü İHA - PX4 + Gazebo 3 Drone Launch Script
# Kullanım: ./launch_sim.sh        (normal mod)
#           ./launch_sim.sh headless (GUI olmadan)

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PX4_DIR="$REPO_DIR/PX4-Autopilot"
ROOTFS="$PX4_DIR/build/px4_sitl_default/rootfs"
PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
NUM_DRONES=3
MODEL="gz_x500"

# Drone başlangıç pozisyonları (x,y,z,roll,pitch,yaw)
# Gazebo ENU: X=East, Y=North → Y boyunca diziyoruz (North ekseni)
POSES=(
    "0,0,0,0,0,0"
    "0,3,0,0,0,0"
    "0,6,0,0,0,0"
)

HEADLESS=false
if [[ "$1" == "headless" ]]; then
    HEADLESS=true
fi

# Renk kodları
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}  Sürü İHA Simülasyon Başlatıcı${NC}"
echo -e "${CYAN}  PX4 + Gazebo - 3 Drone${NC}"
echo -e "${CYAN}========================================${NC}"

# PX4 dizini kontrolü
if [ ! -d "$PX4_DIR" ]; then
    echo -e "${RED}HATA: PX4-Autopilot bulunamadı: $PX4_DIR${NC}"
    exit 1
fi

# Build kontrolü
if [ ! -f "$PX4_BIN" ]; then
    echo -e "${YELLOW}PX4 build edilmemiş, build ediliyor...${NC}"
    cd "$PX4_DIR" && make px4_sitl gz_x500
fi

# rcS dosyası kontrolü
if [ ! -f "$ROOTFS/etc/init.d-posix/rcS" ]; then
    echo -e "${RED}HATA: rcS bulunamadı: $ROOTFS/etc/init.d-posix/rcS${NC}"
    echo -e "${YELLOW}Çözüm: cd $PX4_DIR && make px4_sitl gz_x500${NC}"
    exit 1
fi

# PX4 Gazebo ortam değişkenlerini yükle (pluginler, modeller, world)
GZ_ENV="$ROOTFS/gz_env.sh"
if [ -f "$GZ_ENV" ]; then
    source "$GZ_ENV"
    echo -e "${GREEN}Gazebo ortam değişkenleri yüklendi.${NC}"
else
    echo -e "${RED}HATA: gz_env.sh bulunamadı: $GZ_ENV${NC}"
    exit 1
fi

# Sürü İHA arena modellerini Gazebo path'e ekle
SWARM_MODELS="$REPO_DIR/gazebo/models"
export GZ_SIM_RESOURCE_PATH="${SWARM_MODELS}:${GZ_SIM_RESOURCE_PATH}"
echo -e "${GREEN}Arena model yolu eklendi: $SWARM_MODELS${NC}"

# Arena world dosyası
SWARM_WORLD="$REPO_DIR/gazebo/worlds/swarm_arena.sdf"

# Önceki PX4/Gazebo süreçlerini temizle
echo -e "${YELLOW}Önceki süreçler temizleniyor...${NC}"
pkill -9 -f "px4.*sitl" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
sleep 2

# Log dizini
LOG_DIR="$REPO_DIR/logs"
rm -rf "$LOG_DIR"
mkdir -p "$LOG_DIR"

cleanup() {
    echo -e "\n${YELLOW}Kapatılıyor...${NC}"
    for pid in "${PX4_PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    kill "$GZ_PID" 2>/dev/null || true
    pkill -f "px4" 2>/dev/null || true
    pkill -f "gz sim" 2>/dev/null || true
    echo -e "${GREEN}Temizlik tamamlandı.${NC}"
    exit 0
}
trap cleanup SIGINT SIGTERM

PX4_PIDS=()

# 1) Gazebo'yu başlat
echo -e "${GREEN}[1/4] Gazebo başlatılıyor...${NC}"

if $HEADLESS; then
    gz sim -r -s "$SWARM_WORLD" > "$LOG_DIR/gazebo.log" 2>&1 &
else
    gz sim -r "$SWARM_WORLD" > "$LOG_DIR/gazebo.log" 2>&1 &
fi
GZ_PID=$!
sleep 4

# 2) 3 PX4 instance başlat
for i in $(seq 0 $((NUM_DRONES - 1))); do
    echo -e "${GREEN}[$((i + 2))/4] Drone $((i + 1)) başlatılıyor (instance $i)...${NC}"

    POSE="${POSES[$i]}"
    INSTANCE_FLAG=""
    if [ "$i" -gt 0 ]; then
        INSTANCE_FLAG="-i $i"
    fi

    # PX4 binary rootfs dizininden çalıştırılmalı
    cd "$ROOTFS"
    PX4_SYS_AUTOSTART=4001 \
    PX4_GZ_MODEL_POSE="$POSE" \
    PX4_GZ_MODEL=x500 \
    PX4_SIM_MODEL="$MODEL" \
    PX4_INSTANCE=$i \
    GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH" \
    GZ_SIM_SYSTEM_PLUGIN_PATH="$GZ_SIM_SYSTEM_PLUGIN_PATH" \
    "$PX4_BIN" -d -s etc/init.d-posix/rcS $INSTANCE_FLAG \
        > "$LOG_DIR/drone_$i.log" 2>&1 &

    PX4_PIDS+=($!)
    sleep 3
done

# 3) Bağlantı bekle
echo -e "${GREEN}[4/4] Drone'lar başlatılıyor, bağlantı bekleniyor...${NC}"
sleep 5

# Durum raporu
echo ""
echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}  Simülasyon Aktif!${NC}"
echo -e "${CYAN}========================================${NC}"
echo -e "${GREEN}  Drone sayısı : $NUM_DRONES${NC}"
echo -e "${GREEN}  Gazebo PID   : $GZ_PID${NC}"
echo ""
echo -e "${YELLOW}  MAVLink Portları:${NC}"
for i in $(seq 0 $((NUM_DRONES - 1))); do
    PORT=$((14540 + i))
    echo -e "  Drone $((i + 1)): udp://:$PORT"
done
echo ""
echo -e "${YELLOW}  QGroundControl otomatik bağlanacaktır.${NC}"
echo -e "${YELLOW}  swarm.py çalıştırmak için başka terminalde:${NC}"
echo -e "${CYAN}    cd $REPO_DIR && python3 main.py${NC}"
echo ""
echo -e "${RED}  Durdurmak için: Ctrl+C${NC}"
echo -e "${CYAN}========================================${NC}"

# Bekle
wait
