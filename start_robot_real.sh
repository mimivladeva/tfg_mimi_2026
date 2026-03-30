#!/bin/bash
set -e

WORKSPACE=~/tfg_mimi_2026
SETUP="$WORKSPACE/install/setup.bash"

echo "🛑 Cerrando procesos antiguos..."

pkill -f rviz2 2>/dev/null || true
pkill -f nav2 2>/dev/null || true
pkill -f amcl 2>/dev/null || true
pkill -f map_server 2>/dev/null || true
pkill -f lifecycle_manager 2>/dev/null || true
pkill -f esp32_reader 2>/dev/null || true
pkill -f nav2_supervisor 2>/dev/null || true
pkill -f screen 2>/dev/null || true
pkill -f minicom 2>/dev/null || true

# liberar puerto serie
sudo fuser -k /dev/ttyUSB0 2>/dev/null || true

sleep 2

echo "🔄 Entrando al workspace..."
cd "$WORKSPACE"

if [ ! -f "$SETUP" ]; then
  echo "🔨 Compilando workspace..."
  colcon build --symlink-install
fi

echo "✅ Cargando entorno..."
source "$SETUP"

# ================================
# 1. MAPA + LOCALIZACIÓN
# ================================
echo "🗺️ Lanzando mapa y AMCL..."
gnome-terminal -- bash -c "
source $SETUP
ros2 launch aidguide_04_provide_map aidguide_04_provide_map.launch.py
exec bash
"

echo "⏳ Esperando nodo AMCL..."
until ros2 node list 2>/dev/null | grep -q "/amcl"; do
  sleep 1
done

sleep 3

echo "📍 Publicando pose inicial..."
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
header: {frame_id: map},
pose: {
  pose: {
    position: {x: 0.0, y: 0.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  },
  covariance: [
    0.25,0,0,0,0,0,
    0,0.25,0,0,0,0,
    0,0,0,0,0,0,
    0,0,0,0,0,0,
    0,0,0,0,0,0,
    0,0,0,0,0,0
  ]
}}"

echo "⏳ Esperando /amcl_pose..."
until ros2 topic echo /amcl_pose --once >/dev/null 2>&1; do
  sleep 1
done

echo "✅ Localización correcta"

# ================================
# 2. ESPERAR TF REAL (robusto)
# ================================
echo "⏳ Esperando TF map -> base_link..."
until ros2 run tf2_ros tf2_echo map base_link 2>/dev/null | grep -q "Translation"; do
  sleep 1
done
echo "✅ TF disponible"

# ================================
# 3. NAV2 (NAVEGACIÓN)
# ================================
echo "🚀 Lanzando navegación..."


echo "⏳ Esperando bt_navigator..."
until ros2 node list 2>/dev/null | grep -q "/bt_navigator"; do
  sleep 1
done

echo "⏳ Esperando acción follow_waypoints..."
until ros2 action list 2>/dev/null | grep -q "follow_waypoints"; do
  sleep 1
done

echo "✅ Nav2 completamente operativo"

sleep 2



echo "🧠 Lanzando supervisor Nav2..."
gnome-terminal -- bash -c "
source $SETUP
ros2 run aidguide_04_esp_bridge nav2_supervisor
exec bash
"
# ================================
# 4. ESP32 + SUPERVISOR
# ================================
echo "🔌 Lanzando lector ESP32..."
gnome-terminal -- bash -c "
source $SETUP
ros2 run aidguide_04_esp_bridge esp32_reader
exec bash
"

sleep 2


echo "🎉 Sistema listo"
