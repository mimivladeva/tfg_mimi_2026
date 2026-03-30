#!/bin/bash
set -e

WORKSPACE=~/tfg_mimi_2026
SETUP="$WORKSPACE/install/setup.bash"

echo "🧹 Limpiando procesos..."
pkill -f ros2 || true
sleep 2

echo "📦 Cargando entorno..."
source /opt/ros/jazzy/setup.bash
source "$SETUP"

# ==============================
# 1. ROBOT REAL (IMPORTANTE)
# ==============================
echo "🤖 Lanzando robot..."
gnome-terminal -- bash -c "
source /opt/ros/jazzy/setup.bash;
export TURTLEBOT3_MODEL=burger;
ros2 launch turtlebot3_bringup robot.launch.py;
exec bash
"

sleep 5

# ==============================
# 2. MAPA + AMCL
# ==============================
echo "🗺️ Lanzando mapa + AMCL..."
gnome-terminal -- bash -c "
source $SETUP;
ros2 launch aidguide_04_provide_map aidguide_04_provide_map.launch.py;
exec bash
"

sleep 5

# ==============================
# 3. INITIAL POSE
# ==============================
echo "📍 Enviando initial pose..."
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
header: {frame_id: map},
pose: {
  pose: {
    position: {x: 0.0, y: 0.0, z: 0.0},
    orientation: {w: 1.0}
  },
  covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0]
}}"

sleep 3

# ==============================
# 4. NAV2 (TU LAUNCH VIEJO)
# ==============================
echo "🚀 Lanzando navegación..."
gnome-terminal -- bash -c "
source $SETUP;
ros2 launch aidguide_04_provide_map waypoints_navigation.launch.py;
exec bash
"




echo "✅ SISTEMA LISTO"
