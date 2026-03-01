#!/bin/bash

echo "🛑 Cerrando procesos antiguos..."
pkill -f gz 2>/dev/null
pkill -f nav2 2>/dev/null
pkill -f rviz 2>/dev/null
pkill -f robot_state_publisher 2>/dev/null
pkill -f ros_gz_bridge 2>/dev/null
pkill -f ros2 2>/dev/null

sleep 3

WORKSPACE=~/tfg_mimi_2026
SETUP="$WORKSPACE/install/setup.bash"

echo "🔄 Entrando al workspace..."
cd $WORKSPACE || exit 1

echo "🧹 Limpiando build..."
rm -rf build install log

echo "🔨 Construyendo simulación..."
colcon build --packages-select aidguide_sim --symlink-install
source $SETUP

echo "🚀 Lanzando simulación..."
gnome-terminal -- bash -c "
source $SETUP;
ros2 launch aidguide_sim sim.launch.py;
exec bash"

echo "⏳ Esperando /clock..."
until ros2 topic list 2>/dev/null | grep -q "/clock"; do
    sleep 1
done

echo "⏳ Esperando /odom..."
until ros2 topic list 2>/dev/null | grep -q "/odom"; do
    sleep 1
done

echo "⏳ Esperando /tf..."
until ros2 topic list 2>/dev/null | grep -q "/tf"; do
    sleep 1
done

echo "✅ Simulación estable."
sleep 3

echo "🔨 Construyendo localization..."
colcon build --packages-select aidguide_04_provide_map --symlink-install
source $SETUP

echo "🗺️ Lanzando provide_map..."
gnome-terminal -- bash -c "
source $SETUP;
ros2 launch aidguide_04_provide_map aidguide_04_provide_map.launch.py;
exec bash"

echo "⏳ Esperando nodo AMCL..."
until ros2 node list 2>/dev/null | grep -q "/amcl"; do
    sleep 1
done

sleep 3

echo "📍 Publicando pose inicial (con tiempo sim válido)..."

ros2 topic pub --once --qos-reliability reliable /initialpose geometry_msgs/PoseWithCovarianceStamped "
header:
  frame_id: 'map'
pose:
  pose:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  covariance: [0.25, 0, 0, 0, 0, 0,
               0, 0.25, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0]"

echo "⏳ Esperando /amcl_pose..."
until ros2 topic echo /amcl_pose --once >/dev/null 2>&1; do
    sleep 1
done

echo "✅ Localización correcta."
sleep 2

echo "🚀 Lanzando navegación..."
gnome-terminal -- bash -c "
source $SETUP;
ros2 launch aidguide_04_provide_map waypoints_navigation.launch.py;
exec bash"

echo "⏳ Esperando bt_navigator..."
until ros2 node list 2>/dev/null | grep -q "/bt_navigator"; do
    sleep 1
done

echo "⏳ Esperando lifecycle active..."
until ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q active; do
    sleep 1
done

echo "✅ Navegación activa."
sleep 2

echo "🤖 Ejecutando cliente..."
gnome-terminal -- bash -c "
source $SETUP;
ros2 run aidguide_04_provide_map waypoint_follower_client;
exec bash"

echo "🎉 Sistema completamente iniciado."
