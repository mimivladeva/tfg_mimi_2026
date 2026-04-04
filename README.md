# 🤖 AidGuide Navigation System (ROS2 + ESP32)

Sistema de navegación autónoma basado en ROS 2 (Nav2) con control mediante ESP32 por UART.


## 📡 1. Conexión con el robot

ssh ubuntu@192.168.18.107
export ROS_DOMAIN_ID=12
echo $ROS_DOMAIN_ID



export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py



## 🔌 2. Servicio ESP32 UART

ssh ubuntu@192.168.18.107
export ROS_DOMAIN_ID=12
echo $ROS_DOMAIN_ID

cd ~/esp_uart_ws

colcon build --symlink-install
source install/setup.bash


source /opt/ros/jazzy/setup.bash
source ~/esp_uart_ws/install/setup.bash


ros2 run aidguide_04_esp_bridge esp_reader_uart

Comprobar comunicación:


ros2 topic echo /esp32/event


Si no funciona:

sudo systemctl restart esp32_uart.service


## 🛠️ 3. Compilar workspace


cd ~/tfg_mimi_2026

rm -rf build install log

colcon build --symlink-install

source install/setup.bash


## 🚀 4. Lanzar sistema completo (robot real)


./start_robot_real_uart.sh


Este script incluye:

* AMCL
* Nav2
* Waypoints
* Supervisor
* ESP32 bridge



## 🧪 5. Simulación


./start_simulation.sh


## 🎮 6. Comandos disponibles (ESP32)

| Comando    | Descripción                                |
| ---------- | ------------------------------------------ |
| NORMAL     | Velocidad normal                           |
| SLOW       | Velocidad reducida                         |
| FAST       | Velocidad alta                             |
| STOP       | Detiene el robot (sin cancelar navegación) |
| TURN_LEFT  | Giro a la izquierda                        |
| TURN_RIGHT | Giro a la derecha                          |
| ESTOP      | Parada de emergencia (cancela navegación)  |



## 🧠 Lógica del sistema

* STOP → solo reduce velocidad (no cancela navegación)
* TURN → cancela navegación → ejecuta giro → reanuda misión
* AMCL mantiene localización continua
* Waypoints gestionados dinámicamente


## ⚠️ Notas importantes

* No usar STOP + cancelación → rompe AMCL
* Evitar avanzar waypoint manualmente en abort
* Mantener coherencia entre `resume_index` y navegación real

## 📍 Estado actual

✔ Navegación estable
✔ Integración UART funcional
✔ Control dinámico de velocidad
✔ Reanudación tras giros


## 🚀 Próximos pasos

* Detección de colisiones
* Replanificación inteligente
* Integración sensor háptico (TFG)



ros2 topic echo /emergency_stop

DAta true en 
