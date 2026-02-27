mundo :
rm -rf build install log
cd ~/tfg/aidguide_04-main/aidguide_04_ws

colcon build --packages-select aidguide_sim
source install/setup.bash
ros2 launch aidguide_sim sim.launch.py

si sale mundo defoult : en la carpeta 
mimi@RoboticaUbunto24:~/tfg/aidguide_04-main/aidguide_04_ws$ 

export GZ_SIM_RESOURCE_PATH=/home/mimi/tfg/aidguide_04-main/aidguide_04_ws/src/aidguide_sim/models:/home/mimi/tfg/aidguide_04-main/aidguide_04_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models:$GZ_SIM_RESOURCE_PATH



  SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=models
        ),

<!-- SEMÁFOROS 
    <include>
      <pose>1.26 -2.60 0.4 0 0 0</pose>
      <uri>model://semaforo_rojo</uri>
    </include>
-->
<!--
    <include>
      <pose>-2 4.0 0.4 0 0 0</pose>
      <uri>model://semaforo_verde</uri>
    </include> -->