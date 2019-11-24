Z
Follow this step for installation: https://github.com/AcutronicRobotics/ros2learn/tree/crystal

# Steps Before Run
1. Copy **mara.py** from MARA_env folder and move it to **~/ros2learn/environments/gym-gazebo2/gym_gazebo2/envs/MARA**
   - `cp MARA_env/mara.py ~/ros2learn/environments/gym-gazebo2/gym_gazebo2/envs/MARA/.`
2. Copy **delta_arm.py** from example folder and move it to **~/ros2learn/environments/gym-gazebo2/examples/MARA**
   - `cp example/delta_arm.py ~/ros2learn/environments/gym-gazebo2/examples/MARA/.`
3. Copy all **delta_meshes** files and move it to **~/ros2_mara_ws/src/mara/mara_description/meshes**
   - `cp delta_meshes/* ~/ros2_mara_ws/src/mara/mara_description/meshes/.`
4. Copy **mara.launch.py** from launch folder and move it to **~/ros2_mara_ws/src/mara/mara_bringup/launch**
   - `cp launch/mara.launch.py ~/ros2_mara_ws/src/mara/mara_bringup/launch/.`
5. Copy whole **mara_gazebo_plugins** folder and replace with **~/ros2_mara_ws/src/mara/mara_gazebo_plugins**
   - `cp -rf mara_gazebo_plugins/* ~/ros2_mara_ws/src/mara/mara_gazebo_plugins/.`
6. Copy **arc_delta_onTable.urdf** from urdf folder and move it to **~/ros2_mara_ws/src/mara/mara_description/urdf/reinforcement_learning**
   - `cp urdf/arc_delta_onTable.urdf ~/ros2_mara_ws/src/mara/mara_description/urdf/reinforcement_learning/.`
7. Copy **libdeltaKinematics.so** from delta_kinematic_lib folder and move to **~/ros2_mara_ws/install/lib**
   - `cp delta_kinematic_lib/libdeltaKinematics.so ~/ros2_mara_ws/install/lib/.`
