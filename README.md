# Delta Robot Simulation on Gazebo using MARA-Env

Delta robot pick and place simulation in Gazebo 9.0.0 using MARA environment

![](demo/DeltaRobotPickandplaceapplication.gif)


# Tested on ROS2 Crystal and Gazebo 9.0.0
Download ROS2 MARA workspace (zip) file: https://github.com/AcutronicRobotics/MARA/releases/tag/crystal-v1.0.0

# Install Prerequisites
Follow this well documented steps from MARA repo: https://github.com/AcutronicRobotics/MARA/tree/crystal

# ROS2Learn for environment and RL algorithm
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
(Note: make sure you have build ros2_mara_ws before this command)
   - `cp delta_kinematic_lib/libdeltaKinematics.so ~/ros2_mara_ws/install/lib/.`
 
## Compile and Launch ROS2 Delta Robot Simulation
```
   source ~/ros2learn/environments/gym-gazebo2/provision/mara_setup.sh
   cd ros2_mara_ws
   colcon build && cd
   python3 ros2learn/environments/gym-gazebo2/examples/MARA/delta_arm.py -g
```
# References
1. [Rotary Delta Robot Forward/Inverse Kinematics Calculations](https://www.marginallyclever.com/other/samples/fk-ik-test.html)

2. [MARA Robot RL algo implementation](https://github.com/AcutronicRobotics/gym-gazebo2)

3. [Train Mask-RCNN model on Custom Dataset for Multiple Objects](https://github.com/AutoRoboCulture/mask-rcnn-for-multiple-objects.git)

4. [Weed removal robot using Delta Arm](https://github.com/AutoRoboCulture/nindamani-the-weed-removal-robot.git)
