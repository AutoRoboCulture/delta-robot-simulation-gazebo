# Hans modular servomotor

<a href="http://www.acutronicrobotics.com"><img src="https://acutronicrobotics.com/products/modular-joints/images/ModularJoints_2-04.jpg" align="right" hspace="8" vspace="2" width="200"></a>

The Han's Robot Modular Joints are industrial 2 DoF electrical motors that include precised encoders and an electro-mechanical breaking system for safety purposes. The Modular Joints natively speak ROS 2.0 and interoperate seamlessly with other ROS 2.0 based robot parts, regardless of their original manufacturer. Moreover, they provide time synchronization capabilities and offer deterministic communications enabled by TSN standards.

The Han's Robot Modular Joints are available in a variety of different torque and size combinations, going from 2.8 kg to 17 kg weight and from 9.4 Nm to 156 Nm rated torque.

The H-ROS System on Module (SoM) is a tiny device for building industrial grade plug & play robot modules. All the Hans Robot Modular Joints have an H-ROS SoM inside, running ROS 2.0 natively, and providing security, interoperability, real-time and extensibility capabilities, among others.

<table id="motors-table" style="width:100%">
  <tbody><tr id="table-header">
      <th style="width: 16.66%"></th>
      <th style="width: 16.66%"><img style="width: 75%; padding-bottom: 1rem;" src="https://acutronicrobotics.com/products/modular-joints/images/modules/module-1.png" data-pagespeed-url-hash="2276967716" onload="pagespeed.CriticalImages.checkImageForCriticality(this);"><br>T9.4</th>
      <th style="width: 16.66%"><img style="width: 75%; padding-bottom: 1rem;" src="https://acutronicrobotics.com/products/modular-joints/images/modules/module-2.png" data-pagespeed-url-hash="2571467637" onload="pagespeed.CriticalImages.checkImageForCriticality(this);"><br>T30</th>
      <th style="width: 16.66%"><img style="width: 75%; padding-bottom: 1rem;" src="https://acutronicrobotics.com/products/modular-joints/images/modules/module-3.png" data-pagespeed-url-hash="2865967558" onload="pagespeed.CriticalImages.checkImageForCriticality(this);"><br>T49</th>
      <th class="disabled-text" style="width: 16.66%"><img style="width: 75%; padding-bottom: 1rem;" src="https://acutronicrobotics.com/products/modular-joints/images/modules/module-4.png" data-pagespeed-url-hash="3160467479" onload="pagespeed.CriticalImages.checkImageForCriticality(this);"><br>T85</th>
      <th class="disabled-text" style="width: 16.66%"><img style="width: 75%; padding-bottom: 1rem;" src="https://acutronicrobotics.com/products/modular-joints/images/modules/module-5.png" data-pagespeed-url-hash="3454967400" onload="pagespeed.CriticalImages.checkImageForCriticality(this);"><br>T156</th>
  </tr>
  <tr>
      <td class="row-title">rated torque</td>
      <td>9.4Nm</td>
      <td>30Nm</td>
      <td>49Nm</td>
      <td class="disabled-text">85Nm</td>
      <td class="disabled-text">156Nm</td>
  </tr>
  <tr>
      <td class="row-title">peak torque</td>
      <td>34Nm</td>
      <td>69Nm</td>
      <td>104Nm</td>
      <td class="disabled-text">200Nm</td>
      <td class="disabled-text">420Nm</td>
  </tr>
  <tr>
      <td class="row-title">max rotation speed</td>
      <td>90º/s</td>
      <td>90º/s</td>
      <td>90º/s</td>
      <td class="disabled-text">90º/s</td>
      <td class="disabled-text">90º/s</td>
  </tr>
  <tr>
      <td class="row-title">weight</td>
      <td>2.8kg</td>
      <td>3.8kg</td>
      <td>5.8kg</td>
      <td class="disabled-text">9.6kg</td>
      <td class="disabled-text">17kg</td>
  </tr>
  <tr>
      <td class="row-title">rotation angle</td>
      <td>±360º</td>
      <td>±360º</td>
      <td>±360º</td>
      <td class="disabled-text">±360º</td>
      <td class="disabled-text">±360º</td>
  </tr>
  <tr>
      <td class="row-title">repeatability</td>
      <td>20arcsec</td>
      <td>20arcsec</td>
      <td>20arcsec</td>
      <td class="disabled-text">20arcsec</td>
      <td class="disabled-text">20arcsec</td>
  </tr>
  <tr>
      <td class="row-title">communication</td>
      <td>Gigabit Ethernet</td>
      <td>Gigabit Ethernet</td>
      <td>Gigabit Ethernet</td>
      <td class="disabled-text">Gigabit Ethernet</td>
      <td class="disabled-text">Gigabit Ethernet</td>
  </tr>
  <tr>
      <td class="row-title">supply voltage</td>
      <td>DC 48V</td>
      <td>DC 48V</td>
      <td>DC 48V</td>
      <td class="disabled-text">DC 48V</td>
      <td class="disabled-text">DC 48V</td>
  </tr>
  <tr>
      <td class="row-title">main material</td>
      <td>aluminum allow</td>
      <td>aluminum allow</td>
      <td>aluminum allow</td>
      <td class="disabled-text">aluminum allow</td>
      <td class="disabled-text">aluminum allow</td>
  </tr>
  <tr>
      <td class="row-title">working temperature</td>
      <td>0-50ºC</td>
      <td>0-50ºC</td>
      <td>0-50ºC</td>
      <td class="disabled-text">0-50ºC</td>
      <td class="disabled-text">0-50ºC</td>
  </tr>
</tbody></table>


# Install

## Install ROS 2.0

Install ROS 2.0 following the official instructions: [source](https://index.ros.org/doc/ros2/Linux-Development-Setup/) [debian packages](https://index.ros.org/doc/ros2/Linux-Install-Debians/).

## Create mara ROS 2.0 workspace
Create a ROS workspace, for example:

```bash
mkdir -p ~/ros2_mara_ws/src
cd ~/ros2_mara_ws
sudo apt install -y python3-vcstool python3-numpy
wget https://raw.githubusercontent.com/acutronicrobotics/MARA/master/mara-ros2.repos
vcs import src < mara-ros2.repos
```

Generate HRIM dependencies:

```bash
pip3 install lxml
cd ~/ros2_mara_ws/src/HRIM
python3 hrim.py generate models/actuator/servo/servo.xml
python3 hrim.py generate models/actuator/gripper/gripper.xml
```

## Compile

**Optional note**: If you want to use MoveIT! you need to source ROS 1.0 environment variables. Typically, if you have installed ROS `Kinetic`, you need to source the following file:

```bash
source /opt/ros/kinetic/setup.bash
```

Right now you can compile the code:

```bash
source /opt/ros/crystal/setup.bash
cd ~/ros2_mara_ws && colcon build --merge-install
```

# Launch

### T9.4 ( HANS series 14 )

```
ros2 launch hans_modular_gazebo hans_t9_4.launch.py
```

### T30 ( HANS series 17 )

```
ros2 launch hans_modular_gazebo hans_t30.launch.py
```

### T49 ( HANS series 20 )

```
ros2 launch hans_modular_gazebo hans_t49.launch.py
```
