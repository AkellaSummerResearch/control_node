# control_node

This package uses mavros to conduct a visual inspection of the f-35 using predetermained waypoints. The inspection utalizes r-tab to conduct a localiztion and formulate correction between the local frame of reference and the map of the plane's reference frame


## Dependecies

This package assumes you have already configured *Simulations ROS* Please complete instalation instructions for Simulations ROS first. 

https://github.com/AkellaSummerResearch/simulations_ros

please install 

apt packages
```
sudo apt-get install libeigen3-dev
sudo apt-get install ros-kinetic-octomap
```

The minimum snap package generates smooth trajectories through waypoints. To install it, open the ```scripts``` folder within the ```simulation_ros``` package:

```
cd ~/catkin_ws/src/simulation_ros/scripts
subl install_minsnap_pkg.sh
```

In the very first line, type the path of your catkin workspace. It most likely is ```~/catkin_ws```, if you've been following the instructions.

Execute the file:

```
./install_minsnap_pkg.sh
```

ros packages
```
cd ~/catkin_ws/src
git clone https://github.com/radionavlab/px4_control.git
git clone https://github.com/radionavlab/joystick_drivers
git clone https://github.com/radionavlab/mg_msgs.git
https://github.com/marcelinomalmeidan/mapper.git
cd ~/catkin_ws/src/px4_control
git checkout lockheed_quads
```


## IMPORTANT!! Compatibility: The quads run an older version of PX4, different than in the simulation!

The quadcopters run an older version of PX4, while the simulation runs a recent version of PX4. I could not find out how to run Gazebo's simulation with the same version of PX4 as what runs in the quadcopter. Because of that, the packages ```px4_control``` and ```mavros``` have to be configured before compiling.

If you want to use the simulation, you need Kinetic version of software. To this end, switch mavros to current version:

```
cd ~/catkin_ws/src/mavros
git checkout master
```

If you want to command the quadcopter, you will need to use the Indigo version of mavros:

```
cd ~/catkin_ws/src/mavros
git checkout indigo-devel
```

Depending on either version, the package px4_control has to be compiled accordingly. This can be changed as a flag in the fourth line of ```~/catkin_ws/src/px4_control/CMakeLists.txt```.

For Kinetic (simulation) version:

```
option(USE_KINETIC "Use kinetic version instead of indigo version" TRUE)
```

For Indigo (quadcopter) version:

```
option(USE_KINETIC "Use kinetic version instead of indigo version" FALSE)
```

## installing r-tab

Required dependencies

The easiest way to get all them (Qt, PCL, VTK, OpenCV, ...) is to install/uninstall rtabmap binaries:

```
sudo apt-get install ros-kinetic-rtabmap ros-kinetic-rtabmap-ros
sudo apt-get remove ros-kinetic-rtabmap ros-kinetic-rtabmap-ros
```


innstall RTAB-Map standalone libraries. Add `-DCMAKE_INSTALL_PREFIX=~/catkin_ws/devel` to cmake command below if you want to install in your Catkin's devel folder without sudo. Do not clone in your Catkin workspace.

```
cd ~
git clone https://github.com/AkellaSummerResearch/rtabmap.git
cd rtabmap/build
cmake ..  [<---double dots included]
make
sudo make install
```

Install RTAB-Map ros-pkg in your src folder of your Catkin workspace.

```
cd ~/catkin_ws
git clone https://github.com/AkellaSummerResearch/rtabmap_ros src/rtabmap_ros
catkin_make
```

Create folder for storing map databases:

```
mkdir ~/mapdb
```


## Build control_node

```
cd ~/catkin_ws/src
git clone https://github.com/AkellaSummerResearch/control_node.git
cd ~/catkin_ws
catkin_make
```

## Mapping the plane

The localization algorithm works by performing loop closers. Because of this it is important that you map places in position and orientation that your drone is likley to fly. 

### Generate Mapping Waypoints

Use my python script to generate circular waypoits around the plane

Modify the script to generate yout prefered circles and name the file in which you would like the waypoints are saved

```
cd ~/catkin_ws/src/control_node/scripts/
python waypointGen.py
```

note** waypoint are in the format of `x y z yaw` with respect to the map reference frame

### Set Up the Mapping Yaml

```
subl ~/catkin_ws/src/control_node/cfg/training.yaml
```
set the path and file name of your mapping waypoints ex.

```
path: "/home/lockheed/catkin_ws/src/control_node/waypoints/"
filename: "training.wp"
```

### Running mapping algorithm


```
// first terminal 

./startsim.sh

//next terminal 

roslaunch simulations_ros px4.launch

//next terminal 

roslaunch simulations_ros kinectMapping.launch

//next terminal 

roslaunch control_node training.launch

``` 

The first two commands start the simulation and other packages. If you desire to have a separate window visualizing the drone's camera, then run in a separate terminal:

```
rosrun image_view image_view image:=/webcam/image_raw
```

## Generating Waypoints For Inspection Process

Be sure to start the drone where you generated the origin of your map. by default iris pose should be `<pose>0 0 0 0 0 1.57</pose>`

### Set Up Manual Waypoint Yaml 

```
subl ~/catkin_ws/src/control_node/cfg/manualWaypoints.yaml
```
set the path and file name of your mapping waypoints ex.

```
path: "/home/lockheed/catkin_ws/src/control_node/waypoints/"
filename: "inspection.wp"
```

### Run Manual Waypoint Generation
```
// first terminal 

./startsim.sh

//next terminal 

roslaunch simulations_ros px4.launch

//next terminal 

roslaunch px4_control gazebo.launch

//next terminal

roslaunch control_node manualWaypoints.launch

``` 

note** make sure to run `roslaunch control_node manualWaypoints.launch` before the drone takesoff, as the program will generate a local reference frame based off of the initial yaw angle

Press up on the D pad to generate waypoints. For more information on flying the drone with xbox controller please refer to the github [page](https://github.com/radionavlab/px4_control) 

## Inspection

### Set Up Inspection Yaml


```
subl ~/catkin_ws/src/control_node/cfg/inspection.yaml
```
set the path and file name of your mapping waypoints ex.

```
path: "/home/lockheed/catkin_ws/src/control_node/waypoints/"
filename: "start.wp"
inspection: "train.wp"
```

start.wp should be a circular pattern of waypoints for the purpose of allowing the drone to rectify its local frame with its map frame
train.wp should be the waypoints for the inspection

### Running Inspection

```
// first terminal 

./startsim.sh

//next terminal 

roslaunch simulations_ros px4.launch

//next terminal 

roslaunch simulations_ros localization.launch

//next terminal 

roslaunch control_node control.launch

//sit back relax and enjoy the ride
```

---

## Updating Waypoint Speed and Acceleration

```
subl ~/Firmware/posix-configs/SITL/init/ekf2/iris
```
Full list of [Parameters](https://dev.px4.io/en/advanced/parameter_reference.html)

Usefull parameters under Multicopter Position Control

References:

- https://github.com/radionavlab/px4_control
- https://github.com/introlab/rtabmap_ros
- https://github.com/introlab/rtabmap
- http://wiki.ros.org/rtabmap_ros



