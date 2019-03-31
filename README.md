# RoboND-Term1-P4-Map-My-World
Project 4 of Udacity Robotics Software Engineer Nanodegree Program
![Overview](/videos/Term1-Project3-Where-Am-I-Demo_2.gif)  
[TODO]  
## Overview  
In this project you'll utilize ROS AMCL package to accurately localize a mobile robot inside a map in the Gazebo simulation environments. Here are the steps to learn several aspects of robotic software engineering with a focus on ROS:  
* Create a ROS package that launches a custom robot model in a custom Gazebo world  
* Utilize the ROS AMCL package and the Tele-Operation / Navigation Stack to localize the robot  
* Explore, add, and tune specific parameters corresponding to each package to achieve the best possible localization results  

## Prerequisites/Dependencies  
* Gazebo >= 7.0  
* ROS Kinetic  
* ROS navigation package  
```
sudo apt-get install ros-kinetic-navigation
```
* ROS map_server package  
```
sudo apt-get install ros-kinetic-map-server
```
* ROS move_base package  
```
sudo apt-get install ros-kinetic-move-base
```
* ROS amcl package  
```
sudo apt-get install ros-kinetic-amcl
```
* ROS rtabmap-ros package
```
sudo apt-get install ros-kinetic-rtabmap-ros
```
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Setup Instructions (abbreviated)  
1. Meet the `Prerequisites/Dependencies`  
2. Open Ubuntu Bash and clone the project repository  
3. On the command line execute  
```bash
sudo apt-get update && sudo apt-get upgrade -y
```
4. Build and run your code.  
[TODO]  
## Project Description  
Directory Structure  
```
.Where-Am-I                                    # Where Am I Project
├── catkin_ws                                  # Catkin workspace
│   ├── src
│   │   ├── ball_chaser                        # ball_chaser package        
│   │   │   ├── launch                         # launch folder for launch files
│   │   │   │   ├── ball_chaser.launch
│   │   │   ├── src                            # source folder for C++ scripts
│   │   │   │   ├── drive_bot.cpp
│   │   │   │   ├── process_images.cpp
│   │   │   ├── srv                            # service folder for ROS services
│   │   │   │   ├── DriveToTarget.srv
│   │   │   ├── CMakeLists.txt                 # compiler instructions
│   │   │   ├── package.xml                    # package info
│   │   ├── my_gokart                          # my_gokart package        
│   │   │   ├── launch                         # launch folder for launch files   
│   │   │   │   ├── gokart_description.launch
│   │   │   │   ├── world.launch
│   │   │   ├── urdf                           # urdf folder for xarco files
│   │   │   │   ├── my_gokart.gazebo
│   │   │   │   ├── my_gokart.xacro
│   │   │   ├── worlds                         # world folder for world files
│   │   │   │   ├── empty.world
│   │   │   │   ├── myoffice.world
│   │   │   ├── CMakeLists.txt                 # compiler instructions
│   │   │   ├── package.xml                    # package info
│   │   ├── my_robot                           # my_robot package        
│   │   │   ├── config                         # config folder for configuration files   
│   │   │   │   ├── base_local_planner_params.yaml
│   │   │   │   ├── costmap_common_params.yaml
│   │   │   │   ├── global_costmap_params.yaml
│   │   │   │   ├── local_costmap_params.yaml
│   │   │   ├── launch                         # launch folder for launch files   
│   │   │   │   ├── amcl.launch
│   │   │   │   ├── robot_description.launch
│   │   │   │   ├── world.launch
│   │   │   ├── maps                           # maps folder for maps
│   │   │   │   ├── myoffice.pgm
│   │   │   │   ├── myoffice.yaml
│   │   │   ├── meshes                         # meshes folder for sensors
│   │   │   │   ├── hokuyo.dae
│   │   │   ├── rviz                           # rviz folder for rviz configuration files
│   │   │   │   ├── default.rviz
│   │   │   ├── urdf                           # urdf folder for xarco files
│   │   │   │   ├── my_robot.gazebo
│   │   │   │   ├── my_robot.xacro
│   │   │   ├── worlds                         # world folder for world files
│   │   │   │   ├── empty.world
│   │   │   │   ├── myoffice.world
│   │   │   ├── CMakeLists.txt                 # compiler instructions
│   │   │   ├── package.xml                    # package info
│   │   ├── pgm_map_creator                    # pgm_map_creator        
│   │   │   ├── launch                         # launch folder for launch files   
│   │   │   │   ├── request_publisher.launch
│   │   │   ├── maps                           # maps folder for generated maps
│   │   │   │   ├── Backup_map.pgm
│   │   │   │   ├── map.pgm
│   │   │   ├── msgs                           # msgs folder for communication files
│   │   │   │   ├── CMakeLists.txt
│   │   │   │   ├── collision_map_request.proto
│   │   │   ├── src                            # src folder for main function
│   │   │   │   ├── collision_map_creator.cc
│   │   │   │   ├── request_publisher.cc
│   │   │   ├── world                          # world folder for world files
│   │   │   │   ├── myoffice.world
│   │   │   │   ├── udacity_mtv
│   │   │   ├── CMakeLists.txt                 # compiler instructions
│   │   │   ├── LICENSE                        # License for repository
│   │   │   ├── README.md                      # README for documentation
│   │   │   ├── package.xml                    # package info
│   │   ├── teleop_twist_keyboard              # teleop_twist_keyboard
│   │   │   ├── CHANGELOG.rst                  # change log
│   │   │   ├── CMakeLists.txt                 # compiler instructions
│   │   │   ├── README.md                      # README for documentation
│   │   │   ├── package.xml                    # package info
│   │   │   ├── teleop_twist_keyboard.py       # keyboard controller
├── my_ball                                    # Model files 
│   ├── model.config
│   ├── model.sdf
├── videos                                     # Video files
│   ├── Term1-Project3-Where-Am-I-Demo_1.gif   # Demo video
│   ├── Term1-Project3-Where-Am-I-Demo_2.gif   # Demo video
│   ├── Term1-Project3-Where-Am-I-Demo_3.gif   # Demo video
│   ├── Term1-Project3-Where-Am-I-Demo_4.gif   # Demo video
```
- [Term1-Project3-Where-Am-I-Demo_1.gif](/videos/Term1-Project3-Where-Am-I-Demo_1.gif): A demo video for failure run.  
- [Term1-Project3-Where-Am-I-Demo_2.gif](/videos/Term1-Project3-Where-Am-I-Demo_2.gif): A demo video for successful run.  
- [Term1-Project3-Where-Am-I-Demo_3.gif](/videos/Term1-Project3-Where-Am-I-Demo_3.gif): A demo video for successful run.  
- [Term1-Project3-Where-Am-I-Demo_4.gif](/videos/Term1-Project3-Where-Am-I-Demo_4.gif): A demo video for successful run.  
- [drive_bot.cpp](/catkin_ws/src/ball_chaser/src/drive_bot.cpp): ROS service C++ script, command the robot with specify speeds.  
- [process_images.cpp](/catkin_ws/src/ball_chaser/src/process_images.cpp): ROS service C++ script, process the camera image and return requested speeds.  
- [gokart_description.launch](/catkin_ws/src/my_gokart/launch/gokart_description.launch): Create gokart model in Gazebo world.  
- [world.launch](/catkin_ws/src/my_gokart/launch/world.launch): Launch my_gokart mode in Gazebo world with building and plugins.  
- [my_gokart.gazebo](/catkin_ws/src/my_gokart/urdf/my_gokart.gazebo): Define my_gokart URDF model plugins.  
- [my_gokart.xacro](/catkin_ws/src/my_gokart/urdf/my_gokart.xacro): Define my_gokart URDF model.  
- [empty.world](/catkin_ws/src/my_gokart/worlds/empty.world): Gazebo world file that includes nothing.  
- [myoffice.world](/catkin_ws/src/my_gokart/worlds/myoffice.world): Gazebo world file that includes the models.  
- [CMakeLists.txt](/catkin_ws/src/my_gokart/CMakeLists.txt): File to link the C++ code to libraries.  
- [robot_description.launch](/catkin_ws/src/my_robot/launch/robot_description.launch): Create robot model in Gazebo world.  
- [hokuyo.dae](/catkin_ws/src/my_robot/meshes/hokuyo.dae): Hokuyo LiDAR sensor mesh model.  
- [my_robot.gazebo](/catkin_ws/src/my_robot/urdf/my_robot.gazebo): Define my_robot URDF model plugins.  
- [my_robot.xacro](/catkin_ws/src/my_robot/urdf/my_robot.xacro): Define my_robot URDF model.  
- [amcl.launch](/catkin_ws/src/my_robot/launch/amcl.launch): Launch AMCL node
- [myoffice.pgm](/catkin_ws/src/my_robot/maps/myoffice.pgm): Generated myoffice map
- [myoffice.yaml](/catkin_ws/src/my_robot/maps/myoffice.yaml): Info for myoffice map
- [default.rviz](/catkin_ws/src/my_robot/rviz/default.rviz): Default rviz
- [map.pgm](/catkin_ws/src/pgm_map_creator/maps/map.pgm): Generated myoffice map
[TODO]  
## Run the project  
* Clone this repository
```
https://github.com/jinchaolu/RoboND-Term1-P4-Map-My-World.git
```
* Open the repository and make  
```
cd /home/workspace/RoboND-Term1-P4-Map-My-World/catkin_ws/
catkin_make
```
* Launch my_robot in Gazebo to load both the world and plugins  
```
roslaunch my_robot world.launch
```  
* Launch teleop_twist_keyboard node, open a new terminal, enter  
```
cd /home/workspace/RoboND-Term1-P4-Map-My-World/catkin_ws/
source devel/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```  
* Launch teleop_twist_keyboard node, open a new terminal, enter  
```
cd /home/workspace/RoboND-Term1-P4-Map-My-World/catkin_ws/
source devel/setup.bash
roslaunch my_robot mapping.launch
```  
* Testing  
Send move command via teleop package to control your robot and observe real-time visualization in the environment `rtabmapviz`.  
rtabmap-databaseViewer ~/.ros/rtabmap.db

* View database
Once you statisfied with your move, press `Ctrl + c` to exit then view your database with
```
rtabmap-databaseViewer ~/.ros/rtabmap.db
```
Remember to rename your `~/.ros/rtabmap.db` before your next attempt since it will be deleted due to the launch file setting in `mapping.launch`

## Tips  
1. It's recommended to update and upgrade your environment before running the code.  
```bash
sudo apt-get update && sudo apt-get upgrade -y
```
2. Remember to rename your `~/.ros/rtabmap.db` before your next attempt since it will be deleted due to the launch file setting in `mapping.launch`

## Code Style  
Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Rubric  
### 1. Basic Requirements  
#### 1.1 Did the student submit all required files?  
Yes, he did.   
### 2. Simulation Setup
#### 2.1 Did the student set up the simulation environment properly?  
Yes, he did.  
#### 2.2 Is the student's simulation suitable for mapping task?  
Yes, it is.  
### 3. Mapping Package  
#### 3.1 Does the student correctly build all required launch files for RTAB-Mapping?  
Yes, he does.  
### 4. Mapping Accuracy  
#### 4.1 Was the student able to generate a 3D map using RTAB-Map?  
Yes, he was.  
#### 4.2 Does the student's 3D map portray environment characteristics?  
Yes, he does.  
[TODO]  
## Videos  
Too few particles will result into localization failure.  
![Demo_1](/videos/Term1-Project3-Where-Am-I-Demo_1.gif)  
Good run demo 1.  
![Demo_2](/videos/Term1-Project3-Where-Am-I-Demo_2.gif)  
Good run demo 2.  
![Demo_3](/videos/Term1-Project3-Where-Am-I-Demo_3.gif)  
Good run demo 3.  
![Demo_4](/videos/Term1-Project3-Where-Am-I-Demo_4.gif)  
Good run demo 4.  
![Demo_4](/videos/Term1-Project3-Where-Am-I-Demo_5.gif)  