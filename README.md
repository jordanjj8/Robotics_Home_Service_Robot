# Robotics_Home_Service_Robot
Final Project Home Service Robot for [Robotics Software Engineer Nanodegree Program](https://www.udacity.com/course/robotics-software-engineer--nd209)

## Overview 
  This project involves programming a robot to map out an environment and autonomously navigate itself to pick and deliver objects. Multiple packages, ROS offical packages, along with packages I created myself, were used to make this project happen.
  
  First off, SLAM was used to create a functional map of the environment. The map of the environment would then be used for localization and navigating. A ROS navigation stack was then used to plan out the robot trajectory with a specified start and goal position. Finally, markers were created at the pickup and dropoff locations (start and end positions) to simulate a home service! The robot is responsible of picking up an object (cube in this case) and deliver the cube to the dropoff zone. Two packages were created to make this happen simultaneuously. Look at the links at the end of the document for further documentation. 
  
## Requirements 
* [Gazebo](http://gazebosim.org/) any version later than v7.0.0 
* ROS Kinetic -follow instructions here: [ROS Installation Instructions](http://wiki.ros.org/ROS/Installation)
* GNU make 
  - Installation for Windows 
  - Installation for Mac
  - make is installed by default for most Linux distros 
* C++11 Complier (gcc/g++)

Recommended Virtual Machine Alternative Specs:
* Ubuntu 64-bit Disk Image 
* 2 GB of RAM 
* 20 GB Disk Space
* 4 Logical Cores 

## Setup
1. After installing and meeting the requirements listed above, open up a terminal.
2. Create and initialize a catkin workspace:
``` 
    $ mkdir -p /home/workspace/catkin_ws/src
    $ cd /home/workspace/catkin_ws/src
    $ catkin_init_workspace
```
3. Clone the project repository & move the `my_robot` and `ball_chaser` directories into the `src` directory:
```
    $ cd /home/workspace/catkin_ws/src
    $ git clone https://github.com/jordanjj8/Robotics_Home_Service_Robot.git
    $ mv Robotics_Home_Service_Robot/add_markers /home/workspace/catkin_ws/src/add_markers 
    $ mv Robotics_Home_Service_Robot/my_robot /home/workspace/catkin_ws/src/my_robot 
    $ mv Robotics_Home_Service_Robot/pick_objects /home/workspace/catkin_ws/src/pick_objects
    $ mv Robotics_Home_Service_Robot/rvizConfig /home/workspace/catkin_ws/src/rvizConfig
    $ mv Robotics_Home_Service_Robot/scripts /home/workspace/catkin_ws/src/scripts
    $ mv Robotics_Home_Service_Robot/slam_gmapping /home/workspace/catkin_ws/src/slam_gmapping
    $ mv Robotics_Home_Service_Robot/turtlebot /home/workspace/catkin_ws/src/turtlebot
    $ mv Robotics_Home_Service_Robot/turtlebot_interactions /home/workspace/catkin_ws/src/turtlebot_interactions
    $ mv Robotics_Home_Service_Robot/turtlebot_simulator /home/workspace/catkin_ws/src/turtlebot_simulator 
    $ rm -rf Robotics_Go_Chase_It
```
4. Navigate back to the catkin workspace and build the executables:
```
    $ cd /home/workspace/catkin_ws/
    $ catkin_make
```
5. Navigate to the scripts folder and run the home_service.sh the shell script.
``` 
    $ cd /home/workspace/catkin_ws/src/scripts/
    $ chmod +x home_service.sh
    $ ./home_service.sh
```
## Related Links
http://wiki.ros.org/gmapping
http://wiki.ros.org/turtlebot_teleop
http://wiki.ros.org/turtlebot_rviz_launchers
http://wiki.ros.org/turtlebot_gazebo
http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals
http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes
http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29

