# multiple-grasping-pose-learning
ros package for robot auto data collection and learning different grasping poses for different objects
---
Please see doc/manual.docx

## Workspace build(melodic)
Build ROS with Python3 environment (for tpu)  
Environment:  Ubuntu18.04 and ROS Melodic
```
sudo apt-get install python3-catkin-pkg-modules python3-rospkg-modules python3-venv python3-empy
sudo apt-get install python3-opencv
sudo apt-get install ros-melodic-catkin
source /opt/ros/melodic/setup.bash
mkdir -p ~/aerov_grasp_ws/src
cd ~/aerov_grasp_ws/src
git clone https://github.com/himlen1990/multiple-grasping-pose-learning.git
wstool init
wstool merge multiple-grasping-pose-learning/fc.rosinstall
wstool update
rosdep install --from-paths . --ignore-src -y -r
cd ~/aerov_grasp_ws
catkin init
catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
catkin build
```
(ref https://github.com/knorth55/coral_usb_ros for environment building)  
If nothing goes wrong, we can start collect data.  
May be you should [build aero-ros-pkg manually](https://github.com/seed-solutions/aero-ros-pkg#build-packge ), and then [create eusmodel of aero](https://github.com/jsk-ros-pkg/jsk_robot/tree/master/jsk_aero_robot/aeroeus#create-eusmodel ).

## Data collection
launch file for data collection.
```
roslaunch multiple_grasping_pose_learning data_collection_turntable.launch
```
Run a robot program that collects data.
```
roscd multiple_grasping_pose_learning/euslisp/
roseus collect_data_turntable.l
```