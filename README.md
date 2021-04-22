# multiple-grasping-pose-learning
ros package for robot learning different grasping pose for different objects
---
## Environment
---
- Ubuntu18.04 and ROS Melodic

## Installation

'''bash
git clone https://github.com/himlen1990/multiple-grasping-pose-learning.git
'''

## Run

'''bash
rosrun multiple_grasping_pose_learning demo
roscd multiple_grasping_pose_learning/euslisp/
roseus collect_data.l
'''

## After collected data

'''bash
python labelme2voc /01 /voc --labels /01/labels.txt
'''
