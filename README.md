# multiple-grasping-pose-learning
ros package for robot auto data collection and learning different grasping poses for different objects
---
## Environment
---
- Ubuntu18.04 and ROS Melodic

## Setup ROS Environment and Check Coral EdgeTPU

follow the instruction in https://github.com/himlen1990/semantic_segmentation_tpu

## build aero package

```bash
cd ~/aerov_grasp_ws/src
git clone https://github.com/himlen1990/multiple-grasping-pose-learning.git
catkin build
```

## Real world environment setup

Print two marker2.png in /utils and put them on the shelf

## Run

connect to the aerov robot

```bash
source ~/aerov_grasp_ws/devel/setup.bash
rosrun multiple_grasping_pose_learning data_collection
~~~ open another terminal
roscd multiple_grasping_pose_learning/euslisp/
roseus collect_data.l
~~~ whitin the roseus command line
(collect_data)
```

## Adding new objects
```bash
~~~ open another terminal and send for creating new folder
rostopic pub -1 /aero_data_collection std_msgs/String "add new"
~~~ whitin the roseus command line
(collect_data)
```

## collect the current object data again
```bash
rostopic pub -1 /aero_data_collection std_msgs/String "do again"
~~~ whitin the roseus command line
(collect_data)
```

## turn table version (similar to the grasping version)
rosrun multiple_grasping_pose_learning data_collection_turntable
roseus collect_data_turntable.l
~~~ before collect new object data, adjust the turntable's position to keep it inside the arcode range, do not put any objects on the table!
~~~ whitin the roseus command line, run
(get_table)
~~~ now you can put the object on the table
~~~ adding new objects
rostopic pub -1 /aero_data_collection std_msgs/String "add new"
~~~ whitin the roseus command line, run
(collect_data_once)


## After collected data

```bash
cd utils
(pip install json)
(pip install collections)
(pip install dict2xml)
~~~ create augmented data
python data_augmentation_bbox_turntable.py

python label_generation_labelme_ver.py
~~~ create a labels.txt file and add class names
python labelme2voc ../dataset/rgb/01 ../dataset/rgb/voc --labels ../dataset/rgb/01/labels.txt
```

