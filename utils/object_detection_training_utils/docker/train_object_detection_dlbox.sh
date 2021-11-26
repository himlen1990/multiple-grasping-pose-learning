#!/bin/bash
function message() {
  local color=$1; shift;
  local message=$@

  # Output command being executed
  echo -e "\e[${color}m${message}\e[0m"
}

if [ -z "$1" ]; then
    message 31 "Set path to dataset is required"
    exit 1
fi

DATASET_DIR=$(realpath $1); shift 1;
DATASET_NAME=$(basename $DATASET_DIR)
echo $DATASET_DIR

ITERATION_TIMES=2000

DATE=$(date +"%Y%m%d-%H%M%S")
DATASET_NAME_NEW=$DATE-$ITERATION_TIMES-$DATASET_NAME

## DLBOX use personal user id currently
# DLBOX_IP=shifan@dlbox14.jsk.imi.i.u-tokyo.ac.jp
DLBOX_IP=shifan@dlbox6.jsk.imi.i.u-tokyo.ac.jp

set -x
scp -q -r $DATASET_DIR $DLBOX_IP:~/$DATASET_NAME_NEW
cat <<EOF | ssh -t $DLBOX_IP
    rm -rf object_detection_docker/object_detection object_detection_docker/object_detection_base
    mkdir -p object_detection_docker/object_detection
    mkdir -p object_detection_docker/object_detection_base
    wget https://raw.githubusercontent.com/fanshi14/multiple-grasping-pose-learning/add_dockerfile/utils/object_detection_training_utils/docker/object_detection/Dockerfile -P object_detection_docker/object_detection/
    wget https://raw.githubusercontent.com/fanshi14/multiple-grasping-pose-learning/add_dockerfile/utils/object_detection_training_utils/docker/object_detection/docker_train.sh -P object_detection_docker/object_detection/
    sed -i 's/ITERATIONS/$ITERATION_TIMES/g' object_detection_docker/object_detection/docker_train.sh
    wget https://raw.githubusercontent.com/fanshi14/multiple-grasping-pose-learning/add_dockerfile/utils/object_detection_training_utils/docker/object_detection_base/Dockerfile -P object_detection_docker/object_detection_base/
    cp $DATASET_NAME_NEW/* object_detection_docker/object_detection/ -r
    mkdir -p $DATASET_NAME_NEW/learn
    docker build -t object_detection_base object_detection_docker/object_detection_base
    docker image rm --force object_detection
    docker build -t object_detection object_detection_docker/object_detection
    wget https://raw.githubusercontent.com/fanshi14/multiple-grasping-pose-learning/add_dockerfile/utils/object_detection_training_utils/docker/object_detection/run_detection.sh -O run_detection.sh
    wget https://raw.githubusercontent.com/fanshi14/multiple-grasping-pose-learning/add_dockerfile/utils/object_detection_training_utils/docker/object_detection/run_tensorboard.sh -O run_tensorboard.sh
    bash run_detection.sh $DATASET_NAME_NEW
EOF
scp -r $DLBOX_IP:~/$DATASET_NAME_NEW/learn $DATASET_DIR

cat <<EOF | ssh -t $DLBOX_IP
    bash run_tensorboard.sh $DATASET_NAME_NEW
EOF
