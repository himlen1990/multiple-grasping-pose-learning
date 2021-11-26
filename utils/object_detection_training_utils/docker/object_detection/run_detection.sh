#!/bin/bash

function message() {
  local color=$1; shift;
  local message=$@

  # Output command being executed
  echo -e "\e[${color}m${message}\e[0m"
}

THIS_DIR="$PWD"

if [ -z "$1" ]; then
    message 31 "Set path to dataset is required"
    exit 1
fi

DATASET_DIR=$(realpath $1); shift 1;
DATASET_NAME=$(basename $DATASET_DIR)

PORT=6006
DOCKER_OPTION=""
DOCKER_PORT_OPTION=""
RUN_TENSORBOARD=0
RUN_BASH=0
if [ "$1" == "bash" -o "$1" == "/bin/bash" ]; then
    DOCKER_OPTION="";
    RUN_BASH=1
else
    for i in `seq 1 $#`; do
        if [ "${!i}" == "tensorboard" ]; then
            RUN_TENSORBOARD=1
        elif [ "${!i}" == "--port" ]; then
            j=$(expr $i + 1)
            PORT=${!j}
        fi;
    done
    if [[  RUN_TENSORBOARD -eq 1 ]]; then
        DOCKER_PORT_OPTION="-p $PORT:$PORT"
    fi
fi

mkdir -p ${DATASET_DIR}/learn
if [ -t 1 ]; then
    TTY_OPT='-ti'
else
    TTY_OPT=''
fi

set -x
docker run --rm --privileged \
       --userns=host \
       --gpus all \
       --mount type=bind,src=${DATASET_DIR}/learn,dst=/root/Tensorflow/demo/models_test \
       ${TTY_OPT} object_detection bash docker_train.sh $2
set +x

if [ $RUN_BASH -eq 0 -a $RUN_TENSORBOARD -eq 0 ]; then
  message 32 "Done generating model file for edgetpu object detection"
  message 32 " - ${DATASET_DIR}/learn/models/labels.txt"
  message 32 " - ${DATASET_DIR}/learn/models/output_tflite_graph.tflite"
  message 32 " - ${DATASET_DIR}/learn/models/output_tflite_graph_edgetpu.tflite"
fi
