# FROM nvidia/cudagl:10.0-base-ubuntu16.04
FROM nvidia/cuda:10.0-cudnn7-devel-ubuntu16.04
ENV LANG C.UTF-8
RUN apt-get update && apt-get install -y git wget apt-utils
RUN apt-get install -y software-properties-common
RUN apt-get update

## python
# RUN apt-get install -y cmake python3.6-dev python3-pip libpython3.6-dev libeigen3-dev

## Anaconda environment
SHELL ["/bin/bash", "-c"]
RUN apt-get install -y libgl1-mesa-glx libegl1-mesa libxrandr2 libxrandr2 libxss1 libxcursor1 libxcomposite1 libasound2 libxi6 libxtst6
RUN wget --quiet https://repo.anaconda.com/archive/Anaconda3-2021.05-Linux-x86_64.sh && \
    bash Anaconda3-2021.05-Linux-x86_64.sh -b -p /opt/anaconda3 && \
    rm Anaconda3-2021.05-Linux-x86_64.sh && \
    echo "export PATH=/opt/anaconda3/bin:$PATH" >> ~/.bashrc && \
    source ~/.bashrc && \
    source /opt/anaconda3/bin/activate
ENV PATH /opt/anaconda3/bin:$PATH
RUN conda config --append channels conda-forge && \
    source /opt/anaconda3/bin/activate && \
    # conda update --all && \
    conda init && \
    conda create -n aero_train python==3.6 -y && \
    echo "source /opt/anaconda3/bin/activate" >> ~/.bashrc && \
    echo "conda activate aero_train" >> ~/.bashrc

## pip package for object detection
RUN source /opt/anaconda3/bin/activate && \
    conda activate aero_train && \
    pip install Cython pandas scipy==1.5.4 tf-slim==1.1.0 matplotlib==3.3.4 six==1.16.0 opencv-python==4.5.2.52 tensorflow-gpu==1.15.3 && \
    cd ~/ && \
    mkdir Tensorflow && \
    cd Tensorflow && \
    git clone https://github.com/tensorflow/models.git && \
    cd models/research/ && \
    wget https://github.com/protocolbuffers/protobuf/releases/download/v3.17.3/protoc-3.17.3-linux-x86_64.zip && \
    apt-get install -y unzip gcc && \
    mkdir protoc-3.17.3-linux-x86_64 && \
    cd protoc-3.17.3-linux-x86_64 && \
    unzip ../protoc-3.17.3-linux-x86_64.zip && \
    cd .. && \
    protoc-3.17.3-linux-x86_64/bin/protoc object_detection/protos/*.proto --python_out=. && \
    cp object_detection/packages/tf1/setup.py . && \
    python setup.py install
RUN cd ~/Tensorflow && \
    mkdir demo && \
    cd demo && \
    mkdir images annotations pre-trained-models

# install edgetpu compiler
RUN curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | apt-key add - && \
    echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | tee /etc/apt/sources.list.d/coral-edgetpu.list && \
    apt-get update && apt-get install -y edgetpu-compiler

## multiple_grasping_pose_learning
RUN cd ~/ && \
    git clone https://github.com/himlen1990/multiple-grasping-pose-learning.git && \
    cp -r ~/multiple-grasping-pose-learning/utils/object_detection_training_utils/* ~/Tensorflow/demo/ && \
    cd ~/Tensorflow/demo/pre-trained-models && \
    wget http://download.tensorflow.org/models/object_detection/ssd_mobilenet_v2_quantized_300x300_coco_2019_01_03.tar.gz && \
    tar -xvzf ssd_mobilenet_v2_quantized_300x300_coco_2019_01_03.tar.gz --no-same-owner

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility
EXPOSE 6006