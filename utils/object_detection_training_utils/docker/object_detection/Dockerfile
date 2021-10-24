FROM object_detection_base:latest
ENV LANG C.UTF-8

COPY test.zip /tmp
COPY train.zip /tmp
COPY pipeline.config /tmp
COPY label_map.pbtxt /tmp

RUN cp /tmp/test.zip ~/Tensorflow/demo/images/ && \
    cp /tmp/train.zip ~/Tensorflow/demo/images/ && \
    cp /tmp/pipeline.config ~/Tensorflow/demo/models/my_ssd/ && \
    cp /tmp/label_map.pbtxt ~/Tensorflow/demo/annotations/

RUN cd ~/Tensorflow/demo/images/ && \
    unzip test.zip && \
    unzip train.zip

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility
EXPOSE 6006

# RUN source /opt/anaconda3/bin/activate && \
#     conda activate aero_train && \
#     cd ~/Tensorflow/demo/ && \
#     sh generate_record.sh && \
#     sh ~/Tensorflow/demo/train.sh
