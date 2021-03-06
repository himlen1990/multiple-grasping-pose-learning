#!/usr/bin/env python3

import roslib
roslib.load_manifest("multiple_grasping_pose_learning")
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as ImageMsg
from pycoral.adapters import common
from pycoral.adapters import segment
from pycoral.utils.edgetpu import make_interpreter
from PIL import Image
import sys
import numpy as np

class semantic_segmentation_tpu:
    def __init__(self, model, labels, device=':0', keep_aspect_ratio=True):
        
        self.img_sub = rospy.Subscriber("~input", ImageMsg, self.callback)
        self.interpreter = make_interpreter(model, device=device)
        self.interpreter.allocate_tensors()
        self.model_input_width, self.model_input_height = common.input_size(self.interpreter)
        self.keep_aspect_ratio = keep_aspect_ratio
        self.bridge = CvBridge()
                
    def callback(self, data):
      
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        img = Image.fromarray(cv_image)
        if self.keep_aspect_ratio:
            resized_img, _ = common.set_resized_input(
                self.interpreter, img.size, lambda size: img.resize(size, Image.ANTIALIAS))
        else:
            resized_img = img.resize((self.model_input_width, self.model_input_height), Image.ANTIALIAS)
            common.set_input(interpreter, resized_img)

        self.interpreter.invoke()
        
        result = segment.get_output(self.interpreter)
        if len(result.shape) == 3:
            result = np.argmax(result, axis=-1)

        # If keep_aspect_ratio, we need to remove the padding area.
        new_width, new_height = resized_img.size
        result = result[:new_height, :new_width]
        # the result is a 2-d array, each element in the array is a predicted class label like 0 or 1 or 2...
        
        

def main(args):
    rospy.init_node('semantic_segmentation_server', anonymous=True)
    
    model_path = rospy.get_param('~model_path', default="../models/deeplabv3_mnv2_custom.tflite")
    label_path = rospy.get_param('~label_path', default="../models/labels.txt")    
    device = rospy.get_param('~device', default=':0')
    keep_aspect_ratio = rospy.get_param('~keep_aspect_ratio', default=True)
    sst = semantic_segmentation_tpu(model_path, label_path, device, keep_aspect_ratio)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main(sys.argv)
