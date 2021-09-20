#!/usr/bin/env python3

import roslib
roslib.load_manifest("multiple_grasping_pose_learning")
import rospy
import cv2
import collections
from cv_bridge import CvBridge
import numpy as np
from PIL import Image
from PIL import ImageDraw, ImageFont
from sensor_msgs.msg import Image as ImageMsg
from pycoral.adapters import common
from pycoral.adapters import detect
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter
from multiple_grasping_pose_learning.srv import *

class object_detection_server:
  def __init__(self, model_path, label_path,score_threshold,tile_sizes_str):
    self.model = model_path
    self.label = label_path
    self.tile_sizes_str = tile_sizes_str
    self.tile_overlap =  20
    self.score_threshold = score_threshold
    self.iou_threshold = 0.1
    self.Object = collections.namedtuple('Object', ['label', 'score', 'bbox'])

    self.interpreter = make_interpreter(self.model)
    self.interpreter.allocate_tensors()
    self.labels = read_label_file(self.label) if self.label else {}
    self.object_detection_srv = rospy.Service("aero_goods_demo_object_detect", objectdetect, self.srv_callback)
    self.bridge = CvBridge()
    self.tile_sizes = []
    for tile_size in self.tile_sizes_str.split(','):
      size = tile_size.split('x')
      tile_size_int = [int(i) for i in size]
      self.tile_sizes.append(tile_size_int)

    rospy.loginfo("object_detection_server: ready to server")
    
  def tiles_location_gen(self, img_size, tile_size, overlap):

    tile_width, tile_height = tile_size
    img_width, img_height = img_size
    h_stride = tile_height - overlap
    w_stride = tile_width - overlap
    for h in range(0, img_height, h_stride):
      for w in range(0, img_width, w_stride):
        xmin = w
        ymin = h
        xmax = min(img_width, w + tile_width)
        ymax = min(img_height, h + tile_height)
        yield [xmin, ymin, xmax, ymax]


  def non_max_suppression(self, objects, threshold):
    if len(objects) == 1:
      return [0]

    boxes = np.array([o.bbox for o in objects])
    xmins = boxes[:, 0]
    ymins = boxes[:, 1]
    xmaxs = boxes[:, 2]
    ymaxs = boxes[:, 3]

    areas = (xmaxs - xmins) * (ymaxs - ymins)
    scores = [o.score for o in objects]
    idxs = np.argsort(scores)

    selected_idxs = []
    while idxs.size != 0:

      selected_idx = idxs[-1]
      selected_idxs.append(selected_idx)

      overlapped_xmins = np.maximum(xmins[selected_idx], xmins[idxs[:-1]])
      overlapped_ymins = np.maximum(ymins[selected_idx], ymins[idxs[:-1]])
      overlapped_xmaxs = np.minimum(xmaxs[selected_idx], xmaxs[idxs[:-1]])
      overlapped_ymaxs = np.minimum(ymaxs[selected_idx], ymaxs[idxs[:-1]])

      w = np.maximum(0, overlapped_xmaxs - overlapped_xmins)
      h = np.maximum(0, overlapped_ymaxs - overlapped_ymins)

      intersections = w * h
      unions = areas[idxs[:-1]] + areas[selected_idx] - intersections
      ious = intersections / unions

      idxs = np.delete(
        idxs, np.concatenate(([len(idxs) - 1], np.where(ious > threshold)[0])))

    return selected_idxs


  def draw_object(self, draw, obj):

    draw.rectangle(obj.bbox, outline='red')
    font = ImageFont.truetype("DejaVuSans.ttf", 20)
    draw.text((obj.bbox[0], obj.bbox[3] - 20), obj.label, fill='#0000',font=font)
    draw.text((obj.bbox[0], obj.bbox[3] + 10), str(obj.score), fill='#0000',font=font)


  def reposition_bounding_box(self, bbox, tile_location):
    bbox[0] = bbox[0] + tile_location[0]
    bbox[1] = bbox[1] + tile_location[1]
    bbox[2] = bbox[2] + tile_location[0]
    bbox[3] = bbox[3] + tile_location[1]
    return bbox

  def srv_callback(self,req):
    rospy.loginfo("object_detection_server: got request")
    image = req.Image  
    cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
    img_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    img = Image.fromarray(img_rgb)
    draw = ImageDraw.Draw(img)
    
    objects_by_label = dict()
    img_size = img.size
    for tile_size in self.tile_sizes:
      for tile_location in self.tiles_location_gen(img_size, tile_size,
                                                   self.tile_overlap):
        tile = img.crop(tile_location)
        _, scale = common.set_resized_input(
          self.interpreter, tile.size,
          lambda size, img=tile: img.resize(size, Image.NEAREST))
        self.interpreter.invoke()
        objs = detect.get_objects(self.interpreter, self.score_threshold, scale)

        for obj in objs:
          bbox = [obj.bbox.xmin, obj.bbox.ymin, obj.bbox.xmax, obj.bbox.ymax]
          bbox = self.reposition_bounding_box(bbox, tile_location)

          label = self.labels.get(obj.id, '')
          objects_by_label.setdefault(label,
                            []).append(self.Object(label, obj.score, bbox))

    ObjLabels_list = []
    Scores_list = []
    Bboxes_list_flat = []
    
    for label, objects in objects_by_label.items():
      idxs = self.non_max_suppression(objects, self.iou_threshold)
      for idx in idxs:
        ObjLabels_list.append(int(objects[idx].label))
        Scores_list.append(float(objects[idx].score))
        Bboxes_list_flat.append(objects[idx].bbox)
        self.draw_object(draw, objects[idx])
    Bboxes_list_flat = np.array(Bboxes_list_flat).flatten().tolist()
    #print(Bboxes_list_flat)
    #cvimg = np.array(img)
    #cv_show = cv2.cvtColor(cvimg, cv2.COLOR_BGR2RGB)
    #cv2.imshow("output",cv_show)
    #cv2.waitKey()
    
    resp = objectdetectResponse()
    resp.ObjLabels = ObjLabels_list
    resp.Scores = Scores_list
    resp.Bboxes = Bboxes_list_flat
    return resp
    
def main():
  rospy.init_node('object_detection_server', anonymous=True)
  model_path = rospy.get_param('~model_path')
  label_path = rospy.get_param('~label_path')
  score_threshold = rospy.get_param('~score_threshold')
  tile_sizes = rospy.get_param('~tile_sizes',default="500x500,300x300,250x250")
  ods = object_detection_server(model_path, label_path,score_threshold,tile_sizes)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
  main()
