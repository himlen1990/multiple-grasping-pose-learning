import cv2
import numpy as np
import json
import glob
import os
import base64
import PIL.Image
import cStringIO
from collections import OrderedDict


if __name__ == "__main__":
    data_filename = []    
    data_content = []
    rgb_data_path = "../dataset/rgb"
    labelme_version = "4.5.7"
    
    for obj_num in glob.glob(os.path.join(rgb_data_path, '*')):
        for rgbfile in glob.glob(os.path.join(obj_num, '*jpg')):
            suffix =  rgbfile.split("/")[-1]
            size = os.path.getsize(rgbfile)
            maskfile = rgbfile.replace("rgb","mask")
            img = cv2.imread(maskfile)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            img2 = cv2.medianBlur(gray,5)
            ret, thresh = cv2.threshold(img2, 1, 255, 0)
            im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            points = np.empty((0,2),int)
            object_index = obj_num.split("/")[-1]
            for i in range(len(contours)):    
                contour_points =  contours[i].reshape([-1,2])
                points= np.append(points,contour_points,axis=0)
            sampled_point = points[::8]

            shapes = [
                dict(
                    label = obj_num.split("/")[-1],
                    points = sampled_point.tolist(),
                    shape_type = ("polygon"),
                    flags = {},
                    group_id = "null"
                    )
            ]

            imageData = PIL.Image.open(rgbfile)
            imageWidth, imageHeight = imageData.size
            buffer = cStringIO.StringIO()
            imageData.save(buffer, format="JPEG")
            imageData = base64.b64encode(buffer.getvalue())
            

            data = dict(
                version = labelme_version,
                flags = {},
                shapes = shapes,
                imagePath = suffix,
                imageData = imageData,
                imageHeight = imageHeight,
                imageWidth = imageWidth
            )            
            filename = rgbfile.replace("jpg","json")
            
            with open(filename, "w") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
