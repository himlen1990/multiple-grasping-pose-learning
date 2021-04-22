import cv2
import numpy as np
import json
import glob
import os
    
if __name__ == "__main__":
    data_filename = []    
    data_content = []
    rgb_data_path = "../dataset/rgb"
    
    for obj_num in glob.glob(os.path.join(rgb_data_path, '*')):
        for rgbfile in glob.glob(os.path.join(obj_num, '*jpg')):
            suffix =  rgbfile.split("/")[-1]
            size = os.path.getsize(rgbfile)
            filename = suffix + str(size)
            print filename
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
            all_points_x = sampled_point[:,0].tolist()
            all_points_y = sampled_point[:,1].tolist()

            region_attributes = dict(
                object_index = object_index
            )
            file_attributes = dict()
            labeltype = "polygon"
            shape_attributes = dict(
                name = labeltype,
                all_points_x = all_points_x,
                all_points_y = all_points_y,
                region_attributes = region_attributes
            )
            regions = dict(
                shape_attributes = shape_attributes
            )
            filedata = dict(
                file_attributes = file_attributes,
                filename = suffix,
                size = size,
                regions = [regions]
            )
            

        
            data_filename.append(filename)
            data_content.append(filedata)
    data = {}
    for i in range(len(data_filename)):
        data[data_filename[i]] = data_content[i]
    with open("label.json","w") as f:
        json.dump(data,f)

        
