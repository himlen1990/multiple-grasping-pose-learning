import cv2
import numpy as np
import glob
import os
import base64
import PIL.Image
import cStringIO
# from io import StringIO
import random
from collections import OrderedDict
from dict2xml import dict2xml

rgb_data_path = "../dataset/rgb"
background_data_path = "../dataset/background"
augmented_data_path = "../dataset/aug_data"
background_imgs = []

def changedSV(bgr_img, alpha, beta, color_idx):
    hsvimage = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV_FULL) # BGR->HSV
    hsvf = hsvimage.astype(np.float32)
    hsvf[:,:,color_idx] = np.clip(hsvf[:,:,1] * alpha+beta, 0, 255)
    hsv8 = hsvf.astype(np.uint8)
    return cv2.cvtColor(hsv8,cv2.COLOR_HSV2BGR_FULL)


def data_generation(rgbs,masks,labels,countour_points,roi_dimensions,num_objs,num_data,img_counter):    
    image_counter = img_counter
    for data_id in range(num_data):
        #randomly choose a background
        random_background = np.copy(random.sample(background_imgs,1))[0]
        bg_width = random_background.shape[1]
        bg_height = random_background.shape[0]

        #set the output image size to 300x300
        crop_width = 300 
        crop_height = 300
        # crop_width = 640
        # crop_height = 480

        ## random background region
        height_rand_offset = (bg_height-crop_height) / 2 - 30 + np.random.randint(30)
        # height_rand_offset = (bg_height-crop_height) / 2 ## w/o random

        width_rand_offset = 30 + np.random.randint(40)
        # width_rand_offset = 50 ## w/o random

        # random_background = random_background[bg_height-crop_height:,bg_width-crop_width:,:] ## w/o random
        random_background = random_background[(bg_height-crop_height) / 2 + height_rand_offset:(bg_height+crop_height) / 2 + height_rand_offset, (bg_width-crop_width) / 2 + width_rand_offset:(bg_width+crop_width) / 2 + width_rand_offset,:]
        random_background_mask = np.zeros((crop_height,crop_width,3),random_background.dtype)

        #randomly choose n objects
        sample = random.sample(range(0,len(target_obj_rgbs)),num_objs)
        objects = []
        objects_position = []
        for i in range(num_objs):
            rgb_roi = rgbs[sample[i]]
            mask_roi = masks[sample[i]]
            roi_min_y = roi_dimensions[sample[i]][0]
            roi_min_x = roi_dimensions[sample[i]][1]
            roi_height = roi_dimensions[sample[i]][2]
            roi_width = roi_dimensions[sample[i]][3]
            countour_point = countour_points[sample[i]]

            # ratio_x = random.random()
            # ratio_y = random.random()

            ### avoid self-occlusion
            max_trails = 10 ## try maximum 10 times until no self-occlusion happens
            translate_roi_min_x = translate_roi_min_y = translate_roi_max_x = translate_roi_max_y = 0
            for trail_id in range(0, max_trails):
                ratio_x = random.random()
                # ratio_y = random.random()
                ratio_y = random.random() * 2.0 / 4.0 + 2.0 / 4.0 ## put object only in the lower side of background to simulate the real position

                ## constraint its position to avoid unfeasible region
                ## x: 180 - 500, y: 100 - 420 (task-related value)
                if crop_width == 640:
                    ratio_x = (ratio_x * (500 - 180) + 180) / float(crop_width)
                    ratio_y = (ratio_y * (420 - 100) + 100) / float(crop_height)

                translate_roi_min_x = int(ratio_x * crop_width)
                translate_roi_min_y = int(ratio_y * crop_height)

                if translate_roi_min_x + roi_width > crop_width:
                    translate_roi_min_x = crop_width - roi_width
                if translate_roi_min_y + roi_height > crop_height:
                    translate_roi_min_y = crop_height - roi_height

                translate_roi_max_x = translate_roi_min_x + roi_width
                translate_roi_max_y = translate_roi_min_y + roi_height
                if i == 0 or trail_id == max_trails - 1:
                    objects_position.append([translate_roi_min_x, translate_roi_max_x, translate_roi_min_y, translate_roi_max_y])
                    break
                ## check if self occulusion happens
                current_vertices = [[translate_roi_min_x, translate_roi_min_y], [translate_roi_min_x, translate_roi_max_y], [translate_roi_max_x, translate_roi_min_y], [translate_roi_max_x, translate_roi_max_y]]
                self_occulusion = False
                for prev_object_id in range(0, len(objects_position)):
                    for vertice_id in range(0, 4):
                        if (current_vertices[vertice_id][0] > objects_position[prev_object_id][0] and current_vertices[vertice_id][0] < objects_position[prev_object_id][1] and current_vertices[vertice_id][1] > objects_position[prev_object_id][2] and current_vertices[vertice_id][1] < objects_position[prev_object_id][3]) or (current_vertices[vertice_id][0] < objects_position[prev_object_id][0] and current_vertices[vertice_id][0] > objects_position[prev_object_id][1] and current_vertices[vertice_id][1] < objects_position[prev_object_id][2] and current_vertices[vertice_id][1] > objects_position[prev_object_id][3]):
                            self_occulusion = True
                            break
                    if self_occulusion:
                        break
                if self_occulusion:
                    continue
                else:
                    objects_position.append([translate_roi_min_x, translate_roi_max_x, translate_roi_min_y, translate_roi_max_y])
                    break

            random_background[translate_roi_min_y:translate_roi_max_y,translate_roi_min_x:translate_roi_max_x] = rgb_roi

            random_background_mask[translate_roi_min_y:translate_roi_max_y,translate_roi_min_x:translate_roi_max_x] = mask_roi


            countour_translate_x = roi_min_x - translate_roi_min_x
            countour_translate_y = roi_min_y - translate_roi_min_y
            translated_countour_point = countour_point - np.array([countour_translate_x,countour_translate_y])


            boundingbox =  OrderedDict([
                ("xmin", translate_roi_min_x),
                ("ymin", translate_roi_min_y),
                ("xmax", translate_roi_max_x),
                ("ymax", translate_roi_max_y)
                ])
            obj =  OrderedDict([
                ("name", labels[sample[i]]),
                ("pose", "Unspecified"),
                ("truncated", 0),
                ("difficult", 0),
                ("bndbox",boundingbox)
            ])

            objects.append(obj)

        ## randomize data by adjusting hsv
        hsv_image = cv2.cvtColor(random_background, cv2.COLOR_BGR2HSV)
        hsvf = hsv_image.astype(np.float32)
        hsvf[:,:,2] = np.clip(hsvf[:,:,2] * (1.0 + np.random.randint(-10, 10) / 100.0), 0, 255)
        hsv8 = hsvf.astype(np.uint8)
        random_background = cv2.cvtColor(hsv8,cv2.COLOR_HSV2BGR)

        save_rgb = augmented_data_path + "/rgb/frame%04d.jpg" %image_counter
        save_mask = augmented_data_path + "/mask/frame%04d.jpg" %image_counter
        suffix = "frame%04d.jpg" %image_counter
        #path = "/home/himlen/TensorFlow/training_demo/images/frame%04d.jpg" %image_counter
        path = "does not matter"
        image_counter = image_counter + 1
        cv2.imwrite(save_rgb,random_background)
        cv2.imwrite(save_mask,random_background_mask)

        convert_color = cv2.cvtColor(random_background,cv2.COLOR_BGR2RGB)

        imageData = PIL.Image.fromarray(convert_color)
        imageWidth, imageHeight = imageData.size
        buffer = cStringIO.StringIO()
        imageData.save(buffer, format="JPEG")
        imageData = base64.b64encode(buffer.getvalue())
        source = {'database': "Unknown"}
        data = OrderedDict([
            ("folder","images"),
            ("filename",suffix),                    
            ("path", path),
            ("source", {'database': "Unknown"}),
            ("size", OrderedDict([("width", crop_width),("height",crop_height),("depth",3)])),
            ("segmented", 0),
            ("object", []),
        ])
        for i in range (len(objects)):
            data["object"].append(objects[i])
        filename = save_rgb.replace("jpg","xml")
        xml = dict2xml(data, wrap = "annotation", indent=" ")
        with open(filename, "w") as f:
            f.write(xml)
        f.close()
    '''
    imageset_path = augmented_data_path + "/ImageSets/"
    total_image = np.arange(image_counter)
    np.random.shuffle(total_image)
    rate = int(image_counter * 0.75)
    train = total_image[:rate]
    val = total_image[rate:]
    train_file_path = imageset_path+"train.txt"
    trainval_file_path = imageset_path+"trainval.txt"
    val_file_path = imageset_path+"val.txt"
    f = open(train_file_path,"w")
    for i in train:
        f.write("frame%03d\n" %i)
    f.close()
    f = open(trainval_file_path,"w")
    for i in total_image:
        f.write("frame%03d\n" %i)
    f.close()
    f = open(val_file_path,"w")
    for i in val:
        f.write("frame%03d\n" %i)
    f.close()
    '''
    return image_counter

if __name__ == "__main__":
    #load background image
    for backgroundfile in glob.glob(os.path.join(background_data_path, '*jpg')):
        background_imgs.append(cv2.imread(backgroundfile))

    target_obj_rgbs = []
    target_obj_masks = []
    target_obj_labels = []
    target_obj_countour_points = []
    target_obj_roi_dimensions = []
    for obj_num in glob.glob(os.path.join(rgb_data_path, '*')):
        for rgbfile in glob.glob(os.path.join(obj_num, '*jpg')):
            suffix =  rgbfile.split("/")[-1]
            size = os.path.getsize(rgbfile)
            maskfile = rgbfile.replace("rgb","mask")
            mask_img = cv2.imread(maskfile)
            gray = cv2.cvtColor(mask_img, cv2.COLOR_BGR2GRAY)
            img2 = cv2.medianBlur(gray,5)
            ret, thresh = cv2.threshold(img2, 1, 255, 0)
            # im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            points = np.empty((0,2),int)            
            for i in range(len(contours)):    
                contour_points =  contours[i].reshape([-1,2])
                points= np.append(points,contour_points,axis=0)
            sampled_point = points[::8]
            sorted_point = np.sort(points,axis=0)
            rgb_img = cv2.imread(rgbfile)
            min_x = sorted_point[0][0] - np.random.randint(10)
            min_y = sorted_point[0][1] - np.random.randint(10)
            max_x = sorted_point[-1][0] + np.random.randint(10)
            max_y = sorted_point[-1][1] + np.random.randint(10)
            if(min_x<0):
                min_x = 0
            if(min_y<0):
                min_y = 0
            if(max_x>rgb_img.shape[1]):
                max_x = rgb_img.shape[1]
            if(max_y>rgb_img.shape[0]):
                max_y = rgb_img.shape[0]

            height = max_y - min_y
            width  = max_x - min_x
            roi_dimension  = [min_y,min_x,height,width]
            target_obj = rgb_img[min_y:max_y,min_x:max_x]
            target_obj_mask = mask_img[min_y:max_y,min_x:max_x]
            
            target_obj_rgbs.append(target_obj)
            target_obj_masks.append(target_obj_mask)
            target_obj_labels.append(obj_num.split("/")[-1])
            target_obj_countour_points.append(sampled_point)
            target_obj_roi_dimensions.append(roi_dimension)

    # img_counter = data_generation(target_obj_rgbs,target_obj_masks,target_obj_labels,target_obj_countour_points,target_obj_roi_dimensions,1,200,0)
    # img_counter = data_generation(target_obj_rgbs,target_obj_masks,target_obj_labels,target_obj_countour_points,target_obj_roi_dimensions,2,200,img_counter)
    # img_counter = data_generation(target_obj_rgbs,target_obj_masks,target_obj_labels,target_obj_countour_points,target_obj_roi_dimensions,3,200,img_counter)
    # img_counter = data_generation(target_obj_rgbs,target_obj_masks,target_obj_labels,target_obj_countour_points,target_obj_roi_dimensions,4,200,img_counter)
    # img_counter = data_generation(target_obj_rgbs,target_obj_masks,target_obj_labels,target_obj_countour_points,target_obj_roi_dimensions,5,200,img_counter)

    # img_counter = data_generation(target_obj_rgbs,target_obj_masks,target_obj_labels,target_obj_countour_points,target_obj_roi_dimensions,1,250,0)
    # img_counter = data_generation(target_obj_rgbs,target_obj_masks,target_obj_labels,target_obj_countour_points,target_obj_roi_dimensions,2,250,img_counter)
    # img_counter = data_generation(target_obj_rgbs,target_obj_masks,target_obj_labels,target_obj_countour_points,target_obj_roi_dimensions,3,250,img_counter)
    # img_counter = data_generation(target_obj_rgbs,target_obj_masks,target_obj_labels,target_obj_countour_points,target_obj_roi_dimensions,4,250,img_counter)

    img_counter = data_generation(target_obj_rgbs,target_obj_masks,target_obj_labels,target_obj_countour_points,target_obj_roi_dimensions,1,300,0)
    img_counter = data_generation(target_obj_rgbs,target_obj_masks,target_obj_labels,target_obj_countour_points,target_obj_roi_dimensions,2,300,img_counter)
    img_counter = data_generation(target_obj_rgbs,target_obj_masks,target_obj_labels,target_obj_countour_points,target_obj_roi_dimensions,3,400,img_counter)
