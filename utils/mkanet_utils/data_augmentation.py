from plyfile import (PlyData, PlyElement, make2d, PlyParseError, PlyProperty)
import numpy as np
import random
import h5py
import glob
import os
import math


def generate_scene_grasp(obj_dir, num_scenes=100):
    generated_scene = []
    generated_scene_label = []
    obj = []
    label = []
    num_points = 2400
    for obj_num in glob.glob(os.path.join(obj_dir,'*')):
        for plyfile in glob.glob(os.path.join(obj_num,'*.ply')):
            print "processing file: ",plyfile
            plydata = PlyData.read(plyfile)
            pc = plydata['vertex'].data
            pc_void_to_array = np.asarray(pc.tolist())[:,:3]
            function_part = pc_void_to_array[:,:3]
            centroid = np.mean(function_part, axis=0)
            #print centroid
            txtfile = plyfile[:-4] + '.txt'
            labeldata = np.loadtxt(txtfile)
            labeldata = np.reshape(labeldata, (-1,7))
            transform =  labeldata[:,:3] - centroid
            newlabel = np.c_[transform,labeldata[:,3:]]
            newlabel = np.append(newlabel,np.zeros((1,7)),axis=0)
            column_padding = np.zeros((pc_void_to_array.shape[0],2))
            padded_pc = np.c_[pc_void_to_array, column_padding]
            obj.append(padded_pc)
            label.append(newlabel)

    for i in range(num_scenes):
        cloud_idx = random.sample(range(len(obj)),1)
    
        jitter_x = random.uniform(0,1.0)
        jitter_y = random.uniform(-1.0,1.0)
        jitter_z = random.uniform(0,1.5)            
        jittered_cloud = obj[cloud_idx[0]].copy()

        jittered_cloud[:,0] = jittered_cloud[:,0] + jitter_x
        jittered_cloud[:,1] = jittered_cloud[:,1] + jitter_y
        jittered_cloud[:,2] = jittered_cloud[:,2] + jitter_z

        jittered_cloud[:,3] = 1
        jittered_cloud[:,4] = 1
        scene = jittered_cloud[:,:5]
        
        if scene.shape[0]>num_points:
            sample_idx = random.sample(range(scene.shape[0]), num_points)
            reshape_pc = scene[sample_idx]
            
        elif scene.shape[0]<num_points:
            num_padding = num_points - scene.shape[0]
            padding = np.zeros((num_padding, scene.shape[1]), dtype = scene.dtype)
            reshape_pc = np.append(scene,padding, axis=0)


        generated_scene.append(reshape_pc)
        generated_scene_label.append(label[cloud_idx[0]])
            
    return  np.array(generated_scene),np.array(generated_scene_label)    


if __name__ == "__main__":


        
    obj_dir = "./dataset/"
    data_arr,label_arr = generate_scene_grasp(obj_dir,500)
    print data_arr.shape, label_arr.shape
    data_dtype = "float32"
    file_name = "./grasp_augment.h5"
    print file_name
    h5_fout=h5py.File(file_name,'w')
    h5_fout.create_dataset('data', data=data_arr,
                           compression='gzip', compression_opts=4,
                           dtype=data_dtype)
    h5_fout.create_dataset('label', data=label_arr,
                           compression='gzip', compression_opts=4,
                           dtype=data_dtype)
    h5_fout.close()    
    f = h5py.File(file_name)
    data = f['data']
    label = f['label']
    print data.shape,label.shape
    print label[0]

        


