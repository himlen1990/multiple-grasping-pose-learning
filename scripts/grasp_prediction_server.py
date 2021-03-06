#!/usr/bin/python
import roslib
roslib.load_manifest("multiple_grasping_pose_learning")
import rospy
import numpy as np
import os
import sys

base_dir = os.path.dirname(os.path.abspath(__file__))
root_dir = os.path.dirname(base_dir)
sys.path.append(base_dir)
sys.path.append(os.path.join(root_dir, 'models/mkamodel'))
sys.path.append(os.path.join(root_dir, 'utils/mkanet_utils/pointnet_utils'))

from visualization_msgs.msg import MarkerArray, Marker
from multiple_grasping_pose_learning.srv import *
import tensorflow as tf
import mkanet as model
import random


class grasp_prediction_server:
    def __init__(self):
        self.num_class = 2
        self.num_points = 2400
        #tensorflow related
        self.sess = tf.Session()
        self.pointclouds_pl, clslabels_pl, reglabels_pl = model.placeholder_inputs(1,self.num_points)
        self.is_training_pl = tf.placeholder(tf.bool,shape=())
        self.clspred, self.regpred, end_points = model.get_model(self.pointclouds_pl, self.is_training_pl, self.num_class)
        saver = tf.train.Saver()
        saver.restore(self.sess, os.path.join(root_dir, 'models/mkamodel/log/model.ckpt'))
        #ROS related
        self.mechknownetsrv = rospy.Service('aero_goods_demo_grasp_predict', grasppredict, self.srvCb)
        self.grasp_marker_pub = rospy.Publisher("aero_goods_demo_grasp_marker",MarkerArray, queue_size=1)
        rospy.loginfo("grasp_prediction_server: ready to serve")        
 
         
    def srvCb(self,req):

        rospy.loginfo("grasp_prediction_server: got request")
        resp = grasppredictResponse()
        target_obj_arr = np.asarray(req.TargetObjectFlatPointCloud)
        target_obj_point_cloud = np.reshape(target_obj_arr, (-1,3))
        network_input = self.pointcloud_processor(target_obj_point_cloud)
        segpred_g, regpred_g = self.predict(np.expand_dims(network_input,0))
        grasp_result = np.c_[network_input, segpred_g[0]]
        recoverd_grasp_poses = self.recover_grasp_data(grasp_result, regpred_g[0])

        final_pose = recoverd_grasp_poses[:2]
        print final_pose
        self.show_marker(final_pose)

        if(np.isnan(final_pose[0][0])):
            print "nan detected"
            resp.GraspingPose = []
            return resp
        else:                
            resp.GraspingPose = final_pose.flatten().tolist()     
            return resp


    def pointcloud_processor(self,obj):
        extended_obj = np.c_[obj,np.ones(obj.shape[0])]

        if extended_obj.shape[0] > self.num_points:
            idx = random.sample(range(extended_obj.shape[0]), self.num_points)
            network_input = extended_obj[idx,:]

        elif extended_obj.shape[0] < self.num_points:
            num_padding = self.num_points - extended_obj.shape[0]
            padding = np.zeros((num_padding, extended_obj.shape[1]), dtype = extended_obj.dtype)
            network_input = np.append(extended_obj, padding, axis=0)
        return network_input

    def predict(self,points):
        feed_dict = {self.pointclouds_pl: points,
                     self.is_training_pl: False}
        logits, reglabel = self.sess.run([self.clspred, self.regpred],feed_dict=feed_dict)
        return np.argmax(logits,2), reglabel

    def recover_grasp_data(self, pc, label):
        function_part = pc[:,:3] #the whole object
        #do not count zeros when computing centroid
        index = -1
        for i in range(0,function_part.shape[0]):
            if function_part[i][0] == 0 and function_part[i][1] == 0 and function_part[i][2] == 0:
                index = i
                break
        if index!= -1:
            function_part_no_zeros = function_part[:index]
            centroid = np.mean(function_part_no_zeros,axis=0)
        else:
            centroid = np.mean(function_part,axis=0)
        recover_label = label[:,:3] + centroid
        recover_label = np.c_[recover_label,label[:,3:]]
        return recover_label
    
    def show_marker(self,poses):
        #grasp marker
        marker_array = MarkerArray()
        for i in range(0,poses.shape[0]):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = rospy.Time.now()
            marker.type = marker.MESH_RESOURCE
            marker.mesh_resource = "package://multiple_grasping_pose_learning/gripper.dae"
            marker.action = marker.ADD
            marker.id = 0+i
            marker.pose.position.x = poses[i][0]
            marker.pose.position.y = poses[i][1]
            marker.pose.position.z = poses[i][2]
            marker.pose.orientation.w = poses[i][3]
            marker.pose.orientation.x = poses[i][4]
            marker.pose.orientation.y = poses[i][5]
            marker.pose.orientation.z = poses[i][6]
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.15
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        self.grasp_marker_pub.publish(marker_array)


if __name__ == '__main__':
    rospy.init_node("grasp_prediction_server", anonymous=True)
    gps = grasp_prediction_server()
    rospy.spin()

