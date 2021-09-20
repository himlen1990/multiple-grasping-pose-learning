import os
import sys
BASE_DIR = os.path.dirname(__file__)
sys.path.append(BASE_DIR)
sys.path.append(os.path.join(BASE_DIR, '../utils'))
import tensorflow as tf
import numpy as np
import tf_util
from pointnet_util import pointnet_sa_module, pointnet_fp_module

def placeholder_inputs(batch_size, num_point):
    pointclouds_pl = tf.placeholder(tf.float32, shape=(batch_size, num_point, 4))
    clslabels_pl = tf.placeholder(tf.int32, shape=(batch_size, num_point))
    reglabels_pl = tf.placeholder(tf.float32, shape=(batch_size, 3, 7))
    return pointclouds_pl, clslabels_pl, reglabels_pl


def get_model(point_cloud, is_training, num_class,bn_decay=None):
    """ Part segmentation PointNet, input is BxNx6 (XYZ NormalX NormalY NormalZ), output Bx50 """
    
    batch_size = point_cloud.get_shape()[0].value
    num_point = point_cloud.get_shape()[1].value
    end_points = {}
    l0_xyz = tf.slice(point_cloud, [0,0,0], [-1,-1,3])
    l0_points = tf.slice(point_cloud, [0,0,3], [-1,-1,1])

    # Set Abstraction layers
    l1_xyz, l1_points, l1_indices = pointnet_sa_module(l0_xyz, l0_points, npoint=256, radius=0.02, nsample=64, mlp=[16,16,32], mlp2=None, group_all=False, is_training=is_training, bn_decay=bn_decay, scope='layer1', bn=False)
    l2_xyz, l2_points, l2_indices = pointnet_sa_module(l1_xyz, l1_points, npoint=128, radius=0.05, nsample=64, mlp=[32,32,64], mlp2=None, group_all=False,  is_training=is_training, bn_decay=bn_decay, scope='layer2', bn=False)
    l3_xyz, l3_points, l3_indices = pointnet_sa_module(l2_xyz, l2_points, npoint=None, radius=None, nsample=None, mlp=[64,128,256], mlp2=None, group_all=True, is_training=is_training, bn_decay=bn_decay, scope='layer3', bn=False)

    # Feature Propagation layers
    l2_points = pointnet_fp_module(l2_xyz, l3_xyz, l2_points, l3_points, [64,64], is_training, bn_decay, scope='fa_layer1',bn=False)
    l1_points = pointnet_fp_module(l1_xyz, l2_xyz, l1_points, l2_points, [64,32], is_training, bn_decay, scope='fa_layer2',bn=False)
    l0_points = pointnet_fp_module(l0_xyz, l1_xyz, tf.concat([l0_xyz,l0_points],axis=-1), l1_points, [32,32,32], is_training, bn_decay, scope='fa_layer3',bn=False)

    # FC layers
    net = tf_util.conv1d(l0_points, 128, 1, padding='VALID', bn=False, is_training=is_training, scope='fc1', bn_decay=bn_decay)
    end_points['feats'] = net 
    net = tf_util.dropout(net, keep_prob=0.5, is_training=is_training, scope='dp1')
    net = tf_util.conv1d(net, num_class, 1, padding='VALID', activation_fn=None, scope='fc2')
    
    reg_net = tf.reshape(l3_points, [batch_size, -1])
    reg_net = tf_util.fully_connected(reg_net, 128, bn=False, is_training=is_training, scope='refc1', bn_decay=bn_decay)
    reg_net = tf_util.fully_connected(reg_net, 64, bn=False, is_training=is_training, scope='refc2', bn_decay=bn_decay)
    reg_net = tf_util.fully_connected(reg_net, 21, activation_fn=None, scope='refc3')
    reg_net = tf.reshape(reg_net, [batch_size, 3, 7])

    return net, reg_net, end_points



def get_loss(clspred, regpred, clslabel, reglabel):
    """ pred: BxNxC,
        label: BxN, """
    clsloss = tf.nn.sparse_softmax_cross_entropy_with_logits(logits=clspred, labels=clslabel)
    classify_loss = tf.reduce_mean(clsloss)
    regloss = tf.losses.mean_squared_error(labels=reglabel, predictions=regpred)
    total_loss = 0.1*classify_loss + 0.9*regloss

    tf.summary.scalar('classify loss', classify_loss)
    tf.add_to_collection('losses', classify_loss)
    return classify_loss,regloss,total_loss

if __name__=='__main__':
    with tf.Graph().as_default():
        inputs = tf.zeros((32,2048,6))
        net, _ = get_model(inputs, tf.constant(True))
        print(net)
