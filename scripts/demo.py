import roslib
roslib.load_manifest('multiple_grasping_pose_learning')
import rospy
import numpy as np
from multiple_grasping_pose_learning.srv import *


def demo():
    rospy.init_node('aero_goods_grasping_demo', anonymous=True)
    success = call_deploy("1",[0.6, -0.3, 0.93, 0.95, 0.2, 1.2]) #obj_cls and boundingbox [xmin,ymin,zmin,xmax,ymax,zmax]

    print "success? ",success
    if success:
        print "ready to go"

def call_deploy(tagetObj,workspace):
    got_result_flag = False
    rospy.wait_for_service('aero_goods_grasping_deploy')
    try:
        deploy_srv = rospy.ServiceProxy('aero_goods_grasping_deploy', graspanobject)
        response = deploy_srv(tagetObj,workspace)
        return response.Success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    try:
        demo()
    except rospy.ROSInterruptException:
        pass

    
