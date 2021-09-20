#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <multiple_grasping_pose_learning/graspanobject.h>
#include <multiple_grasping_pose_learning/objectdetect.h>
#include <multiple_grasping_pose_learning/grasppredict.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/crop_box.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <pcl/io/ply_io.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;
using namespace cv;
typedef pcl::PointXYZ PCType;

class aero_goods_grasping_deploy
{
public:
  ros::NodeHandle nh_;
  message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime< sensor_msgs::Image, sensor_msgs::Image> >* sync_input_2_; 
  message_filters::Subscriber<sensor_msgs::Image> rgb_sub;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub;
  sensor_msgs::Image RGBImg_;
  sensor_msgs::Image DepthImg_;
  ros::Subscriber camera_info_sub;
  ros::Publisher pointcloud_pub;
  boost::array<double,9> camera_info;
  multiple_grasping_pose_learning::objectdetect object_detect_srv;
  ros::ServiceClient object_detection_client;

  multiple_grasping_pose_learning::grasppredict grasp_predict_srv;
  ros::ServiceClient grasp_prediction_client;
  
  float fx;
  float fy;
  float cx;
  float cy;
  float invfx;
  float invfy;
  tf::TransformListener listener;
  string tf_reference_frame_;
  string camera_frame_;
  ros::ServiceServer systemService;
  
  aero_goods_grasping_deploy():camera_info{0}
  {
    rgb_sub.subscribe(nh_, "/openni_camera/rgb/image_rect_color", 1);
    depth_sub.subscribe(nh_, "/openni_camera/depth_registered/hw_registered/image_rect_raw", 1);
    sync_input_2_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime< sensor_msgs::Image,sensor_msgs::Image> >(10);
    sync_input_2_->connectInput(rgb_sub,depth_sub);
    sync_input_2_->registerCallback(boost::bind(&aero_goods_grasping_deploy::image_callback, this, _1, _2));
    
    camera_info_sub = nh_.subscribe("/openni_camera/depth_registered/camera_info",1, &aero_goods_grasping_deploy::camera_info_cb,this);

    object_detection_client = nh_.serviceClient<multiple_grasping_pose_learning::objectdetect>("aero_goods_demo_object_detect");
    grasp_prediction_client= nh_.serviceClient<multiple_grasping_pose_learning::grasppredict>("aero_goods_demo_grasp_predict");
    pointcloud_pub = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("aero_goods_grasping_pointcloud_visualize", 1);

    systemService = nh_.advertiseService("aero_goods_grasping_deploy", &aero_goods_grasping_deploy::srv_callback, this);
    tf_reference_frame_ = "/base_link";
  }

  
  void camera_info_cb(const sensor_msgs::CameraInfoPtr& camInfo)
  {
    camera_info = camInfo->K;
    fx = camera_info[0];
    fy = camera_info[4];
    cx = camera_info[2];
    cy = camera_info[5];
    invfx = 1.0f/fx;
    invfy = 1.0f/fy;
    camera_info_sub.shutdown();
  }

  void publish_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    //add fake color
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud, *colored_cloud);
    for(int i=0; i<colored_cloud->points.size(); i++)
      {
	colored_cloud->points[i].r = 0;
	colored_cloud->points[i].g = 255;
	colored_cloud->points[i].b = 0;
      }
    sensor_msgs::PointCloud2 pc2;
    pcl::PCLPointCloud2::Ptr pcl_pc_2(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2 (*colored_cloud, *pcl_pc_2);
    pcl_conversions::fromPCL( *pcl_pc_2, pc2 );
    pc2.header.stamp = ros::Time::now();
    pc2.header.frame_id = tf_reference_frame_;
    pointcloud_pub.publish(pc2);
  }

  
  void image_callback(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD)
  {
    RGBImg_ = *msgRGB;
    DepthImg_ = *msgD;
    camera_frame_ = msgD->header.frame_id;
  }

  pcl::PointCloud<PCType>::Ptr image_roi_to_pointcloud(std::vector<int> candiate_bbox)
  {
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
      {
        cv_ptrRGB = cv_bridge::toCvCopy(RGBImg_);
      }
    catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
      }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
      {
        cv_ptrD = cv_bridge::toCvCopy(DepthImg_);
      }
    catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
      }

    cv::Mat depth;
    cv_ptrD->image.copyTo(depth);
    cv::Mat depth_f;
    
    cv::Mat RGBImg;
    cv_ptrRGB->image.copyTo(RGBImg);
    cv::cvtColor(RGBImg,RGBImg,CV_BGR2RGB);                            
    if (depth.type()==2)
      depth.convertTo(depth_f,CV_32FC1, 1.0/1000);
    else if (depth.type()==5)
      depth_f = depth;
    else
      {
        cout<<"unknown depth Mat type"<<endl;
      }

    if(candiate_bbox[0] < 0)
      candiate_bbox[0]=0;
    if(candiate_bbox[1] < 0)
      candiate_bbox[1]=0;
    if(candiate_bbox[0] + candiate_bbox[2] > RGBImg.cols)
      candiate_bbox[2]= RGBImg.cols - candiate_bbox[0];
    if(candiate_bbox[1] + candiate_bbox[3] > RGBImg.rows)
      candiate_bbox[3]= RGBImg.rows - candiate_bbox[1];

    cv::Rect roi_rect(candiate_bbox[0],candiate_bbox[1],candiate_bbox[2],candiate_bbox[3]);

    cv::Mat RGB_ROI = RGBImg(roi_rect);
    cv::Mat depth_ROI = depth_f(roi_rect);

    pcl::PointCloud<PCType>::Ptr cloud(new pcl::PointCloud<PCType>());
    for(int i = 0; i < depth_ROI.cols; i++)
      {
	for(int j = 0; j < depth_ROI.rows; j++)
	  {
	    float z =  depth_ROI.at<float>(j,i);
	    if(z>0)
	      {
		PCType point;
		point.x = (i+candiate_bbox[0]-cx)*z*invfx;
		point.y = (j+candiate_bbox[1]-cy)*z*invfy;
		point.z = z;
		cloud->points.push_back(point);
	      }
	  }
      }
    //add header for transformation                                   
    cloud->header.frame_id = camera_frame_;
    //transform point cloud
    try
      {
	listener.waitForTransform(tf_reference_frame_, camera_frame_ , ros::Time(0), ros::Duration(3.0));
      }
    catch (tf::TransformException ex){ROS_ERROR("%s",ex.what());}
    pcl::PointCloud<PCType>::Ptr cloud_trans(new pcl::PointCloud<PCType>());
    pcl_ros::transformPointCloud(tf_reference_frame_, *cloud, *cloud_trans, listener);
    return cloud_trans;
  }

  pcl::PointCloud<PCType>::Ptr background_removal(pcl::PointCloud<PCType>::Ptr bbox_cloud, std::vector<float> workspace)
  {    
    pcl::PointCloud <PCType>::Ptr cloud_cropped (new pcl::PointCloud <PCType>);
    pcl::CropBox<PCType> boxFilter;
    
    boxFilter.setMin(Eigen::Vector4f(workspace[0], workspace[1], workspace[2], 1.0));
    boxFilter.setMax(Eigen::Vector4f(workspace[3], workspace[4], workspace[5], 1.0));
    boxFilter.setInputCloud(bbox_cloud);
    boxFilter.filter(*cloud_cropped);
    std::vector<int> nan_indices;
    pcl::removeNaNFromPointCloud(*cloud_cropped, *cloud_cropped, nan_indices);
    return cloud_cropped;
  }

  std::vector<float> pointCloud_to_FlatPointCloud(pcl::PointCloud<PCType>::Ptr pointCloud)
  {
        std::vector<float> FlatPointCloud;
        for (int i=0; i<pointCloud->points.size(); i++)
          {
	    FlatPointCloud.push_back(pointCloud->points[i].x);
	    FlatPointCloud.push_back(pointCloud->points[i].y);
	    FlatPointCloud.push_back(pointCloud->points[i].z);
          }
        return FlatPointCloud;
  }
  
  std::vector<float>  grasp_target_object(int obj_label, std::vector<float> workspace)
  {
    std::vector<float> final_pose;
    object_detect_srv.request.Image = RGBImg_;
    //find out the target object in the rgb image
    if(object_detection_client.call(object_detect_srv))
      {
	ROS_INFO("call object_detection_client successfully");
	std::vector<int32_t> ObjLabels = object_detect_srv.response.ObjLabels;
	std::vector<float> Scores = object_detect_srv.response.Scores;
	std::vector<int32_t> Bboxes = object_detect_srv.response.Bboxes;
	ROS_INFO("Number of objects detected: %d ", int(ObjLabels.size()));
	int num_candidates_counter = 0;
	for (int i=0; i<ObjLabels.size(); i++)
	  {
	    ROS_INFO("found object: %d with score %.3f", ObjLabels[i], Scores[i]);
	  }
	for (int i=0; i<ObjLabels.size(); i++)
	  {
	    if(ObjLabels[i]==obj_label)
	      {
		num_candidates_counter++;
		//get object point cloud within boundingbox and robot workspace
		std::vector<int> candidate_bbox;
		candidate_bbox.push_back(Bboxes[i*4]); //x0
		candidate_bbox.push_back(Bboxes[i*4+1]); //y0 
		candidate_bbox.push_back(Bboxes[i*4+2] - Bboxes[i*4]); //x1 - x0 
		candidate_bbox.push_back(Bboxes[i*4+3] - Bboxes[i*4+1]); //y1 - y0
		pcl::PointCloud<PCType>::Ptr bbox_cloud = image_roi_to_pointcloud(candidate_bbox);
		pcl::PointCloud<PCType>::Ptr object_cloud = background_removal(bbox_cloud,workspace);
		publish_pointcloud(object_cloud);
		//call grasp_predict_srv to predict grasping pose for the object point cloud
		std::vector<float> TargetFlatPointCloud = pointCloud_to_FlatPointCloud(object_cloud);
		grasp_predict_srv.request.TargetObjectFlatPointCloud = TargetFlatPointCloud;
		ROS_INFO("call grasp pose prediction server");
		if(grasp_prediction_client.call(grasp_predict_srv))
		  {		   
		    ROS_INFO("call grasp pose prediction server successfully");
		    std::vector<float> grasp_pose = grasp_predict_srv.response.GraspingPose;

		    final_pose = grasp_pose;
		  }
	      }
	  }
      }//end of if(object_detection_client.call(object_detect_srv))
    return final_pose;
  }
  
  bool srv_callback(multiple_grasping_pose_learning::graspanobject::Request& req, multiple_grasping_pose_learning::graspanobject::Response& res)
  {
    string targetObj = req.TargetObj;
    std::vector<float> workspace = req.WorkSpace;
    if (workspace.size() != 6)
      {
	ROS_INFO("the size of workspace vector must be 6");
	return false;
      }
    int obj_label = std::stoi(targetObj);
    ROS_INFO("received command: grasp object: %s ", targetObj.c_str());
    std::vector<float> final_pose = grasp_target_object(obj_label,workspace);
    if (!final_pose.empty())
      {
	tf2::Quaternion q(final_pose[4],final_pose[5],final_pose[6],final_pose[3]);
	tf2::Quaternion q2(final_pose[11],final_pose[12],final_pose[13],final_pose[10]);
	q.normalize();
	q2.normalize();
	//update final_pose with normalized q and q2
	final_pose[3] = q.w();
	final_pose[4] = q.x();
	final_pose[5] = q.y();
	final_pose[6] = q.z();

	final_pose[10] = q.w();
	final_pose[11] = q.x();
	final_pose[12] = q.y();
	final_pose[13] = q.z();	
	res.Pose = final_pose;
	return true;
      }
    res.Pose = final_pose; //return an empty final_pose
    return true;
  }
};
int main(int argc, char** argv)
{
  ros::init(argc, argv, "aero_goods_grasping_deploy");
  aero_goods_grasping_deploy deploy;
  ros::spin();

  return 0;
}
