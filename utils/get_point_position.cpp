#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

using namespace std;
using namespace cv;

float fx;
float fy;
float cx;
float cy;
float invfx;
float invfy;
boost::array<double,9> camera_info = {0};
ros::Subscriber camera_info_sub;
cv::Mat depth_f;
cv::Mat RGBImg;

string camera_frame;
string robot_frame = "base_link";
int clicked_x = 0;
int clicked_y = 0;
bool clicked_flag = false; 

void mouse_callback(int event, int x, int y, int flags, void *userdata)
{
  if (event == EVENT_LBUTTONDOWN) {
    clicked_x = x;
    clicked_y = y;
    clicked_flag = true;

  }
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

void Imagecallback(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD)
{
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    camera_frame = msgD->header.frame_id;
    try
      {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
      }
    catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
      {
        cv_ptrD = cv_bridge::toCvShare(msgD);
      }
    catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
    cv::Mat depth;
    cv_ptrD->image.copyTo(depth);
    cv_ptrRGB->image.copyTo(RGBImg);
    cv::cvtColor(RGBImg,RGBImg,CV_BGR2RGB);

    if (depth.type()==2)
      depth.convertTo(depth_f,CV_32FC1, 1.0/1000);
    else if (depth.type()==5)
      depth_f = depth;
    else
      {
        cout<<"unknown depth Mat type"<<endl;
        return;
      }

    imshow("rgb", RGBImg);
    setMouseCallback("rgb", mouse_callback);
    waitKey(5);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_point_position");
  ros::NodeHandle nh_;
  tf::TransformListener listener;
  message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime< sensor_msgs::Image, sensor_msgs::Image> >* sync_input_2_; 
  message_filters::Subscriber<sensor_msgs::Image> rgb_sub;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub; 
  rgb_sub.subscribe(nh_, "/openni_camera/rgb/image_rect_color", 1);
  depth_sub.subscribe(nh_, "/openni_camera/depth_registered/image_raw", 1);
  sync_input_2_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime< sensor_msgs::Image,sensor_msgs::Image> >(10);
  sync_input_2_->connectInput(rgb_sub,depth_sub);
  sync_input_2_->registerCallback(boost::bind(&Imagecallback, _1, _2));    
  camera_info_sub = nh_.subscribe("/openni_camera/depth_registered/camera_info",1, camera_info_cb);
  ros::Rate loop_rate(5);
  while (ros::ok())
    {
      ros::spinOnce();
      if(clicked_flag)
	{
	  geometry_msgs::PointStamped camera_point;
	  camera_point.header.frame_id = camera_frame;    
	  float z =  depth_f.at<float>(clicked_y,clicked_x);
	  if (z>0)
	    {
	      camera_point.point.x = (clicked_x-cx)*z*invfx;
	      camera_point.point.y = (clicked_y-cy)*z*invfy;
	      camera_point.point.z = z;
	      
	      try{
		geometry_msgs::PointStamped base_point;
		listener.transformPoint(robot_frame, camera_point, base_point);
		ROS_INFO("target point's 3D position is: (%.2f, %.2f, %.2f)", base_point.point.x, base_point.point.y, base_point.point.z);
	      }
	      catch(tf::TransformException& ex){
		ROS_ERROR("Received an exception trying to transform a point from \"camera\" to \"base_link\": %s", ex.what());
	      }
	      
	    }
	  else
	    ROS_INFO("target point's z value is <= 0");
	  clicked_flag = false;
	}
      loop_rate.sleep();
    }
  return 0;
}


