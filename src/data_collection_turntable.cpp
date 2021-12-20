#include <ros/package.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/common.h>
#include <tf/transform_listener.h>
#include <multiple_grasping_pose_learning/gettablepose.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/String.h>
#include <sys/stat.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/radius_outlier_removal.h>

using namespace std;
using namespace cv;
typedef pcl::PointXYZRGB PCType;

class aero_goods_grasping_demo
{
public:

  message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime< sensor_msgs::Image, sensor_msgs::Image> >* sync_input_2_; 
  message_filters::Subscriber<sensor_msgs::Image> rgb_sub;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub;

  ros::Subscriber camera_info_sub;
  ros::Subscriber new_object_sub;
  boost::array<double,9> camera_info;
  ros::Publisher my_pointcloud_pub;
  ros::ServiceServer systemService;
  ros::ServiceServer saveImageService;
  ros::ServiceServer saveSingleRgbImageService;

  float fx;
  float fy;
  float cx;
  float cy;
  float invfx;
  float invfy;
  
  tf::TransformListener listener_;
  string robot_base_frame_;
  string robot_camera_frame_;
  pcl::PointCloud<PCType>::Ptr object_cloud;
  pcl::PointCloud<PCType>::Ptr table_cloud;
  cv::Mat current_RGBImg_;
  cv::Mat current_DepthImg_;
  int data_counter_;
  int object_counter_;
  float table_height_;
  std::vector<cv::Point3f> bbox_3d_global_;
  
  std::string base_dir;
  std::string rgb_data_dir;
  std::string mask_data_dir;
  std::string pointcloud_data_dir;
  
  aero_goods_grasping_demo(ros::NodeHandle nh_):camera_info{0}
  {
    //publish and subscribe

    rgb_sub.subscribe(nh_, "/openni_camera/rgb/image_rect_color", 1);
    depth_sub.subscribe(nh_, "/openni_camera/depth_registered/hw_registered/image_rect_raw", 1);
    sync_input_2_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime< sensor_msgs::Image,sensor_msgs::Image> >(10);
    sync_input_2_->connectInput(rgb_sub,depth_sub);
    sync_input_2_->registerCallback(boost::bind(&aero_goods_grasping_demo::callback, this, _1, _2));
    
    camera_info_sub = nh_.subscribe("/openni_camera/depth_registered/camera_info",1, &aero_goods_grasping_demo::camera_info_cb,this);
    my_pointcloud_pub = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("myoutput", 1);

    new_object_sub = nh_.subscribe("/aero_data_collection",1, &aero_goods_grasping_demo::new_object_cb,this);

    //services
    systemService = nh_.advertiseService("/aero_goods_grasping_demo", &aero_goods_grasping_demo::srvCb, this);
    saveImageService = nh_.advertiseService("/aero_new_image_collection", &aero_goods_grasping_demo::saveImgSrvCb, this);
    saveSingleRgbImageService = nh_.advertiseService("/aero_new_single_rgb_image_collection", &aero_goods_grasping_demo::saveSingleRgbImgSrvCb, this);

    robot_base_frame_ = "/base_link";
    data_counter_ = 0;
    int start_obj_num = 1; 
    if (nh_.hasParam("start_obj_num"))
      {
	nh_.getParam("start_obj_num", start_obj_num);
      }
    
    object_counter_ = start_obj_num;
    ROS_INFO("start collecting object from num %i", object_counter_);
    base_dir = ros::package::getPath("multiple_grasping_pose_learning");
    string rgb_data_dir_prefix = base_dir + "/dataset/rgb";
    string mask_data_dir_prefix = base_dir + "/dataset/mask";
    string pointcloud_data_dir_prefix  = base_dir + "/dataset/pointcloud";
    char object_num[10];	
    sprintf(object_num,"/%02d/", object_counter_);
    rgb_data_dir = rgb_data_dir_prefix + object_num;
    mask_data_dir = mask_data_dir_prefix + object_num;
    pointcloud_data_dir = pointcloud_data_dir_prefix + object_num;
    if(!IsdirExist(rgb_data_dir.c_str()))
      mkdir(rgb_data_dir.c_str(),0777);
    if(!IsdirExist(mask_data_dir.c_str()))
      mkdir(mask_data_dir.c_str(),0777);
    if(!IsdirExist(pointcloud_data_dir.c_str()))
      mkdir(pointcloud_data_dir.c_str(),0777);

  }
  
  int IsdirExist(const char *path)
{
    struct stat info;
    if(stat( path, &info ) != 0)
        return 0;
    else if(info.st_mode & S_IFDIR)
        return 1;
    else
        return 0;
}


  void new_object_cb(const std_msgs::String &message)
  {
    if(message.data=="add new")
      {

	object_counter_++;
	ROS_INFO("ready for the new object %i",object_counter_);
	data_counter_ = 0;
	char object_num[10];	
	sprintf(object_num,"/%02d/", object_counter_);
	string mask_data_dir_prefix = mask_data_dir.substr(0,mask_data_dir.length()-3);
	mask_data_dir = mask_data_dir_prefix + object_num;
	string rgb_data_dir_prefix = rgb_data_dir.substr(0,rgb_data_dir.length()-3);
	rgb_data_dir = rgb_data_dir_prefix + object_num;
	string pointcloud_data_dir_prefix = pointcloud_data_dir.substr(0,pointcloud_data_dir.length()-3);
	pointcloud_data_dir = pointcloud_data_dir_prefix + object_num;

	
	if(!IsdirExist(rgb_data_dir.c_str()))
	  mkdir(rgb_data_dir.c_str(),0777);
	if(!IsdirExist(mask_data_dir.c_str()))
	  mkdir(mask_data_dir.c_str(),0777);
	if(!IsdirExist(pointcloud_data_dir.c_str()))
	  mkdir(pointcloud_data_dir.c_str(),0777);

      }
    else if(message.data=="do again")
      {
	ROS_INFO("collect data again, please delete old data");
	data_counter_ = 0;
      }
    
    else
      ROS_INFO("Undefined command %s ", message.data.c_str());
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

  void callback(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD)
  {

    cv_bridge::CvImageConstPtr cv_ptrRGB;
    cv_bridge::CvImage cv_img;
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
    cv::Mat depth_f;

    if (depth.type()==2)
      depth.convertTo(depth_f,CV_32FC1, 1.0/1000);
    else if (depth.type()==5)
      depth_f = depth;
    else
      {
        cout<<"unknown depth Mat type"<<endl;
        return;
      }

    cv::Mat RGBImg;
    cv_ptrRGB->image.copyTo(RGBImg);
    cv::cvtColor(RGBImg,RGBImg,CV_BGR2RGB);
      
    current_RGBImg_ = RGBImg;
    current_DepthImg_ = depth_f;
    robot_camera_frame_ = cv_ptrD->header.frame_id;
      
  }

  geometry_msgs::PointStamped point2d_to_point3d(cv::Point p)
  {
    geometry_msgs::PointStamped point3d;
    geometry_msgs::PointStamped point3d_trans;
    float z = current_DepthImg_.at<float>(p.y,p.x);    
    if(z>0)
      {
	point3d.point.x = (p.x-cx)*z*invfx;
	point3d.point.y = (p.y-cy)*z*invfy;
	point3d.point.z = z;
      }
    else
      {
	ROS_INFO("z value in target pixel depth <= 0; use a neighbor pixel in y axis instead");
	bool found_valid_z_flag = false;
	int bias = 0;
	if(p.y > current_DepthImg_.size().height/2)
	  {
	    for(int i=0; i<30; i++)
	      {
		if(current_DepthImg_.at<float>(p.y-i,p.x)>0)
		  {
		    bias = i;
		    found_valid_z_flag = true;
		    break;		    
		  }
	      }
	    float new_z = current_DepthImg_.at<float>(p.y-bias,p.x);
	    point3d.point.x = (p.x-cx)*new_z*invfx;
	    point3d.point.y = (p.y-bias-cy)*new_z*invfy;
	    point3d.point.z = new_z;
	  }
	else
	  {
	    for(int i=0; i<30; i++)
	      {
		if(current_DepthImg_.at<float>(p.y+i,p.x)>0)
		  {
		    bias = i;
		    found_valid_z_flag = true;
		    break;	    
		  }
	      }
	    float new_z = current_DepthImg_.at<float>(p.y+bias,p.x);
	    point3d.point.x = (p.x-cx)*new_z*invfx;
	    point3d.point.y = (p.y+bias-cy)*new_z*invfy;
	    point3d.point.z = new_z;	   
	  }
	if (!found_valid_z_flag)
	  ROS_FATAL("fail to transform point, nan depth occurred");
      }
    point3d.header.frame_id = robot_camera_frame_;
    try{
      listener_.waitForTransform(robot_base_frame_, robot_camera_frame_, ros::Time(0), ros::Duration(3.0));
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }    
    listener_.transformPoint(robot_base_frame_, point3d, point3d_trans);
    return point3d_trans;
  }
  
  cv::Mat get_target_obj_mask(std::vector<cv::Point3f> bbox_3d)
  {

    //get and transform points within 2d bounding box
    pcl::PointCloud<PCType>::Ptr bbox_cloud(new pcl::PointCloud<PCType>());
    for(int i = 0; i < current_DepthImg_.cols; i++) {
      for(int j = 0; j < current_DepthImg_.rows; j++) {
	float z =  current_DepthImg_.at<float>(j,i);
	if(z>0)
	  {
	    PCType point;
	    point.x = (i-cx)*z*invfx;
	    point.y = (j-cy)*z*invfy;
	    point.z = z;
	    point.r = current_RGBImg_.at<Vec3b>(j,i)[0];
	    point.g = current_RGBImg_.at<Vec3b>(j,i)[1];
	    point.b = current_RGBImg_.at<Vec3b>(j,i)[2];
	    bbox_cloud->points.push_back(point);
	  }    
      }
    }
    
    bbox_cloud->header.frame_id = robot_camera_frame_;

    try{
      listener_.waitForTransform(robot_base_frame_, robot_camera_frame_, ros::Time(0), ros::Duration(3.0));
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }

    pcl::PointCloud<PCType>::Ptr cloud_trans(new pcl::PointCloud<PCType>());
    pcl_ros::transformPointCloud(robot_base_frame_, *bbox_cloud, *cloud_trans, listener_);

    pcl::PointCloud <PCType>::Ptr cloud_cropped (new pcl::PointCloud <PCType>);
    pcl::CropBox<PCType> boxFilter;
    
    boxFilter.setMin(Eigen::Vector4f(bbox_3d[0].x, bbox_3d[0].y, bbox_3d[0].z, 1.0));
    boxFilter.setMax(Eigen::Vector4f(bbox_3d[1].x, bbox_3d[1].y, bbox_3d[0].z+0.6, 1.0));
    
    boxFilter.setInputCloud(cloud_trans);
    boxFilter.filter(*cloud_cropped);

    cv::Mat mask = cv::Mat::zeros(current_RGBImg_.size(),current_RGBImg_.type());
    
    if(cloud_cropped->points.size()>0)
      {
	//remove outliers
	pcl::RadiusOutlierRemoval<PCType> outrem;
	outrem.setInputCloud(cloud_cropped);
	outrem.setRadiusSearch(0.8);
	outrem.setMinNeighborsInRadius (2);
	outrem.setKeepOrganized(true);
	outrem.filter (*cloud_cropped);
	std::vector<int> nan_indices;
	pcl::removeNaNFromPointCloud(*cloud_cropped, *cloud_cropped, nan_indices);

	object_cloud = cloud_cropped;
	object_cloud->header.frame_id = robot_base_frame_;
	//save point cloud
	char suffix[30];
	sprintf(suffix,"frame%02d.ply", data_counter_);
	std::string pointcloud_path = pointcloud_data_dir + suffix;
	pcl::PLYWriter ply_saver;
	// move the point cloud to coordinate origin before saving
	pcl::PointCloud<PCType>::Ptr cloud_centered = move_point_cloud_to_origin(cloud_cropped); 
	ply_saver.write(pointcloud_path,*cloud_centered);
	ROS_INFO("pointcloud is saved");
	//back project pointcloud to 2d mask
	pcl::PointCloud<PCType>::Ptr cloud_trans_back(new pcl::PointCloud<PCType>());
	pcl_ros::transformPointCloud(robot_camera_frame_, *object_cloud, *cloud_trans_back, listener_);
	
	for(int i=0; i<cloud_trans_back->points.size(); i++)
	  {
	    PCType point = cloud_trans_back->points[i];
	    float z = point.z;
	    int x = point.x/invfx/z + cx;
	    int y = point.y/invfy/z + cy;
	    mask.at<Vec3b>(y,x) = current_RGBImg_.at<Vec3b>(y,x);
	  }
	
	//publish point cloud 
	sensor_msgs::PointCloud2 pc2;
	pcl::PCLPointCloud2::Ptr pcl_pc_2(new pcl::PCLPointCloud2());
	pcl::toPCLPointCloud2 (*cloud_cropped, *pcl_pc_2);
	pcl_conversions::fromPCL( *pcl_pc_2, pc2 );
	pc2.header.stamp = ros::Time::now();
	pc2.header.frame_id = robot_base_frame_;
	my_pointcloud_pub.publish(pc2);	
      }

    return mask;

  }

  std::vector<cv::Point3f> get_3d_bbox(std::vector<cv::Point> bbox_2d)
  {
    //compute 3d bounding box boundary
    float min_x = 100;
    float max_x = -100;
    float min_y = 100;
    float max_y = -100;
    float min_z = -100;
    for(int i=0; i< bbox_2d.size(); i++)
      {
	geometry_msgs::PointStamped point3d = point2d_to_point3d(bbox_2d[i]);
	if(point3d.point.x < min_x)
	  min_x = point3d.point.x;
	if(point3d.point.y < min_y)
	  min_y = point3d.point.y;
	if(point3d.point.x > max_x)
	  max_x = point3d.point.x;
	if(point3d.point.y > max_y)
	  max_y = point3d.point.y;
	if(point3d.point.z > min_z) //take larger z to remove tabletop
	  min_z = point3d.point.z + 0.05;
      }

    //get and transform points within 2d bounding box
    pcl::PointCloud<PCType>::Ptr bbox_cloud(new pcl::PointCloud<PCType>());
    for(int i = 0; i < current_DepthImg_.cols; i++) {
      for(int j = 0; j < current_DepthImg_.rows; j++) {
	float z =  current_DepthImg_.at<float>(j,i);
	if(z>0)
	  {
	    PCType point;
	    point.x = (i-cx)*z*invfx;
	    point.y = (j-cy)*z*invfy;
	    point.z = z;
	    point.r = current_RGBImg_.at<Vec3b>(j,i)[0];
	    point.g = current_RGBImg_.at<Vec3b>(j,i)[1];
	    point.b = current_RGBImg_.at<Vec3b>(j,i)[2];
	    bbox_cloud->points.push_back(point);
	  }    
      }
    }    
    bbox_cloud->header.frame_id = robot_camera_frame_;
    try{
      listener_.waitForTransform(robot_base_frame_, robot_camera_frame_, ros::Time(0), ros::Duration(3.0));
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    pcl::PointCloud<PCType>::Ptr cloud_trans(new pcl::PointCloud<PCType>());
    pcl_ros::transformPointCloud(robot_base_frame_, *bbox_cloud, *cloud_trans, listener_);
    pcl::PointCloud <PCType>::Ptr cloud_cropped (new pcl::PointCloud <PCType>);
    pcl::CropBox<PCType> boxFilter;   
    boxFilter.setMin(Eigen::Vector4f(min_x, min_y, min_z, 1.0));
    boxFilter.setMax(Eigen::Vector4f(max_x, max_y, min_z+0.5, 1.0));
    boxFilter.setInputCloud(cloud_trans);
    boxFilter.filter(*cloud_cropped);
    std::vector<cv::Point3f> bbox_3d;
    bbox_3d.push_back(cv::Point3f(-1,-1,-1));
    if(cloud_cropped->points.size()>0)
      {
	std::vector<int> nan_indices;
	pcl::removeNaNFromPointCloud(*cloud_cropped, *cloud_cropped, nan_indices);

	table_cloud = cloud_cropped;
	table_cloud->header.frame_id = robot_base_frame_;

	// find_table_height
	pcl::PointXYZ minpt;
	pcl::PointXYZ maxpt;
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cloud_cropped, *temp_cloud_xyz);//xyzrgb to xyz for the getminmax function
	pcl::getMinMax3D(*temp_cloud_xyz,minpt,maxpt);

	bbox_3d[0] = cv::Point3f(min_x,min_y,maxpt.z);//bbox should be higher than the table
	bbox_3d.push_back(cv::Point3f(max_x,max_y,maxpt.z+0.6));//we do not care max_z
      }
	return bbox_3d;
  }

  pcl::PointCloud<PCType>::Ptr move_point_cloud_to_origin(pcl::PointCloud<PCType>::Ptr cloud)
  //move the point cloud centroid to (0,0,0)
  {
    pcl::PointCloud<PCType>::Ptr cloud_trans(new pcl::PointCloud<PCType>());
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cloud, pcaCentroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
    eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
    eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));
    Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();
    tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();   //R. 
    tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) *(pcaCentroid.head<3>());//  -R*t                    
    tm_inv = tm.inverse();
    pcl::PointCloud<PCType>::Ptr transformedCloud(new pcl::PointCloud<PCType>);
    pcl::transformPointCloud(*cloud, *transformedCloud, tm);
    PCType min_p1, max_p1;
    Eigen::Vector3f c1, c;
    pcl::getMinMax3D(*transformedCloud, min_p1, max_p1);
    c1 = 0.5f*(min_p1.getVector3fMap() + max_p1.getVector3fMap());   
    Eigen::Affine3f tm_inv_aff(tm_inv);
    pcl::transformPoint(c1, c, tm_inv_aff);   
    Eigen::Vector3f whd, whd1;
    whd1 = max_p1.getVector3fMap() - min_p1.getVector3fMap();
    whd = whd1;
    float sc1 = (whd1(0) + whd1(1) + whd1(2)) / 3;
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << -pcaCentroid(0), -pcaCentroid(1), -pcaCentroid(2);
    pcl::transformPointCloud (*cloud, *cloud_trans, transform);
    return cloud_trans;
  }
    
  bool srvCb(multiple_grasping_pose_learning::gettablepose::Request& req, multiple_grasping_pose_learning::gettablepose::Response& res)
  {

    string command = req.Command;
    ROS_INFO("got request %s ", command.c_str());
    
    //get bounding box boundaries
    vector< int > markerIds;
    vector< vector<Point2f> > markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::detectMarkers(current_RGBImg_, dictionary, markerCorners, markerIds);
    
    cout<<"detected number of markers: "<<markerIds.size()<<endl;

    int min_x_2d = 5000; //a very large number
    int max_x_2d = 0;
    int min_y_2d = 5000;
    int max_y_2d = 0;
    //compute bounding box boundary in the image
    if (markerIds.size() == 2)
      {
	for(int i=0; i< markerIds.size(); i++)	  
	  for(int j=0; j< markerCorners[i].size(); j++)
	    {
	      if(markerCorners[i][j].x < min_x_2d)
		min_x_2d = markerCorners[i][j].x;
	      if(markerCorners[i][j].y < min_y_2d)
		min_y_2d = markerCorners[i][j].y;
	      if(markerCorners[i][j].x > max_x_2d)
		max_x_2d = markerCorners[i][j].x;
	      if(markerCorners[i][j].y > max_y_2d)
		max_y_2d = markerCorners[i][j].y;
	    }
	std::vector<cv::Point> bbox_2d;
	bbox_2d.push_back(cv::Point(min_x_2d,min_y_2d));
	bbox_2d.push_back(cv::Point(max_x_2d,max_y_2d));
	std::vector<cv::Point3f> bbox_3d = get_3d_bbox(bbox_2d);
	if(bbox_3d[0].x == -1 && bbox_3d[0].y == -1 && bbox_3d[0].z == -1)
	  return false;
	bbox_3d_global_ = bbox_3d;

      }

    else
      {
	ROS_WARN("fail to detect two markers, please check the image");
	return false;
      }
    std::vector<float> result;

    Eigen::Vector4f objCentroid;
    pcl::compute3DCentroid(*table_cloud,objCentroid);

    for(int i=0; i<3; i++)
      {
	result.push_back(objCentroid(i)*1000); //m to mm in euslisp
      }    
    res.Pose = result;
    return true;      
  }

  bool saveImgSrvCb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    if(bbox_3d_global_.empty())
      {
	bbox_3d_global_.push_back(cv::Point3f(-1,-1,-1)); //avoid segment fault
	return false;
      }
    cv::Mat obj_mask = get_target_obj_mask(bbox_3d_global_);
    char suffix[30];
    //char maskfilename[30];
    sprintf(suffix,"frame%02d.jpg", data_counter_);
    //sprintf(maskfilename,"framemask%02d.jpg", data_counter_);
    std::string rgb_path = rgb_data_dir + suffix;
    std::string mask_path = mask_data_dir + suffix;
    cv::imwrite(rgb_path,current_RGBImg_);
    cv::imwrite(mask_path,obj_mask);
    data_counter_++;
    ROS_INFO("images are saved");
    return true;
  }

  bool saveSingleRgbImgSrvCb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    char suffix[30];
    //char maskfilename[30];
    sprintf(suffix,"frame%02d.jpg", data_counter_);
    //sprintf(maskfilename,"framemask%02d.jpg", data_counter_);
    std::string rgb_path = rgb_data_dir + suffix;
    cv::imwrite(rgb_path,current_RGBImg_);
    data_counter_++;
    ROS_INFO("images are saved");
    return true;
  }
  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aero_goods_grasping_demo");

  ros::NodeHandle nh_ = ros::NodeHandle("~");
  aero_goods_grasping_demo demo(nh_);
  ros:: Rate r(10);
  ros::spin();
  return 0;
}
