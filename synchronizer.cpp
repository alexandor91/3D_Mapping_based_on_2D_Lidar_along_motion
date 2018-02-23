
/*****************************************************************************/
/* File:        synchronize.cpp                                              */
/* Description: synchronoze the trajectory topic and point cloud topic       */
/*              trajectory topic provides the heading direction and position */
/*              setpoints for each degree of freedom                         */
/* Email:       alexander.kang@tum.de                                        */
/* Author:      alexander.kang                                               */
/* Date:        12-2017                                                      */
/*****************************************************************************/
#include<iostream>
#include<algorithm>
#include<fstream>
#include<math.h>

#include<chrono>
#include<string>
#include<math.h>
//message filters
#include<tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//ros message type
#include "nav_msgs/Path.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <pcl_ros/point_cloud.h>

//pcl library
#include <pcl/io/pcd_io.h> 
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>


#define PI 3.14159265

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace std;
//path for data storage
string pcd_path = "/home/robond/catkin_ws/src/fusion_octomap/point_cloud/motion_raw_pcl.pcd";       //final output pcd file
string pcd_path1 = "/home/robond/catkin_ws/src/fusion_octomap/point_cloud/motion_downsample_pcl.pcd";      //output pcd file after downsampling filter
string pcd_path2 = "/home/robond/catkin_ws/src/fusion_octomap/point_cloud/motion_passthrough_pcl.pcd";    //output pcd file after passthrough filter

//ofstream Fusion_scan_File ("Fusion_scan.csv");
float height_offset = 0.94; //offset for desktop height and additional height
float laser_sweep_xoffset = -0.155;  //big box:-0.155m, small box:-0.15m, rplidar put at left of sweep
float laser_sweep_yoffset = -0.085;   //big box:-0.085m, small box:-0.10m, rplidar put behind the sweep
float initial_heading_imu = 220 * PI /180;   //initial orientation
float initial_heading_slam = 0.0;
float initial_xpose = 2.0;         //initial x position of Rplidar  
float initial_ypose = 0.0;         //initial y position of Rplidar  
//registration parameters
float heading_thre = 3.0;  //degree
float x_thre = 0.01;      //only for straight movement
float y_thre = 0.01;      //only for straight movement
float dist_thre = 0.025;  //euclidean distance
float percent_thre = 0.9; //proportion of number of points fitting into plane model to the total number in one scan

float steady_x = 0.0;
float steady_y= 0.0;
float steady_yaw = 0.0;     //last steady position and orientation
float last_x = 0.0;
float last_y= 0.0;
float last_yaw_degree = 0.0;
float last_yaw = 0.0;     //last time position and orientation
size_t size_counters = 0;
size_t valid_measurements = 10;  //the valid measurements from a scan should not be too little
bool initial_pose = false;    //flag set after the first registration at start position

pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud_fused(new pcl::PointCloud<pcl::PointXYZ> );    //global pointer to register multiple pointcloud icrementally     
size_t Points_size;   //size of all collected pointcloud so far

class MatchPointcloud{
public:
       MatchPointcloud(){}
       void Callback(const sensor_msgs::PointCloud2ConstPtr& Pcl_msg, const nav_msgs::PathConstPtr& Path_msg);
       sensor_msgs::PointCloud2 Fusion_msg;

       //ros node
       ros::NodeHandle nh;
       //ros publisher
       ros::Publisher fusion_pub = nh.advertise<sensor_msgs::PointCloud2>("/Fusion/map",1); // fused pointcloud to be publilshed
};  //match class

void MatchPointcloud::Callback(const sensor_msgs::PointCloud2ConstPtr &Pcl_msg, const nav_msgs::PathConstPtr &Path_msg){

  cout<<"start match:"<<endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud (new pcl::PointCloud<pcl::PointXYZ> );   //incoming cloud point from a scan

  pcl::PointXYZ point; //one point

  pcl::fromROSMsg (*Pcl_msg, *xyz_cloud);  //convert back to ros message type

  double roll,pitch,yaw;
  double delta_yaw;
  size_t counter = 0;
  //initial quaternion variable
  tf::Quaternion quater;
  int index = Path_msg -> poses.size();
  float current_xposition = - Path_msg -> poses[index -1].pose.position.x;
  float current_yposition = Path_msg -> poses[index -1].pose.position.y;
  //convert quaternion msg to quaternion
  tf::quaternionMsgToTF(Path_msg -> poses[index -1].pose.orientation, quater);
  tf::Matrix3x3(quater).getRPY(roll, pitch, yaw);
  float yaw_degree = yaw * 180.0/PI; //to degree
  cout<< "yaw degree" << yaw_degree <<endl;
  cout<< "x pose" << (- Path_msg -> poses[index -1].pose.position.x) <<endl;
  cout<< "y pose" << (Path_msg -> poses[index -1].pose.position.y) <<endl;

  if (xyz_cloud->points.size () >= valid_measurements){
  counter = xyz_cloud->points.size ();
  cout << "cloud points per scan" << counter << endl;
   
  if (!initial_pose)
  {
  //TODO initial position 
   cout << "register start" << endl;
   initial_heading_slam = yaw;
   initial_pose = true;
   if (yaw > 0.0)
      yaw = -yaw;
   for (size_t point_i = 0; point_i < counter; ++point_i)
    {
     point.x = xyz_cloud->points[point_i].x * cos(yaw) + xyz_cloud->points[point_i].y * sin(yaw);
     point.y = xyz_cloud->points[point_i].x * (-sin(yaw)) + xyz_cloud->points[point_i].y * cos(yaw);
     point.x += current_xposition + laser_sweep_xoffset + initial_xpose;
     point.y += current_yposition + laser_sweep_yoffset + initial_ypose;
     point.z = xyz_cloud->points[point_i].z + height_offset;
     xyz_cloud_fused->points.push_back(point);
     Points_size += 1;
    } 
   }

  else if ((initial_pose) && ((sqrt(pow(current_xposition - last_x, 2) + pow(current_yposition -last_y, 2)) >= dist_thre) || (current_xposition - last_x >= x_thre) || (current_yposition - last_y >= y_thre) || ((yaw - last_yaw)*180.0/PI >= heading_thre))) //the propotion of the points fitting into the model should reach an
  {
  //TODO after movement rotation or translational movement, update new lcoation
  cout << "register after start" << endl;
  if (yaw > 0.0)
      yaw = yaw - 2 * PI;
  yaw = yaw - initial_heading_slam;  //counter-clockwise in rivz
  for (size_t point_i = 0; point_i < counter; ++point_i)
    {
     point.x = xyz_cloud->points[point_i].x * cos(yaw) + xyz_cloud->points[point_i].y * sin(yaw);
     point.y = xyz_cloud->points[point_i].x * (-sin(yaw)) + xyz_cloud->points[point_i].y * cos(yaw);
     point.x += current_xposition + laser_sweep_xoffset + initial_xpose;
     point.y += current_yposition + laser_sweep_yoffset + initial_ypose;
     point.z = xyz_cloud->points[point_i].z + height_offset;
     xyz_cloud_fused->points.push_back(point);
     Points_size += 1;
    }
  steady_x = current_xposition;
  steady_y = current_yposition;
  steady_yaw = yaw;
  }
  else{
  //TODO append the points to the previous location
  cout << "register not changed" << endl;
  for (size_t point_i = 0; point_i < counter; ++point_i)
  {
   point.x = xyz_cloud->points[point_i].x * cos(steady_yaw) + xyz_cloud->points[point_i].y * sin(steady_yaw);
   point.y = xyz_cloud->points[point_i].x * (-sin(steady_yaw)) + xyz_cloud->points[point_i].y * cos(steady_yaw);
   point.x += steady_x + laser_sweep_xoffset + initial_xpose;
   point.y += steady_y + laser_sweep_yoffset + initial_ypose;
   point.z = xyz_cloud->points[point_i].z + height_offset;
   xyz_cloud_fused->points.push_back(point);
   Points_size += 1;
   }
     }
}
 else{
  cout << "no registration" << endl;
  }
  //update the variables
  last_x = current_xposition;
  last_y = current_yposition;
  last_yaw = yaw;
  if (last_yaw > 0.0)
      last_yaw = last_yaw - 2 * PI;
  //Points_size += xyz_cloud->points.size ();
  xyz_cloud_fused->width = Points_size;
  xyz_cloud_fused->height = 1;
  xyz_cloud_fused->points.resize (xyz_cloud_fused->width * xyz_cloud_fused->height);  // maximal points collected
 //convert back to ros message to be published  
  pcl::toROSMsg (*xyz_cloud_fused, MatchPointcloud::Fusion_msg);
  MatchPointcloud::Fusion_msg.header.stamp = ros::Time::now();
  MatchPointcloud::Fusion_msg.header.frame_id = "map";
  //puclish the message
  MatchPointcloud::fusion_pub.publish (MatchPointcloud::Fusion_msg);
}

int main(int argc, char** argv)
{
  //rangelog.open (range_file,ios::out | ios::trunc);  //  ios::app,   ios::ate ,other modes
  //timestamplog.open (timestamp_file,ios::out | ios::trunc);
  
  ros::init(argc, argv, "synchronizer");
  //ros::start();

  MatchPointcloud Fusion;
  //filte for sychcronization on timestamp for the two topics
  message_filters::Subscriber<sensor_msgs::PointCloud2> sweep_sub(Fusion.nh,"/sweep_fusion/cloudpoint", 1);
  message_filters::Subscriber<nav_msgs::Path> path_sub(Fusion.nh,"/trajectory", 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Path> sync_pol;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(5), sweep_sub, path_sub);
  sync.registerCallback(boost::bind(&MatchPointcloud::Callback, &Fusion, _1, _2));
  ros::spin();
  pcl::io::savePCDFileASCII (pcd_path, *xyz_cloud_fused);
  //pass through to filter out the measurements out of range
  pcl::PassThrough<pcl::PointCloud<pcl::PointXYZ>::PointType> pass1;
  pass1.setInputCloud (xyz_cloud_fused);
  pass1.setFilterFieldName ("z");
  pass1.setFilterLimits (1.0, 3.0);
  //pass.setFilterLimitsNegative (true);
  pass1.filter (*xyz_cloud_fused);

  pcl::PassThrough<pcl::PointXYZ> pass2;
  pass2.setInputCloud (xyz_cloud_fused);
  pass2.setFilterFieldName ("x");
  pass2.setFilterLimits (0.0, 9.5);
  //pass.setFilterLimitsNegative (true);
  pass2.filter (*xyz_cloud_fused);

  pcl::PassThrough<pcl::PointCloud<pcl::PointXYZ>::PointType> pass3;
  pass3.setInputCloud (xyz_cloud_fused);
  pass3.setFilterFieldName ("y");
  pass3.setFilterLimits (0.0, 15.0);
  //pass.setFilterLimitsNegative (true);

  //pcl ransac
  /*pcl::SampleConsensusModelPlane<pcl::PointXYZ >::Ptr model (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (xyz_cloud_fused));
  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model);
  ransac.setDistanceThreshold (0.015);
  bool result = ransac.computeModel ();
  std::vector<int> inliers;//boost::shared_ptr<vector<int> > inliers (new vector<int>);
  ransac.getInliers (inliers);
  pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloud (new pcl::PointCloud<pcl::PointXYZ>);  //points from a scan fitting into plane
  // copies all inliers of the model computed to another PointCloud
  pcl::copyPointCloud<pcl::PointXYZ>(*xyz_cloud_fused, inliers, *inlier_cloud);*/
  pcl::io::savePCDFileASCII (pcd_path2, *xyz_cloud_fused);
  //delete xyz_cloud_fused;
  return 0;
}
