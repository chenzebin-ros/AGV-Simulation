#include "std_msgs/String.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#include <cmath>
#include <vector>
#include <string>
#include <nav_msgs/Odometry.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/UInt8MultiArray.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
//#include <safety_protection/obstacle.h>
#include "std_msgs/UInt8MultiArray.h"
#include <opencv2/opencv.hpp>
#include <safety_protection/line_para_msgs.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include "std_msgs/UInt8.h"

using std::atan2;
using std::cos;
using std::sin;
using namespace std;
using namespace Eigen;
using namespace sensor_msgs;

typedef pcl::PointXYZI PointType;

bool systemInited = false;
const int systemDelay = 0; 
int systemInitCount = 0;
int N_SCANS = 16;

float cloudCurvature[400000];
int cloudSortInd[400000];
int cloudNeighborPicked[400000];
int cloudLabel[400000];

bool comp (int i,int j) { return (cloudCurvature[i]<cloudCurvature[j]); }

double MINIMUM_RANGE = 1.6; 

const double scanPeriod = 0.1;

uint8_t area_nu=0;
uint8_t lidar_date=1;
std_msgs::Float32MultiArray parame2;
std_msgs::UInt8MultiArray waring_para2;
safety_protection::line_para_msgs parame;

 float  ob_max_y;
 float  ob_front1_x ;
 float  ob_front2_x ;
float  ob_after_x ,ob_after_z;
int point_quantity2;

pcl::PointCloud<PointType> *alarm_cloud;

ros::Publisher puboneCloud;
ros::Publisher publineCloud;
ros::Publisher pubpara;
ros::Publisher pub_bounding_boxsob_ ;
ros::Publisher publinehoughCloud;
ros::Publisher pubalarm0Cloud;
ros::Publisher pubalarm1Cloud;
ros::Publisher pubalarm2Cloud;
ros::Publisher pubalarm3Cloud;
ros::Publisher pubalarm4Cloud;
ros::Publisher pubtargCloud;
ros::Publisher pubLaserCloud;
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
//void LineFitLeastSquares(float *data_x, float *data_y, int data_n,  std::vector<float> &vResult);
pcl::PointCloud<PointType> *Obstacle1_subfunction( pcl::PointCloud<pcl::PointXYZ> laserCloudobs,uint8_t speed ,std::vector<uint8_t> &vResultobs);
//pcl::PointCloud<PointType> Line_Hough(pcl::PointCloud<PointType> laserCloudobs );
//pcl::PointCloud<PointType> Close2target( pcl::PointCloud<PointType> laserCloudobs ,  std::vector<int8_t> &vResult);

 
//读取参数模板
template<typename T>
T getParam(const std::string& name,const T& defaultValue)//This name must be namespace+parameter_name
{
    T v;
    if(ros::param::get(name,v))//get parameter by name depend on ROS.
    {
        ROS_INFO_STREAM("Found parameter: "<<name<<",\tvalue: "<<v);
        return v;
    }
    else 
        ROS_WARN_STREAM("Cannot find value for parameter: "<<name<<",\tassigning default: "<<defaultValue);
    return defaultValue;//if the parameter haven't been set,it's value will return defaultValue.
}



template <typename PointT>
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                              pcl::PointCloud<PointT> &cloud_out, float thres)
{
	 std_msgs::UInt8 obsDate;
	 
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;
	size_t lengg1 = 0;
	size_t lengg2 = 0;

    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres || cloud_in.points[i].z <ob_after_z)
            continue;
            
        if (cloud_in.points[i].x> ob_after_x && cloud_in.points[i].x< ob_front1_x && cloud_in.points[i].y > 0-ob_max_y && cloud_in.points[i].y < ob_max_y )
	  {
		 lengg1 ++;

	  }
      
      if (cloud_in.points[i].x> ob_front1_x && cloud_in.points[i].x< ob_front2_x && cloud_in.points[i].y > 0-ob_max_y && cloud_in.points[i].y < ob_max_y )
      {
         lengg2 ++;	  
      }   
            
         
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if(lengg2 > point_quantity2)
    {
    	obsDate.data = 2;
    }
    
    if(lengg1 > point_quantity2)
    {
    	obsDate.data = 1;
    }    
    
     pubpara.publish(obsDate);
    
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}



void arer_numberHandler(const std_msgs::UInt8MultiArray &paraMsg)
{
    area_nu = paraMsg.data[0];

}


void arer_dateHandler(const safety_protection::line_para_msgs &paraMsg)
{
    lidar_date = paraMsg.data_warning;

}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{

    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
    std::vector<int> indices;

 // 首先对点云滤波，去除NaN值的无效点，以及在Lidar坐标系原点MINIMUM_RANGE距离以内的点
    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
    removeClosedPointCloud(laserCloudIn, laserCloudIn, MINIMUM_RANGE);
    
    sensor_msgs::PointCloud2 laserCloudOutMsg2;
    pcl::toROSMsg(laserCloudIn, laserCloudOutMsg2);

    laserCloudOutMsg2.header.stamp =  ros::Time::now();//ros::Time().fromSec(time_Point);
    laserCloudOutMsg2.header.frame_id = "/velodyne";
    pubLaserCloud.publish(laserCloudOutMsg2); 
}      


void bounding_boxesHandler (const jsk_recognition_msgs::BoundingBoxArray &paraMsg)
{
  std_msgs::UInt8 obsDate;
     jsk_recognition_msgs::BoundingBoxArray bbox_array;
  int obSize =  paraMsg.boxes.size();
  obsDate.data = 0;
  for (int  i = 0; i < obSize; i++)
  {
      double obSizeX = paraMsg.boxes[i].pose.position.x;
      double obSizeY = paraMsg.boxes[i].pose.position.y;

	  if (obSizeX> ob_after_x && obSizeX< ob_front1_x && obSizeY > 0-ob_max_y && obSizeY < ob_max_y )
	  {
		 bbox_array.boxes.push_back(paraMsg.boxes[i]);
		 obsDate.data = 1;
		 bbox_array.header = paraMsg.header;
		 pub_bounding_boxsob_.publish(bbox_array);
		 pubpara.publish(obsDate);		 
		 break;
	  }
      
      if (obSizeX>ob_after_x && obSizeX< ob_front2_x && obSizeY > 0-ob_max_y && obSizeY < ob_max_y )
      {
         bbox_array.boxes.push_back(paraMsg.boxes[i]); 
        obsDate.data = 2;		  
      }
      
      /* code for loop body */
  }

  bbox_array.header = paraMsg.header;
  pub_bounding_boxsob_.publish(bbox_array);
  pubpara.publish(obsDate);

}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "obstacle_handle");

  ros::NodeHandle n;

 point_quantity2=getParam<float>("com_safe_area/point_quantity",0);
 ob_max_y=getParam<float>("com_safe_area/max_y",0.0f);
 ob_front1_x = getParam<float>("com_safe_area/front1_x",0.0f);
 ob_front2_x = getParam<float>("com_safe_area/front2_x",0.0f);
 ob_after_x  = getParam<float>("com_safe_area/after_x",0.0f);
 ob_after_z  = getParam<float>("com_safe_area/after_z",0.0f);
 std::cout << "   point_quantity2:  "<<  point_quantity2<< "   max_y:  "<<  ob_max_y<< "   front1_x:  "<< ob_front1_x << "   front2_x:  "<< ob_front2_x  <<" after_x " << ob_after_x<<" after_z " << ob_after_z<< std::endl;

ros::Subscriber arer_number = n.subscribe("/region_para", 100, arer_numberHandler);
ros::Subscriber arer_date = n.subscribe("/lidardate_para", 1, arer_dateHandler);
ros::Subscriber bounding_boxes_date = n.subscribe("/bounding_boxes", 1, bounding_boxesHandler);

ros::Subscriber subLaserCloud = n.subscribe<sensor_msgs::PointCloud2>("/points_raw", 2, laserCloudHandler);

 pub_bounding_boxsob_ =n.advertise<jsk_recognition_msgs::BoundingBoxArray>("/obs_bounding_boxes", 5);
pubLaserCloud =n.advertise<sensor_msgs::PointCloud2>("/points_raw_obs", 100);

pubpara =n.advertise<std_msgs::UInt8>("/obs_data", 100);

  ros::Rate loop_rate(10);
   while (ros::ok()) 
   {
  
        ros::spinOnce(); 
        loop_rate.sleep(); 
     }
        
  //ros::spin();

  return 0;
}

