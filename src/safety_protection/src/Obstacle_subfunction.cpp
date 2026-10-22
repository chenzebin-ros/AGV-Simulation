#include "std_msgs/String.h"

#include <cmath>
#include <vector>
#include <string>
#include <nav_msgs/Odometry.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
/*
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
*/

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
//#include <safety_protection/obstacle.h>
#include <safety_protection/obstacle_dete.h>
#include <opencv2/opencv.hpp>
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/UInt8.h"

using std::atan2;
using std::cos;
using std::sin;
using namespace std;
using namespace Eigen;
using namespace sensor_msgs;

typedef pcl::PointXYZI PointType;

//bool systemInited = false;
const int systemDelay = 0; 
//int systemInitCount = 0;

safety_protection::obstacle_dete waring_msgs;
std_msgs::UInt8MultiArray waring_msgs2;

int REGION2=7;
int point_quantity =20;

float max_y = 0.5;
float min_y = -0.5;

float car_clearance = 0.1;
float car_length = 2.5;
float car_width = 2.5;

float max_z = 1;
float min_z = -0.4;

float front1_x = 0.1;
float front2_x = 1.1;
float front3_x = 1.1;
float front4_x =  1.1;


float after1_x = -0.1;
float after2_x = -1.1;
float after3_x =  -1.1;
float after4_x = -1.1;

bool sids_flg = true;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

//template <typename PointT>

 
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
 

pcl::PointCloud<PointType> alarmcloud[5];
uint8_t speed2=0;
pcl::PointCloud<PointType> *Obstacle1_subfunction( pcl::PointCloud<pcl::PointXYZ> laserCloudobs,uint8_t speed ,std::vector<uint8_t> &vResultobs)
{
     //  printf("prepare time %f \n" );

     if ( speed==0 && speed2 != speed )
    {
        speed2 = speed;
        point_quantity= getParam<int>("low_safe_area/point_quantity",0);

           std::cout << "  point_quantity:  " << point_quantity << "  " << std::endl;
        max_y=getParam<float>("low_safe_area/max_y",0.0f);
        min_y = getParam<float>("low_safe_area/min_y",0.0f);
        max_z =getParam<float>("low_safe_area/max_z",0.0f);
        min_z=getParam<float>("low_safe_area/min_z",0.0f);
        front1_x=getParam<float>("low_safe_area/front1_x",0.0f);
        front2_x = getParam<float>("low_safe_area/front2_x",0.0f);
        front3_x = getParam<float>("low_safe_area/front3_x",0.0f);
        front4_x = getParam<float>("low_safe_area/front4_x",0.0f);

        after1_x= getParam<float>("low_safe_area/after1_x",0.0f);
        after2_x = getParam<float>("low_safe_area/after2_x",0.0f);
        after3_x = getParam<float>("low_safe_area/after3_x",0.0f);
        after4_x = getParam<float>("low_safe_area/after4_x",0.0f);

        car_length=  getParam<float>("car_size/car_length",0.0f);
        car_width =  getParam<float>("car_size/car_width",0.0f);
        car_clearance =  getParam<float>("car_size/car_clearance",0.0f);
    }
   else  if ( speed==1 && speed2 != speed )
    {
        speed2 = speed;
        point_quantity= getParam<int>("low_safe_area/point_quantity",0);
        max_y= getParam<float>("low_safe_area/max_y",0.0f);
        min_y = getParam<float>("low_safe_area/min_y",0.0f);
        max_z = getParam<float>("low_safe_area/max_z",0.0f);
        min_z= getParam<float>("low_safe_area/min_z",0.0f);
        front1_x= getParam<float>("low_safe_area/front1_x",0.0f);
        front2_x = getParam<float>("low_safe_area/front2_x",0.0f);
        front3_x = getParam<float>("low_safe_area/front3_x",0.0f);
        front4_x = getParam<float>("low_safe_area/front4_x",0.0f);

        after1_x= getParam<float>("low_safe_area/after1_x",0.0f);
        after2_x = getParam<float>("low_safe_area/after2_x",0.0f);
        after3_x = getParam<float>("low_safe_area/after3_x",0.0f);
        after4_x = getParam<float>("low_safe_area/after4_x",0.0f);

        car_length= getParam<float>("car_size/car_length",0.0f);
        car_width = getParam<float>("car_size/car_width",0.0f);
        car_clearance = getParam<float>("car_size/car_clearance",0.0f);
    }
else if (speed==2 && speed2 != speed)
{
    speed2 = speed;
      point_quantity= getParam<int>("med_safe_area/point_quantity",0);
        max_y= getParam<float>("med_safe_area/max_y",0.0f);
        min_y = getParam<float>("med_safe_area/min_y",0.0f);
        max_z = getParam<float>("med_safe_area/max_z",0.0f);
        min_z= getParam<float>("med_safe_area/min_z",0.0f);
        front1_x= getParam<float>("med_safe_area/front1_x",0.0f);
        front2_x = getParam<float>("med_safe_area/front2_x",0.0f);
        front3_x = getParam<float>("med_safe_area/front3_x",0.0f);
        front4_x = getParam<float>("med_safe_area/front4_x",0.0f);

        after1_x= getParam<float>("med_safe_area/after1_x",0.0f);
        after2_x = getParam<float>("med_safe_area/after2_x",0.0f);
        after3_x = getParam<float>("med_safe_area/after3_x",0.0f);
        after4_x = getParam<float>("med_safe_area/after4_x",0.0f);

        car_length= getParam<float>("car_size/car_length",0.0f);
        car_width = getParam<float>("car_size/car_width",0.0f);
        car_clearance = getParam<float>("car_size/car_clearance",0.0f);
}
else if (speed==3 && speed2 != speed)
{
    speed2 = speed;
      point_quantity= getParam<int>("high_safe_area/point_quantity",0);
        max_y= getParam<float>("high_safe_area/max_y",0.0f);
        min_y = getParam<float>("high_safe_area/min_y",0.0f);
        max_z = getParam<float>("high_safe_area/max_z",0.0f);
        min_z= getParam<float>("high_safe_area/min_z",0.0f);
        front1_x= getParam<float>("high_safe_area/front1_x",0.0f);
        front2_x = getParam<float>("high_safe_area/front2_x",0.0f);
        front3_x = getParam<float>("high_safe_area/front3_x",0.0f);
        front4_x = getParam<float>("high_safe_area/front4_x",0.0f);

        after1_x= getParam<float>("high_safe_area/after1_x",0.0f);
        after2_x = getParam<float>("high_safe_area/after2_x",0.0f);
        after3_x = getParam<float>("high_safe_area/after3_x",0.0f);
        after4_x = getParam<float>("high_safe_area/after4_x",0.0f);

        car_length= getParam<float>("car_size/car_length",0.0f);
        car_width = getParam<float>("car_size/car_width",0.0f);
        car_clearance = getParam<float>("car_size/car_clearance",0.0f);
}
else if (speed== 4 && speed2 != speed)
{
    speed2 = speed;
      point_quantity= getParam<int>("dock_safe_area/point_quantity",0);
        max_y= getParam<float>("dock_safe_area/max_y",0.0f);
        min_y = getParam<float>("dock_safe_area/min_y",0.0f);
        max_z = getParam<float>("dock_safe_area/max_z",0.0f);
        min_z= getParam<float>("dock_safe_area/min_z",0.0f);
        front1_x= getParam<float>("dock_safe_area/front1_x",0.0f);
        front2_x = getParam<float>("dock_safe_area/front2_x",0.0f);
        front3_x = getParam<float>("dock_safe_area/front3_x",0.0f);
        front4_x = getParam<float>("dock_safe_area/front4_x",0.0f);

        after1_x= getParam<float>("dock_safe_area/after1_x",0.0f);
        after2_x = getParam<float>("dock_safe_area/after2_x",0.0f);
        after3_x = getParam<float>("dock_safe_area/after3_x",0.0f);
        after4_x = getParam<float>("dock_safe_area/after4_x",0.0f);

         car_length= getParam<float>("car_size/car_length",0.0f);
        car_width = getParam<float>("car_size/car_width",0.0f);
        car_clearance = getParam<float>("car_size/car_clearance",0.0f);
}

 //after1_x = -0.1;
 //after2_x = -1.1;
 //after3_x = 0;//-1.1;
// after4_x =0;// -6.1;
 point_quantity =60;
//printf("point_quantity %d \n", point_quantity );
 // 首先对点云滤波，去除NaN值的无效点，以及在Lidar坐标系原点MINIMUM_RANGE距离以内的点
 //   pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
 //   removeClosedPointCloud(laserCloudIn, laserCloudIn, MinIMUM_RANGE);

   
  //  for ( int i = 0;  i < laserCloudIn.points.size();  i++ ) {
 //   	cout << laserCloudIn.points[i].x << ", " << laserCloudIn.points[i].y << ", " << laserCloudIn.points[i].z << endl;
//	}
   int cloudSize = laserCloudobs.points.size();
   PointType point;
   std::vector<pcl::PointCloud<PointType>> laserCloudScans(REGION2);

   float car_xx = car_clearance + car_length/2.0;
   float car_yy = car_clearance + car_width/2.0;

    std::cout << "  car_xx:  " << car_xx << "  "
     << " car_yy:  " << car_yy << "  "
     << " front1_x:  " << front1_x << "  "
      << " front2_x:  " << front2_x  << " after1_x:  " << after1_x<< " after2_x:  " << after2_x << std::endl;

  std::cout << "   after2_x  - car_xx:  "<<  after2_x  - car_xx<< "   after1_x - car_xx:  "<<  after1_x - car_xx << std::endl;
for (int i = 0; i < cloudSize; i++)
    {
        point.x = laserCloudobs.points[i].x;
        point.y = laserCloudobs.points[i].y;
        point.z = laserCloudobs.points[i].z;

        if ( (point.x  > ( -car_xx ) + after4_x &&  point.x <  ( car_xx ) + front4_x) && (point.y  > ( -car_yy ) +min_y &&  point.y <  ( car_yy ) + max_y) &&  (point.z  > min_z &&  point.z < max_z)  )
        {
            /* code for True */
            if (point.y > car_yy || point.y <  -car_yy  )
            {
                 laserCloudScans[0].push_back(point); //车辆两侧点云
            }
            else {
                    if ( point.x  > front1_x + car_xx &&  point.x  < front2_x  + car_xx   )//前
                    {
                        /* code for True */
                       //  std::cout << "  car_xx111111111:  "<< std::endl;
                        laserCloudScans[1].push_back(point); //第一区域
                    }
                    if ( point.x  > front3_x  + car_xx &&  point.x < front4_x + car_xx   )
                    {
                        laserCloudScans[2].push_back(point); //第二区域，存在角迭区
                    }


                    if ( point.x  > after2_x  - car_xx  &&  point.x < after1_x - car_xx    )//后
                    {
                          //  std::cout << "  car_xx000000000:  "<< std::endl;
                        /* code for True */
                        laserCloudScans[3].push_back(point); //第一区域
                    }
                    if ( point.x  > after4_x - car_xx  &&  point.x < after3_x  - car_xx   )
                    {
                        laserCloudScans[4].push_back(point); //第二区域，存在角迭区
                    }
            }                                
        }    
    }

   int zero_region_side =  laserCloudScans[0].points.size();
   int one_region_front =  laserCloudScans[1].points.size();
    int two_region_front =  laserCloudScans[2].points.size();
    int one_region_after =  laserCloudScans[3].points.size();
    int two_region_after =  laserCloudScans[4].points.size();
    std::cout << " one_region_front:  "<<one_region_front<< " one_region_after:  "<<one_region_after<< std::endl;

    alarmcloud[0] =  laserCloudScans[1];
    alarmcloud[1] =  laserCloudScans[2];
    alarmcloud[2] =  laserCloudScans[3];
    alarmcloud[3] =  laserCloudScans[4];
    alarmcloud[4] =  laserCloudScans[0];

    if (one_region_front > point_quantity)
    {
        waring_msgs.front_warning1 = 1;
    }
    else waring_msgs.front_warning1 = 0;

    if (two_region_front > point_quantity)
    {
        waring_msgs.front_warning2 = 1;
    }
    else waring_msgs.front_warning2 = 0;

    if (zero_region_side > point_quantity)
    {
        waring_msgs.front_warning3 = 1;
    }
    else waring_msgs.front_warning3 = 0;

//    waring_msgs.front_warning3 = 0;
//========================================================================
     if (one_region_after > point_quantity)
    {
        waring_msgs.after_warning1 = 1;
    }
    else   waring_msgs.after_warning1 = 0;

    if (two_region_after > point_quantity)
    {
        waring_msgs.after_warning2 = 1;
    }
    else   waring_msgs.after_warning2 = 0;

     if (zero_region_side > point_quantity)
    {
        waring_msgs.after_warning3 = 1;
    }
    else waring_msgs.after_warning3 = 0;

  //    waring_msgs.after_warning3 = 0;
//============================================================
    if ( cloudSize==0 )
    {
        waring_msgs.data_warning = 0;
    }
    else waring_msgs.data_warning = 1;

    vResultobs.push_back(waring_msgs.front_warning1);
    vResultobs.push_back( waring_msgs.front_warning2);
    vResultobs.push_back(waring_msgs.front_warning3);
    vResultobs.push_back(waring_msgs.after_warning1);
    vResultobs.push_back( waring_msgs.after_warning2);
    vResultobs.push_back( waring_msgs.after_warning3);
    vResultobs.push_back (waring_msgs.data_warning);

/*
 sensor_msgs::PointCloud2 laserCloudOutMsg;
    pcl::toROSMsg( laserCloudScans[1], laserCloudOutMsg);
    //laserCloudOutMsg.header.stamp = laserCloudobs.header.stamp;
    laserCloudOutMsg.header.frame_id = "/lidar_link";
    pubalarmCloud.publish(laserCloudOutMsg);
*/

//waring_msgs.stamp =  laserCloudMsg->header.stamp;
// obstacle_warning.publish(waring_msgs2);

 //printf("one_region_front %d \n", one_region_front );
 //printf("two_region_front %d \n", two_region_front );
 // printf("rr %f \n", vResult2[2]  );
//  printf("one_region_after %d \n", one_region_after  );
 //// printf("two_region_after %d \n", two_region_after  );
return alarmcloud;

}


