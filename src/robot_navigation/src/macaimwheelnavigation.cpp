#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<sys/socket.h>
#include<netinet/in.h>
#include<arpa/inet.h>
#include<unistd.h>
#include<iostream>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/UInt32MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <cmath>
//#include "geometry_msgs/PoseStamped"
using namespace std;

//navgation_pack::androidyaokong wangluo;
uint8_t  loc[10] = { 0xff, 0xff,0xff,0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff };
uint8_t output_date[] = {0xff,0xaa,0x00,0xf2,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x22,0xee,0xee};
uint8_t output_handle[] = {0xff,0xaa,0x00,0xf2,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x22,0xee,0xee};
 int fd ;
int socket_fd ;
 uint8_t lidar_date=1;
bool send_flg = false;
uint16_t send_cont = 0;
uint8_t erry_map_data=3;
uint8_t loadmap_data=0;
char *XX;
char *YY;
char *ang;
char *state;

 
double current_xx = 0;
double current_yy = 0;
double current_heading = 0;
double star_site[2] = { 0 };
double end_site[2] = { 0 };
double current_pose[3] = { 0 };
 int pathleng = 1;
std_msgs::Float64MultiArray Path_Point;
uint16_t pathupflag = 0;
uint32_t starcar=0,runcarnum=0,starcarend=1,car_war=0,starcarmod=1;
//struct mcnamu_robot by_robot = {0,0,0,0,false,false,false,false, 0, true };

ros::Publisher pubchongdingwei;
ros::Publisher pubnavigdata;
//-------------------------------------麦克纳姆论导航返回参数-------------------------------------------------
struct mcnamu_robot
{
	float  lfrontwheelspeed;    //左前轮速度
    float  rfrontwheelspeed;    //右前轮速度
	float  lbackwheelspeed;    //左后轮速度
    float  rbackwheelspeed;    //右后轮速度

     float starend_distance;  //距离终点的距离
    float targ_distance;  //距离终点的距离
    float driv_distance;  //距离起点的距离    
};

//----------------------------限副函数-------------------------------------
float LIMIT (float mindata ,float actdata ,float maxdata)
{
    float redata;
    redata = actdata >= mindata ? actdata : mindata;
    redata = redata <= maxdata ? redata : maxdata;
    return redata;
}
//-----------------------------求向量夹角-----------------------------------
float VectorAngle( float vector_ax ,float vector_ay, float vector_bx ,float vector_by)
{
    float vectoranle;
    float vector_axn = vector_ax*vector_by - vector_bx*vector_ay;   //a x n乘积,求偏角
    float vector_a_n = vector_ax*vector_bx + vector_by*vector_ay;   //a 点 n乘积,求偏角
    //float err_angle = asin(vector_axn/ vector_alm);                     //追踪线与车辆夹角，注意此反正弦舍弃，范围正负90
    // err_angle = err_angle*180/3.1415926;
    float err_angle = atan2( vector_axn, vector_a_n);                     //追踪线与车辆夹角，，范围正负180
    vectoranle = err_angle*180/3.1415926;
    return vectoranle;
}
//-----------------------------求向量垂直距离------------------------------------
float VectorDistance( float vector_ax ,float vector_ay, float vector_bx ,float vector_by)
{
    float vectordistance;
    float vector_axb = vector_ax*vector_by - vector_bx*vector_ay;   //a x b乘积
    float vector_alm =  sqrt(vector_ax*vector_ax + vector_ay*vector_ay );   //AB路径a向量模长
    float err_distance = vector_axb / vector_alm ;   //求偏距，包括方向
    vectordistance = err_distance;
    return vectordistance;
}

//-------------------------------------------------------------------------------------
mcnamu_robot mcnamu_navigation3(double *cur_pose, double *star_point,  double *end_point ,  float track, float wheelbase, float vtx, float *p_con,bool dirflag,bool TranOrRota )
{

vtx = 0-vtx;
 std::cout << "  cur_pose[0]:  " << cur_pose[0] << "  "
         << "cur_pose[1]:  "  << cur_pose[1] << "  "
          << "cur_pose[2] :  " << cur_pose[2]  << std::endl;

           std::cout << "  star_point[0]:  " << star_point[0] << "  "
         << "star_point[1]:  "  << star_point[1] << "  "
          << "end_point[0] :  " << end_point[0]    << "  end_point[1] :  " << end_point[1]  << std::endl;

    struct mcnamu_robot by_robot = {0,0,0,0, 0, 0, 0 };
    static bool  VerticalOrHorizontal = true;

    //  a x b = |a|x|b|xsin() =  x1y2 - x2y1;求偏距|b|xsin() =  ( x1y2 - x2y1)/ |a|
    //  大于 0 在左侧;  小于 0 在右侧;  等于 0 在直线上

    float vector_ax = end_point[0] - star_point[0] ;     //AB路径a向量x
    float vector_ay = end_point[1] - star_point[1] ;     //AB路径a向量y
    float vector_bx = cur_pose[0] - star_point[0] ;     //AC方向b向量x
    float vector_by = cur_pose[1] - star_point[1] ;     //AC方向b向量y

    float car_nx = cos( cur_pose[2] *3.1415926 / 180.0 );                              //车辆单位向量x
    float car_ny = sin( cur_pose[2] *3.1415926 / 180.0 );                              //车辆单位向量y

    printf("car_nx[0] %f  \n", car_nx);   
    printf("car_ny[1] %f  \n", car_ny);  

     float err_distance =0 - VectorDistance( vector_ax, vector_ay, vector_bx ,vector_by ) ;   //求偏距，包括方向
     float err_angle2 = 0-VectorAngle(vector_ax,vector_ay,  car_nx, car_ny);                     //追踪线与车辆夹角，，范围正负180

   float limit_w = 500.0;
   // dirflag=true;
     if ( dirflag==true ) //纵向横向判断
    {
        if (err_angle2 > 120 || err_angle2 < -120)
        {
            vector_ax = star_point[0] - end_point[0] ;  
            vector_ay = star_point[1] - end_point[1];
            vtx = 0 - vtx;
            err_angle2 = 0- VectorAngle(vector_ax,vector_ay,  car_nx,car_ny);                     //追踪线与车辆夹角，，范围正负180;  
            err_distance = 0- err_distance;
        }

       // if (TranOrRota == false)
        {
           if (fabs(err_angle2) > 10)
            {
                vtx = 0;
                p_con[0] = 0;
                p_con[1] = 100;
                limit_w = 1000.0;
            }
        }    

    }
    else 
    {
         if ( err_angle2>0  && err_angle2 < 180 ) 
        {
            vector_ax = star_point[0] - end_point[0] ;  
            vector_ay = star_point[1] - end_point[1];
             vtx = 0 - vtx;
            err_angle2 = 0- VectorAngle(vector_ax,vector_ay,  car_nx,car_ny);                     //追踪线与车辆夹角，，范围正负180;  
            err_distance = 0- err_distance;
        }      
    }

    float vty =0;
    float w = 0;
    if ( dirflag==true )
    {
        vty = err_distance * p_con[0];     //计算x方向平移速度分量
        w = err_angle2 * p_con[1] * ( track + wheelbase) / 2.0;     //角速度分量
        //限幅
        vty = LIMIT(-400.0, vty ,400.0);
        w    = LIMIT(-limit_w, w ,limit_w);
    }
    else
    {
        vty = vtx;
        vtx = 0 - err_distance * p_con[0];     //计算y方向平移速度分量
        w = (err_angle2+90 )* p_con[1] * ( track + wheelbase) / 2.0;     //角速度分量
        //限幅
        vtx = LIMIT(-400.0, vtx ,400.0);
        w    = LIMIT(-limit_w, w ,limit_w); 
    }

    float star_end = sqrt( (star_point[0] - end_point[0])* (star_point[0] - end_point[0]) + (star_point[1] - end_point[1])* (star_point[1] - end_point[1]) );     // 路径长度，为调用函数判断使用
    float remainder_distance = sqrt( (cur_pose[0] - end_point[0])* (cur_pose[0] - end_point[0]) + (cur_pose[1] - end_point[1])* (cur_pose[1] - end_point[1]) );     // 车辆距离终点长度，为调用函数判断使用
    float driving_distance = sqrt( (cur_pose[0] - star_point[0])* (cur_pose[0] - star_point[0]) + (cur_pose[1] - star_point[1])* (cur_pose[1] - star_point[1]) );     // 车辆在AB路径已经行使的距离，为调用函数判断使用


    printf("star_end[0] %f  \n", star_end);   
    printf("driving_distance[1] %f  \n", driving_distance);  

    by_robot.lfrontwheelspeed = vtx + vty  -  w;     //左前轮速度
    by_robot.rfrontwheelspeed = vtx - vty  + w;     //右前轮速度
    by_robot.lbackwheelspeed = vtx -  vty  -  w;     //左后轮速度
    by_robot.rbackwheelspeed = vtx + vty  + w;     //右后轮速度
    by_robot.targ_distance = remainder_distance; //车辆距离终点长度
    by_robot.driv_distance = driving_distance;          // 车辆在AB路径已经行使的距离
    by_robot.starend_distance = star_end;               //AB线路径长度

     std::cout << " vty:  " << vty << "  "
         << "vtx:  "  << vtx << "  "
          << "w :  " << w   << " err_distance :  " << err_distance  << " err_angle2 :  " << err_angle2 << " driving_distance :  " << driving_distance  << " star_end :  " << star_end<< " VerticalOrHorizontal :  " << VerticalOrHorizontal<< std::endl;
 
   return by_robot;
}


//-------------------------------------------------------------------------------------
mcnamu_robot mcnamu_navigation2(double *cur_pose, double *star_point,  double *end_point ,  float track, float wheelbase, float vty, float *p_con,bool dirflag,bool TranOrRota )
{

 std::cout << "  cur_pose[0]:  " << cur_pose[0] << "  "
         << "cur_pose[1]:  "  << cur_pose[1] << "  "
          << "cur_pose[2] :  " << cur_pose[2]  << std::endl;

           std::cout << "  star_point[0]:  " << star_point[0] << "  "
         << "star_point[1]:  "  << star_point[1] << "  "
          << "end_point[0] :  " << end_point[0]    << "  end_point[1] :  " << end_point[1]  << std::endl;

    struct mcnamu_robot by_robot = {0,0,0,0, 0, 0, 0 };
    static bool  VerticalOrHorizontal = true;

    //  a x b = |a|x|b|xsin() =  x1y2 - x2y1;求偏距|b|xsin() =  ( x1y2 - x2y1)/ |a|
    //  大于 0 在左侧;  小于 0 在右侧;  等于 0 在直线上

    float vector_ax = end_point[0] - star_point[0] ;     //AB路径a向量x
    float vector_ay = end_point[1] - star_point[1] ;     //AB路径a向量y
    float vector_bx = cur_pose[0] - star_point[0] ;     //AC方向b向量x
    float vector_by = cur_pose[1] - star_point[1] ;     //AC方向b向量y

    float car_nx = cos( cur_pose[2] *3.1415926 / 180.0 );                              //车辆单位向量x
    float car_ny = sin( cur_pose[2] *3.1415926 / 180.0 );                              //车辆单位向量y

    printf("car_nx[0] %f  \n", car_nx);   
    printf("car_ny[1] %f  \n", car_ny);  

     float err_distance =0 - VectorDistance( vector_ax, vector_ay, vector_bx ,vector_by ) ;   //求偏距，包括方向
     float err_angle2 = 0-VectorAngle(vector_ax,vector_ay,  car_nx, car_ny);                     //追踪线与车辆夹角，，范围正负180

   float limit_w = 500.0;
   // dirflag=true;
     if ( dirflag==true ) //纵向横向判断
    {
        if (err_angle2 > 120 || err_angle2 < -120)
        {
            vector_ax = star_point[0] - end_point[0] ;  
            vector_ay = star_point[1] - end_point[1];
            vty = 0 - vty;
            err_angle2 = 0- VectorAngle(vector_ax,vector_ay,  car_nx,car_ny);                     //追踪线与车辆夹角，，范围正负180;  
            err_distance = 0- err_distance;
        }

       // if (TranOrRota == false)
        {
           if (fabs(err_angle2) > 10)
            {
                vty = 0;
                p_con[0] = 0;
                p_con[1] = 100;
                limit_w = 1000.0;
            }
        }    

    }
    else 
    {
         if ( err_angle2>0  && err_angle2 < 180 ) 
        {
            vector_ax = star_point[0] - end_point[0] ;  
            vector_ay = star_point[1] - end_point[1];
             vty = 0 - vty;
            err_angle2 = 0- VectorAngle(vector_ax,vector_ay,  car_nx,car_ny);                     //追踪线与车辆夹角，，范围正负180;  
            err_distance = 0- err_distance;
        }      
    }

    float vtx =0;
    float w = 0;
    if ( dirflag==true )
    {
        vtx = err_distance * p_con[0];     //计算x方向平移速度分量
        w = err_angle2 * p_con[1] * ( track + wheelbase) / 2.0;     //角速度分量
        //限幅
        vtx = LIMIT(-400.0, vtx ,400.0);
        w    = LIMIT(-limit_w, w ,limit_w);
    }
    else
    {
        vtx = vty;
        vty = 0 - err_distance * p_con[0];     //计算y方向平移速度分量
        w = (err_angle2+90 )* p_con[1] * ( track + wheelbase) / 2.0;     //角速度分量
        //限幅
        vty = LIMIT(-400.0, vty ,400.0);
        w    = LIMIT(-limit_w, w ,limit_w); 
    }

    float star_end = sqrt( (star_point[0] - end_point[0])* (star_point[0] - end_point[0]) + (star_point[1] - end_point[1])* (star_point[1] - end_point[1]) );     // 路径长度，为调用函数判断使用
    float remainder_distance = sqrt( (cur_pose[0] - end_point[0])* (cur_pose[0] - end_point[0]) + (cur_pose[1] - end_point[1])* (cur_pose[1] - end_point[1]) );     // 车辆距离终点长度，为调用函数判断使用
    float driving_distance = sqrt( (cur_pose[0] - star_point[0])* (cur_pose[0] - star_point[0]) + (cur_pose[1] - star_point[1])* (cur_pose[1] - star_point[1]) );     // 车辆在AB路径已经行使的距离，为调用函数判断使用


    printf("star_end[0] %f  \n", star_end);   
    printf("driving_distance[1] %f  \n", driving_distance);  

    by_robot.lfrontwheelspeed = vty + vtx  -  w;     //左前轮速度
    by_robot.rfrontwheelspeed = vty - vtx  + w;     //右前轮速度
    by_robot.lbackwheelspeed = vty -  vtx  -  w;     //左后轮速度
    by_robot.rbackwheelspeed = vty + vtx  + w;     //右后轮速度
    by_robot.targ_distance = remainder_distance; //车辆距离终点长度
    by_robot.driv_distance = driving_distance;          // 车辆在AB路径已经行使的距离
    by_robot.starend_distance = star_end;               //AB线路径长度

     std::cout << " vty:  " << vty << "  "
         << "vtx:  "  << vtx << "  "
          << "w :  " << w   << " err_distance :  " << err_distance  << " err_angle2 :  " << err_angle2 << " driving_distance :  " << driving_distance  << " star_end :  " << star_end<< " VerticalOrHorizontal :  " << VerticalOrHorizontal<< std::endl;
 
   return by_robot;
}


/*---------------------------------------麦克纳姆论导航算法-------------------------------------------------------------------------------------
输入量：
double *cur_pose: 指针类型，表示当前的位置航向信息，cur_pose[0-2]分别为但前的x,y,航向角，航向角范围为(0,2pi)
double *star_point:指针类型，表示路径起点坐标信息，star_point[0-1]分别表示起点的x,y;
double *end_point:指针类型，表示路径起点坐标信息，end_point[0-1]分别表示终点的x,y;
 float track: 浮点数类型 ，表示车辆的轮距
 float wheelbase: 浮点数类型 ，表示车辆的轴距
float vty: 浮点数类型 ，表示车辆运行速度
float *p_con: 指针类型，表示控制参数，p_con[0-1]分别表示平移比例系数和旋转比例系数
bool dirflag：bool类型，true为纵向追踪，false为横向追踪
输出量：
结构体类型，包括4个轮子的转速，车辆已经的运行距离，车辆距离终点的距离，以及本次任务是否执行结束
-------------------------------------------------------------------------------------------------------------------------------------------------------*/
mcnamu_robot mcnamu_navigation(double *cur_pose, double *star_point,  double *end_point ,  float track, float wheelbase, float vty, float *p_con,bool dirflag,bool runcar )
{

 std::cout << "  cur_pose[0]:  " << cur_pose[0] << "  "
         << "cur_pose[1]:  "  << cur_pose[1] << "  "
          << "cur_pose[2] :  " << cur_pose[2]  << std::endl;

           std::cout << "  star_point[0]:  " << star_point[0] << "  "
         << "star_point[1]:  "  << star_point[1] << "  "
          << "end_point[0] :  " << end_point[0]    << "  end_point[1] :  " << end_point[1]  << std::endl;

    struct mcnamu_robot by_robot = {0,0,0,0, 0, 0, false };
    //  a x b = |a|x|b|xsin() =  x1y2 - x2y1;求偏距|b|xsin() =  ( x1y2 - x2y1)/ |a|
    //  大于 0 在左侧;  小于 0 在右侧;  等于 0 在直线上


    float vector_ax = end_point[0] - star_point[0] ;     //AB路径a向量x
    float vector_ay = end_point[1] - star_point[1] ;     //AB路径a向量y
    float vector_bx = cur_pose[0] - star_point[0] ;     //AC方向b向量x
    float vector_by = cur_pose[1] - star_point[1] ;     //AC方向b向量y

      if (runcar==false)
    {
       vector_ax = star_point[0] - end_point[0] ;  
       vector_ay = star_point[1] - end_point[1];
        vty = 0 - vty;
    }

    float car_nx = cos( cur_pose[2] *3.1415926 / 180.0 );                              //车辆单位向量x
    float car_ny = sin( cur_pose[2] *3.1415926 / 180.0 );                              //车辆单位向量y

    printf("car_nx[0] %f  \n", car_nx);   
    printf("car_ny[1] %f  \n", car_ny);  

    float vector_axb = vector_ax*vector_by - vector_bx*vector_ay;   //a x b乘积
    float vector_alm =  sqrt(vector_ax*vector_ax + vector_ay*vector_ay );   //AB路径a向量模长
    float err_distance =0 - vector_axb / vector_alm ;   //求偏距，包括方向

    float vector_axn = vector_ax*car_ny - car_nx*vector_ay;   //a x n乘积,求偏角
     float vector_a_n = vector_ax*car_nx + car_ny*vector_ay;   //a 点 n乘积,求偏角
   //float err_angle = asin(vector_axn/ vector_alm);                     //追踪线与车辆夹角，注意此反正弦舍弃，范围正负90
   // err_angle = err_angle*180/3.1415926;

     float err_angle2 = 0-atan2( vector_axn, vector_a_n);                     //追踪线与车辆夹角，，范围正负180
    err_angle2 = err_angle2*180/3.1415926;

//  std::cout << " vector_axn:  " << vector_axn << "  "<< "   vector_a_n:  " << vector_a_n << "  "<< " car_nx:  " << car_nx << "  "<< "   car_ny:  " << car_ny << "  "
 //        << "vector_alm:  "  << vector_alm  << "  err_angle:  "  << err_angle  << "  err_angle2:  "  << err_angle2 << std::endl;

    float star_end = sqrt( (star_point[0] - end_point[0])* (star_point[0] - end_point[0]) + (star_point[1] - end_point[1])* (star_point[1] - end_point[1]) );     // 路径长度，为调用函数判断使用
    float remainder_distance = sqrt( (cur_pose[0] - end_point[0])* (cur_pose[0] - end_point[0]) + (cur_pose[1] - end_point[1])* (cur_pose[1] - end_point[1]) );     // 车辆距离终点长度，为调用函数判断使用
    float driving_distance = sqrt( (cur_pose[0] - star_point[0])* (cur_pose[0] - star_point[0]) + (cur_pose[1] - star_point[1])* (cur_pose[1] - star_point[1]) );     // 车辆在AB路径已经行使的距离，为调用函数判断使用


  printf("star_end[0] %f  \n", star_end);   
    printf("driving_distance[1] %f  \n", driving_distance);  

    float vtx =0;
    float w = 0;
    if ( dirflag==true )
    {
        vtx = err_distance * p_con[0];     //计算x方向平移速度分量
        w = err_angle2 * p_con[1] * ( track + wheelbase) / 2.0;     //角速度分量
        //限幅
        if (vtx >= 400)
        {
        vtx = 400;
        }
        else  if (vtx <= -400)
        {
        vtx = -400;
        }

        if (w >= 400)
        {
        w = 400;
        }
        else  if (w <= -400)
        {
        w = -400;
        }

    }
    else
    {
        vtx = vty;
        vty = 0 - err_distance * p_con[0];     //计算y方向平移速度分量
        w = (err_angle2+90 )* p_con[1] * ( track + wheelbase) / 2.0;     //角速度分量
        //限幅
        if (w >= 400)
        {
        w = 400;
        }
        else  if (w <= -400)
        {
        w = -400;
        }
            if (vty >= 400)
        {
        vty = 400;
        }
        else  if (vty <= -400)
        {
        vty = -400;
        }

    }


    by_robot.lfrontwheelspeed = vty + vtx  -  w;     //左前轮速度
    by_robot.rfrontwheelspeed = vty - vtx  + w;     //右前轮速度
    by_robot.lbackwheelspeed = vty -  vtx  -  w;     //左后轮速度
    by_robot.rbackwheelspeed = vty + vtx  + w;     //右后轮速度
    by_robot.targ_distance = remainder_distance; //车辆距离终点长度
    by_robot.driv_distance = driving_distance;          // 车辆在AB路径已经行使的距离
    by_robot.starend_distance = star_end;               //AB线路径长度

     std::cout << " vty:  " << vty << "  "
         << "vtx:  "  << vtx << "  "
          << "w :  " << w   << " err_distance :  " << err_distance  << " err_angle2 :  " << err_angle2 << " driving_distance :  " << driving_distance  << " star_end :  " << star_end<< std::endl;
 
   return by_robot;
}

//---------------------------------------------------------------------------------------------------------------------
char mid[4]  ;
char *int_to_char(int n)
{
	mid[0] = (char) (n & 0xff);
	mid[1] = (char) (n >> 8 & 0xff);
	mid[2] = (char) (n >> 16 & 0xff);
	mid[3] = (char) (n >> 24 & 0xff);
	return mid;
}

//---------------------------------------------------------------------------------------------------------------------
uint8_t BCC_CHECK2(uint8_t *aubData_p, uint8_t auwDataLength)
{
    uint8_t aubChecksum = 0;
    uint8_t auwCnt = 2;
 
    while(auwCnt < auwDataLength)
    {
        aubChecksum ^= aubData_p[auwCnt];
        auwCnt++;
    }
 
    return aubChecksum;
}

//---------------------------------------------------------------------------------------------------------------------

void tx_locdata(double xxx , double yyy ,double angle, uint8_t zhuangtai , uint8_t loadmappcess )
{
    int tx_angle = (int)(angle*1000);
   int  X = (int)(xxx*1000);
   int  Y = (int)(yyy*1000);

   output_handle[4] = loadmap_data;

	XX = int_to_char(X);
    output_handle[5] = XX[3];
    output_handle[6] = XX[2];
    output_handle[7] = XX[1];
    output_handle[8] = XX[0];
            
    YY = int_to_char(Y);
    output_handle[9] = YY[3];
    output_handle[10] = YY[2];
    output_handle[11] = YY[1];
    output_handle[12] = YY[0];
                    
    ang = int_to_char(tx_angle);
    output_handle[13] = ang[3];
    output_handle[14] = ang[2];
    output_handle[15] = ang[1];
    output_handle[16] = ang[0];       

    //state =0xf5;
    output_handle[17] = 0;                       
    output_handle[18] = BCC_CHECK2(output_handle,17);
    output_handle[19] = 0xee;
    output_handle[20] = 0xee;
                    
    for(char i=0;i<21;i++)
    {
        output_date[i] = output_handle[i];
    }

      write(fd,output_date, 21);

}

//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------
float vty2 = -500;
float vty1 = 0;
uint8_t runflag = 0;
uint8_t pathflag = 0;
uint8_t stopflag = 0;
//  分别x,y,速度，方向，左右前进选择方向《0左右
// double pathpoint[6][5]={
//     {10.4235,3.4225,-500,1,-1},
//     {27.5526,3.41965,-1500,1,1},
//      {27.59,2.45,-500,1,-1},
//     {10.457,2.46858,-1200,-1,1},
//     {10.4235,3.4225,-500,-1,-1},
//      {10.4235,3.4225,0,1,1}
// };

// double pathpoint[6][5]= {
//     {9.515,3.402,-800,1,-1},
//     {16.562,3.420,-2000,1,1},
//      {16.612,0.442,-800,1,-1},
//     {9.617,0.442,-2000,-1,1},
//     {9.515,3.402,-800,-1,-1},
//      {9.515,3.402,0,1,1}
// };

double pathpoint[6][5]= {
    {-13.50,-17.72,-800,1,-1},
    {-6.98,-17.69,-2000,1,1},
     {-6.94,-21.06,-800,1,-1},
    {-13.50,-21.20,-2000,-1,1},
    {-13.50,-17.72,-800,-1,-1},
     {-13.50,-17.72,0,1,1}
};

double  pathpoint2[1000][9]={0};
double secs11 =0, secs22 = 0; 
void outputdata(const geometry_msgs::PoseStamped::ConstPtr& laserOdometry)
{
      // currentHeader = laserOdometry->header;
      //  timeLaserOdometry = laserOdometry->header.stamp.toSec();
        double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat = laserOdometry->pose.orientation;
        //tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);
        tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);
         double xxx, yyy, zzz,heading;
 //        yyy = laserOdometry->pose.pose.position.x;
 //        zzz = laserOdometry->pose.pose.position.y;
  //       xxx = laserOdometry->pose.pose.position.z;    

         xxx = laserOdometry->pose.position.x;
         yyy = laserOdometry->pose.position.y;
         zzz = laserOdometry->pose.position.z; 


         heading  = -yaw*180/3.1415926;

         if ( heading >= 0)
         {
            heading = heading;
         }
         else
           heading = 360 + heading;

         std::cout << "  xxx:  " << xxx << "  "
            << "yyy:  " << yyy << "  "
             << "zzz:  " << zzz << "  "
             << "heading:  " << heading  << "yaw:  " << yaw << std::endl;

        double cur_pose2 [3] = { xxx ,yyy, heading };

        float track2 =1.7;
        float wheelbase2 = 1.97;
         //float vty2 = -500;
        float p_con2[2] = {1000,50};
        struct mcnamu_robot by_robotcur;
       std_msgs::UInt32MultiArray  punavidata;

 	secs11 =ros::Time::now().toSec();

	double secscc  = fabs( secs11-secs22);
	if ( secscc>= pathpoint2[pathflag][8] )
	{
	    starcarmod=1;
	}

	//printf("secscc %f  \n", secscc);    

        if (pathleng<9)
        {   printf("pathleng %d  \n", pathleng);      
            return;
        }
        
         
        uint16_t pathnums = Path_Point.data[0];
        uint16_t pointnum =  pathleng/9;
       // printf("pointnum %d  \n", pointnum); 

        if (pathupflag == 1)
        {
            pathupflag = 0;
            for (size_t i = 0; i < pathleng/9; i++)
            {
                for (size_t j = 0; j < 9; j++)
                {
                    pathpoint2[i][j] = Path_Point.data[ i*9+j+1 ];
                    printf("  %f  ", pathpoint2[i][j]);  
                }
                  printf("  \n");  
            }
             Path_Point.data.clear();
        }
        
	printf("starcar %d  \n", starcar); 
	printf("starcarend %d  \n", starcarend); 
	printf("starcarmod %d  \n", starcarmod); 
        if (starcar==0 || starcarend==0|| starcarmod == 0)
        {
            punavidata.data.push_back(0);
            punavidata.data.push_back(0);
            punavidata.data.push_back(0);
            punavidata.data.push_back(0);
            pubnavigdata.publish(punavidata);   
            return;
        }
        
        
        printf("runflag %d  \n", runflag);   
        printf("pathflag %d \n", pathflag);  

        if (runflag == pathflag)
        {
          
           //Path_Point.data[1*( pathflag+1 )]
          //Path_Point.data[1*( pathflag+1 )+1]
            double star_point1[2] = { pathpoint2[pathflag][0] ,pathpoint2[pathflag][1] };
            double end_point1[2] = { pathpoint2[pathflag+1][0] ,pathpoint2[pathflag+1][1] };

             bool lrdirrunflag = true;
            if (pathpoint2[pathflag+1][5] < 0)
            {
                 lrdirrunflag = false;
            }

            bool fbdirrunflag = true;
            if (pathpoint2[pathflag+1][6] < 0)
            {
                 fbdirrunflag = false;
            }
    
              printf("fbdirrunflag %d  \n", fbdirrunflag);   
              printf("lrdirrunflag %d \n", lrdirrunflag);  

             by_robotcur = mcnamu_navigation3( cur_pose2 ,star_point1 ,end_point1, track2 , wheelbase2, vty1, p_con2,lrdirrunflag,fbdirrunflag);
            
              printf("by_robotcur.lfrontwheelspeed %f \n", by_robotcur.lfrontwheelspeed);  

            punavidata.data.push_back(by_robotcur.lfrontwheelspeed);
            punavidata.data.push_back(by_robotcur.rfrontwheelspeed);
            punavidata.data.push_back(by_robotcur.lbackwheelspeed);
            punavidata.data.push_back(by_robotcur.rbackwheelspeed);
            pubnavigdata.publish(punavidata);   

            if (stopflag==1)
            {
               stopflag=0;
              // p_con2[0] = 0;
              //  p_con2[1] = 0;
                secs22 =ros::Time::now().toSec();
                starcarmod=0;
              // sleep( pathpoint2[pathflag][8] );
               //sleep( 2 );
            }   else
            {
                vty1 = pathpoint2[pathflag+1][4];
                //p_con2[0] = 400;
               // p_con2[1] = 15;
            }    
                        
             printf("by_robotcur.driv_distance %f  \n", by_robotcur.driv_distance);   
            printf("by_robotcur.starend_distance %f \n", by_robotcur.starend_distance);  

            if (lrdirrunflag == true)
            {
                 if (by_robotcur.driv_distance>by_robotcur.starend_distance-0.4 )
                {
                    vty1 = 0;
                    runflag = 1 + runflag;
                    pathflag = 1 + pathflag;
                    if (pathflag==pointnum-1)
                    {
                       runflag = 0;
                       pathflag = 0;
                       starcar=0;
                       starcarend=0;
                    }
                    
                    stopflag = 1;
                }
            }
            else
            {
                 if (by_robotcur.driv_distance>by_robotcur.starend_distance-0.2)
                {
                    vty1 = 0;
                    runflag = 1 + runflag;
                    pathflag = 1 + pathflag;
                    if (pathflag==pointnum-1)
                    {
                       runflag = 0;
                      pathflag = 0;
                       starcar=0;
                       starcarend=0;
                    }

                    stopflag = 1;
                }
            }
        }
        else
        {
            punavidata.data.push_back(0);
            punavidata.data.push_back(0);
            punavidata.data.push_back(0);
            punavidata.data.push_back(0);
            pubnavigdata.publish(punavidata);   
        }
        
        
  
 }

//---------------------------------------------------------------------------------------------------------------------
void arer_dateHandler(const  std_msgs::UInt8MultiArray &paraMsg)
{
    lidar_date = paraMsg.data[0];

}
//---------------------------------------------------------------------------------------------------------------------
void erry_mapHandler(const std_msgs::UInt8MultiArray &paraMsg)
{
    erry_map_data = paraMsg.data[0];
}
//---------------------------------------------------------------------------------------------------------------------
void loadmapHandler(const std_msgs::UInt8MultiArray &paraMsg)
{
   loadmap_data = paraMsg.data[0];
}
//---------------------------------------------------------------------------------------------------------------------
void pathpointeHandler(const std_msgs::Float64MultiArray &paraMsg)
{
    Path_Point.data.clear();
    Path_Point.data =  paraMsg.data;
    pathleng = paraMsg.data.size();
    pathupflag = 1;
    printf("pathleng %d  \n", pathleng);   
    printf("Path_Point.data[0]  %f  \n", Path_Point.data[0] );   
    printf("Path_Point.data[pathleng-1]  %f  \n", Path_Point.data[pathleng-1] );   
}
//---------------------------------------------------------------------------------------------------------------------
void carrun_numHandler(const std_msgs::UInt32 &paraMsg)
{
   runcarnum = paraMsg.data;
   
   printf("runcarnum %d  \n", runcarnum);  
}
//---------------------------------------------------------------------------------------------------------------------
void carrun_starHandler(const std_msgs::UInt32 &paraMsg)
{
     starcar = paraMsg.data;
     starcarend = starcar;
   starcarmod=starcar;
     printf("starca1r %d  \n", starcar);  
}
//---------------------------------------------------------------------------------------------------------------------
void carrun_safHandler(const std_msgs::UInt8MultiArray &paraMsg)
{
     //starcar = paraMsg.data[0];
     if (car_war==1)
     {
        // starcar = 0;
         return;
     }
     
     if (paraMsg.data[0]==1 || paraMsg.data[1]==1 || paraMsg.data[2]==1 || paraMsg.data[3]==1 || paraMsg.data[4]==1 || paraMsg.data[5]==1 )
     {
         starcar = 0;
     }
     else 
     { 
         starcar = 1;
     }
     
     //printf("starcar2 %d  \n", starcar);  
}

//---------------------------------------------------------------------------------------------------------------------
void carrun_warHandler(const std_msgs::UInt32 &paraMsg)
{
     car_war = paraMsg.data;
     printf("car_war %d  \n", car_war);  
}
//---------------------------------------------------------------------------------------------------------------------

int main(int argc, char** argv)
{
    ros::init(argc, argv, "macaimwheelnavigation");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle n;

   // ros::Publisher tcp_jie =n.advertise<navgation_pack::androidyaokong>("/android_para", 100);

    pubchongdingwei = n.advertise<std_msgs::UInt8MultiArray>("/chongdingwei", 100);
    ros::Subscriber  subLaserOdometry = n.subscribe<geometry_msgs::PoseStamped>("/ndt_pose", 5,  outputdata);
    //ros::Subscriber  subLaserOdometry = n.subscribe<nav_msgs::Odometry>("/integrated_to_init", 5,  outputdata);
    ros::Subscriber arer_date = n.subscribe("/lidardate_para", 1, arer_dateHandler);
    pubnavigdata = n.advertise<std_msgs::UInt32MultiArray>("/navigdata", 100);
    ros::Subscriber path_point = n.subscribe("/pathpoint", 1, pathpointeHandler);
    ros::Subscriber carrun_num = n.subscribe("/car_run_num", 1, carrun_numHandler);
    ros::Subscriber carrun_star = n.subscribe("/car_run_star", 1, carrun_starHandler);
      ros::Subscriber carrun_saf = n.subscribe("/safety_data", 1, carrun_safHandler);

      ros::Subscriber carrun_war = n.subscribe("/car_waring", 1, carrun_warHandler);

    ros::Subscriber  erry_Map = n.subscribe("/erry_map", 10, erry_mapHandler);
    ros::Subscriber  loadMap = n.subscribe("/loadmap", 10, loadmapHandler);
//pubpathpoint = n.advertise<std_msgs::Float64MultiArray>("/pathpoint", 100);

    ros::Rate loop_rate(40);
   while (ros::ok()) 
   {
   
                  
                ros::spinOnce(); 
                loop_rate.sleep(); 
     }
        
    return 0;
}
