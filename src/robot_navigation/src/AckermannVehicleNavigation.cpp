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
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include "std_msgs/UInt8MultiArray.h"

#include "std_msgs/Int16MultiArray.h"
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

float  erry_map_data2=0;
int16_t  erry_map_cut=20;
 
double current_xx = 0;
double current_yy = 0;
double current_heading = 0;
double star_site[2] = { 0 };
double end_site[2] = { 0 };
double current_pose[3] = { 0 };
 int pathleng = 1;
std_msgs::Float64MultiArray Path_Point;
std_msgs::Float64MultiArray LastPath_Point;

uint16_t pathupflag = 0;
uint32_t starcar=0,runcarnum=0,starcarend=0,car_war=0,starcarmod=1,runcarstar=0;
uint32_t EndTask=1;//到达目的地为1
double _starAX=0 ,_starAY=0;
double _endBX=0, _endBY=0;
double _endP1X=0, _endP1Y=0;
double _endP2X=0, _endP2Y=0;
float _spleed=0;
float _AreaSele;
uint8_t ExcuteActiondata=0;

uint8_t starVe=0,direcVe=0;
uint8_t backCarflag = 0;

uint8_t CloseRoute = 0;
//struct mcnamu_robot by_robot = {0,0,0,0,false,false,false,false, 0, true };
std_msgs::UInt8 pubdataCarStu;


ros::Publisher pubVehStatus;
ros::Publisher pubnavigdata;
ros::Publisher pubAreasele;

ros::Publisher pubctrl_init;
ros::Publisher pubEndTaskID;
ros::Publisher pubTargetTaskID;
ros::Publisher pubDout7;
ros::Publisher pubDout8;
ros::Publisher pubDout1;
ros::Publisher pubDout2;
ros::Publisher pubDout3;
ros::Publisher pubDout4;
ros::Publisher pubDout5;
ros::Publisher pubCarStu;
ros::Publisher publoc_stau;

//-------------------------------------------------------------------------
typedef union
{
    uint16_t DataIn;
    uint8_t DataIN[2];
    struct
    {
        /* Byte Segment 0 */
        uint8_t DI1           : 1;
        uint8_t DI2           : 1;
        uint8_t DI3           : 1;
        uint8_t DI4           : 1;
        uint8_t DI5           : 1;
        uint8_t DI6           : 1;
        uint8_t DI7           : 1;
        uint8_t DI8           : 1;
        uint8_t DI9           : 1;//1#避障雷达信号1
        uint8_t DI10          : 1;//1#避障雷达信号2
        uint8_t DI11          : 1;//1#避障雷达信号3
        uint8_t DI12           : 1;//2#避障雷达信号1
        uint8_t DI13           : 1;//2#避障雷达信号2
        uint8_t DI14           : 1;//2#避障雷达信号3
        uint8_t DI15           : 1;
        uint8_t DI16           : 1;
    } bit;
} unionObstacle;

unionObstacle ObsSignal;
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
    // float vector_alm1 =  sqrt(vector_ax*vector_ax + vector_ay*vector_ay );   //AB路径a向量模长
     //float vector_alm2 =  sqrt(vector_bx*vector_bx + vector_by*vector_by );   //AB路径a向量模长
    //float err_angle = asin(vector_axn/ vector_alm);                     //追踪线与车辆夹角，注意此反正弦舍弃，范围正负90
    // err_angle = err_angle*180/3.1415926;
    float err_angle = atan2( vector_axn, vector_a_n);                     //追踪线与车辆夹角，，范围正负180
    //float err_angle = asin( vector_axn/(vector_alm1*vector_alm2));                     //追踪线与车辆夹角，，范围正负180
    vectoranle = err_angle;
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
//计算圆与直线的交点--------------------------------------------------------------------------------------------------------------------------------------

struct Point {
    double x;
    double y;
};

struct Resultspoint {
	uint8_t NumResu;
    Point res1;
    Point res2;
};

struct VehicleControl {
	float angle;
    float spleed;
    float erangle;
};


struct Posture {
    Point pos;
    float yaw;
};

// 计算两点之间的距离
double distance(const Point& p1, const Point& p2) {
    return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
}
// 求圆与线段的交点
Resultspoint findIntersection(const Point& center, double radius, const Point& startPoint, const Point& endPoint) {
    Resultspoint results;
	// 计算线段长度
    double segmentLength = distance(startPoint, endPoint);
    
    // 计算线段向量
    double dx = (endPoint.x - startPoint.x) / segmentLength;
    double dy = (endPoint.y - startPoint.y) / segmentLength;
    
    // 计算圆心到起点的向量
    double ex = startPoint.x - center.x;
    double ey = startPoint.y - center.y;
    
    // 计算直线方程参数A、B、C
    double A = dx * dx + dy * dy;
    double B = 2 * (dx * ex + dy * ey);
    double C = ex * ex + ey * ey - radius * radius;
  	// 计算判别式D，用于判断是否有交点
  	double D = B*B-4*A*C;
  	if(D<0){
		results.NumResu = 0;
    	// 判别式小于0，说明没有交点，输出提示信息即可
    	std::cout << "No intersection point found!" << std::endl;
  	}else if(D==0){
		results.NumResu = 1;
    	// 判别式等于0，说明有一个交点
    	double t = -B / (2*A);
      	double intersectionX = startPoint.x + t * dx;
      	double intersectionY = startPoint.y + t * dy;

		results.res1.x = intersectionX;
		results.res1.y = intersectionY;
      
     	std::cout << "Intersection point: (" << intersectionX << ", " << intersectionY << ")" << std::endl;
  	}else{
    	// 判别式大于0，说明有两个交点
    	double t1 = (-B+std::sqrt(D)) / (2*A);
        double intersectionX1 = startPoint.x + t1 * dx;
        double intersectionY1 = startPoint.y + t1 * dy;
        double t2 = (-B-std::sqrt(D)) / (2*A);
        double intersectionX2 = startPoint.x + t2 * dx;
        double intersectionY2 = startPoint.y + t2 * dy;

		results.NumResu = 2;
		results.res1.x = intersectionX1;
		results.res1.y = intersectionY1;
		results.res2.x = intersectionX2;
		results.res2.y = intersectionY2;

      	std::cout << "Intersection points: (" << intersectionX1 << ", " << intersectionY1 << ") and (" 
                  << intersectionX2<<", "<<intersectionY2<<")"<<std::endl;
  	}
	return results;
}
//向量计算预瞄点(P为垂足点，Q为预瞄点)-------------------------------------------------------------------------------------------------------------------------
// 计算向量的点积
double dotProduct(Point v1, Point v2) {
    return v1.x * v2.x + v1.y * v2.y;
}
// 计算向量的长度
double vectorLength(Point v) {
    return std::sqrt(v.x * v.x + v.y * v.y);
}
// 计算点P的坐标
Point calculateP(Point A, Point B, Point C) {
    Point AB, AC, AP;
    
    AB.x = B.x - A.x;
    AB.y = B.y - A.y;
    
    AC.x = C.x - A.x;
    AC.y = C.y - A.y;
    
    double AB_length = vectorLength(AB);
     std::cout<< "AB_length:" <<AB_length<< " AB.x:" << AB.x<< " AB.y:"  << AB.y << "  AC.x  : "<<   AC.x    << " AC.y: "<<AC.y  << std::endl;
    
    double projectionFactor = dotProduct(AB, AC) / (AB_length * AB_length);//比例因子，可用于停车判断，大于1可以认为停车。
    
    AP.x = A.x + AB.x * projectionFactor;
    AP.y = A.y + AB.y * projectionFactor;

    std::cout<< "dotProduct(AB, AC):" <<dotProduct(AB, AC)<< " AB_length:" <<AB_length<< " projectionFactor:"  << projectionFactor << "  AP.x : "<<   AP.x   << " AP.y: "<<AP.y  << std::endl;
    
    return AP;
}
// 根据点P和距离L计算点Q的坐标
Point calculateQ(Point P, Point B, double L) {
    Point PB;
    
    PB.x = B.x - P.x;
    PB.y = B.y - P.y;
    
    double PB_length = vectorLength(PB);
    
    double factor = L / PB_length;
    
    Point AQ;
    
    AQ.x = P.x + PB.x * factor;
    AQ.y = P.y + PB.y * factor;
       std::cout<<"factor :"<<factor<<" PB_length:" <<PB_length<< "  AQ.x : "<<   AQ.x   << " AQ.y: "<<AQ.y  << std::endl;
    
    return AQ;
}


//路径追踪控制------------------------------------------------------------------------------------------------------------------------------
//const Point& starP  路径起点
//const Point& endP 路径终点
//const Point& TargetP  追踪的目标点，预瞄圆与路径线交点
//const Point& curP			车辆当前坐标
 VehicleControl PathTrackingControl2 ( const Point& starP ,const Point& endP ,const Point& TargetP ,const Posture& curP, float forwardDis ,float wheelbase , float speed)//预瞄圆方式计算
 {
	VehicleControl CtrlData;
    float car_nx = cos( curP.yaw );                              //车辆单位向量x
    float car_ny = sin( curP.yaw );                              //车辆单位向量y

    float PCx =  TargetP.x-curP.pos.x;                              //预瞄向量x
    float PCy =  TargetP.y-curP.pos.y;                              //预瞄向量y

    std::cout << "car_nx: "<< car_nx   <<" car_ny:  "<<  car_ny  << std::endl;
    std::cout << "PCx: "<< PCx   <<" PCy:  "<<  PCy  << std::endl;

    float tackangle_A1 = VectorAngle(PCx ,PCy ,car_nx ,car_ny);
    std::cout << "tackangle_Aq1: "<< tackangle_A1 *180 / 3.14    << std::endl;
    /**车体运行方向**/
    backCarflag = 0;
    if(speed < 0)//tackangle_A1 < -1.5708 || tackangle_A1 > 1.5708 )
    {
    	car_nx =  car_nx; 
    	car_ny =  car_ny; 
    	tackangle_A1 = VectorAngle(PCx ,PCy ,car_nx ,car_ny);
    	//speed = -speed;
    	backCarflag = 1;
    }
    
	//float tackangle_A1=  atan2(  TargetP.y-curP.pos.y ,  TargetP.x-curP.pos.x ); //计算车体与预瞄线角度
	float tackangle_A = 0;
	//if (tackangle_A1 - curP.yaw > -90 && tackangle_A1 -  curP.yaw  < 90)
	//{
	tackangle_A = -tackangle_A1 ;
	//}
	float tackangle_A2 =atan( 2 * wheelbase * sin(tackangle_A) / forwardDis);
 
     float err_offset = -2;
     if(speed < 0)
     {
     err_offset = 1.5;
     }
 
    tackangle_A2 = err_offset + tackangle_A2 *180 / 3.14;
    std::cout << "tackangle_A1: "<< tackangle_A1 *180 / 3.14  <<" tackangle_A2:  "<<  tackangle_A2  << std::endl;

	CtrlData.angle =LIMIT( -60 , tackangle_A2 ,60) ;
 
	CtrlData.spleed = speed;
    std::cout << "CtrlData.spleed: "<< CtrlData.spleed  << " CtrlData.angle: "<<CtrlData.angle  << std::endl;
    return CtrlData;
 }
//路径追踪控制------------------------------------------------------------------------------------------------------------------------------
//const Point& starP  最后一段路径起点
//const Point& endP 最后一段路径终点
//const Posture& TargetP  停车位点
//const Point& curP			车辆当前坐标
//float pp ,比例增益系数
//float speed最大转弯速度

float lodyaw = 0.0;
uint32_t count_err = 0;
 VehicleControl ReverseParking (const Posture& TargetP ,const Posture& curP, float Kp , float speed)//预瞄圆方式计算
 {
	 VehicleControl CtrlData;

     if(lodyaw == TargetP.yaw)
     {
        count_err ++;
     }
     else
     {
        count_err = 0;
     }

     if (count_err >20)
     {
        count_err = 20;
     }
     
     lodyaw = TargetP.yaw;

	 float Targetnx = cos( TargetP.yaw );                              //目标航向单位向量x
     float Targetny = sin( TargetP.yaw );                              //目标航向单位向量y

     float PSEx =  cos( curP.yaw );                              //最后一段路径向量x
     float PSEy =  sin( curP.yaw );                              //最后一段路径向量y

     std::cout << "Targetnx: "<< Targetnx   <<" Targetny:  "<<  Targetny  << std::endl;
     std::cout << "PSEx: "<< PSEx   <<" PSEy:  "<<  PSEy  << std::endl;
     std::cout << "TargetP.yaw: "<< TargetP.yaw   <<" curP.yaw:  "<<  curP.yaw  << std::endl;

     float tackangle_A1 =VectorAngle(PSEx ,PSEy ,Targetnx ,Targetny);// TargetP.yaw - curP.yaw;//VectorAngle(PSEx ,PSEx ,Targetnx ,Targetny);//(-PI,PI) 
     float tackangle_A3 = TargetP.yaw - curP.yaw;
     std::cout << "tackangle_Aq1: "<< tackangle_A1 *180 / 3.14    << ":a3" << tackangle_A3 *180 / 3.14 << std::endl;
     CtrlData.erangle = tackangle_A1 *180 / 3.14;
     /*
     if(CtrlData.erangle < 1.0)
     {
        count_err = 0;
     }
    */
     CtrlData.angle =90;
     CtrlData.spleed = 0;
     if (count_err == 20)
     {
        CtrlData.spleed = LIMIT( -speed , tackangle_A1*Kp ,speed) ;
     }
     
     return CtrlData;
     }
//路径追踪控制------------------------------------------------------------------------------------------------------------------------------
//const Point& starP  路径起点
//const Point& endP 路径终点
//const Point& TargetP  追踪的目标点，预瞄圆与路径线交点
//const Point& curP			车辆当前坐标
 VehicleControl PathTrackingControl ( const Point& starP ,const Point& endP ,const Resultspoint& TargetP ,const Posture& curP, float forwardDis ,float wheelbase , float speed)//预瞄圆方式计算
 {
	VehicleControl CtrlData;
	if (TargetP.NumResu == 2)
	{
		float tackangle_A1=  atan2(  TargetP.res1.y-curP.pos.y ,  TargetP.res1.x-curP.pos.x ); //计算车体与预瞄线角度
		float tackangle_A2 =  atan2(  TargetP.res2.y-curP.pos.y ,  TargetP.res2.x-curP.pos.x ); //计算车体与预瞄线角度
		float tackangle_A = 0;
		if (tackangle_A1 - curP.yaw > -90 && tackangle_A1 -  curP.yaw  < 90)
		{
			tackangle_A = tackangle_A1 - curP.yaw ;
		}
		else
		{
			tackangle_A = tackangle_A2  - curP.yaw ;
		}
		
		CtrlData.angle =LIMIT( -30 ,atan( 2 * wheelbase * sin(tackangle_A) / forwardDis) ,30) ;

		CtrlData.spleed = speed;
	}
	else{
		CtrlData.angle = 0;
		CtrlData.spleed = 0;
	}
 }

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++2as
//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++2as
//---------------------------------------------------------------------------------------------------------------------
//float dSC:距离目标点实际的位移
//float dS：距离目标点开始减速的位移
//float dSmin：距离目标点最小匀速距离
//float spleed:车辆原始速度
//---------------------------------------------------------------------------------------------------------------------
float spleedctrl(float dSC ,float dS ,float dSmin ,float spleed)
{	
	float spleedout = spleed *( dSC / dS );

    if (spleedout < 0.1 )
    {
       spleedout = 0.1;
    }
    
    if ( dSC < dSmin )
    {
        spleedout = 0.1;
    }
    
    return spleedout;
	
}

//---------------------------------------------------------------------------------------------------------------------
//float dSC:距离目标点实际的位移
//float Dec：减速度
//float dSmin：距离目标点最小停车距离
//float spd:车辆原始速度
//float spdmin:车辆原始速度
//---------------------------------------------------------------------------------------------------------------------
float spleedctrl2(float dSC ,float Dec ,float dSmin ,float spd, float spdmin)
{	
	float spdout = 0;
    float dSst = spd * spd /( 2 * Dec );

    if (spd > 0)
    {
        if (dSC > dSst )
        {
        spdout = spd;
        }
        else if (dSC <= dSst)
        {
            spdout = sqrt( 2* Dec *dSC);
            if ( spdout <= spdmin )
            {
                spdout = spdmin;
            }
            
        }
    }
    else
    {
        if (dSC > dSst )
        {
        spdout = spd;
        }
        else if (dSC <= dSst)
        {
            spdout = 0 -sqrt( 2* Dec *dSC);
            if ( spdout >= (0-spdmin) )
            {
                spdout = 0 - spdmin;
            }
            
        }
    }
    
    if (dSC < dSmin )
    {
       spdout = 0.0;
    }

    return spdout;	
}

//-------------------------------------------------------------------------------------------------------------------------



uint8_t runflag = 0;
uint8_t pathflag = 0;
uint8_t stopflag = 0;


double pathpoint[6][5]= {
    {-13.50,-17.72,-800,1,-1},
    {-6.98,-17.69,-2000,1,1},
     {-6.94,-21.06,-800,1,-1},
    {-13.50,-21.20,-2000,-1,1},
    {-13.50,-17.72,-800,-1,-1},
     {-13.50,-17.72,0,1,1}
};

Point starA;
Point endB;
Point HstarA;
Point HendB;
Posture curC;
Posture tagA;
//starpointA.x = -2.94119;
VehicleControl Ctrldata;
double  pathpoint2[1000][9]={0};
double secs11 =0, secs22 = 0; 
int cunntt =0;
float spleedtak = 0,spleedcoef=1;
double locX=0,locY=0;

double laslocX=0,laslocY=0;
uint8_t initflg = 0;
uint8_t loc_statu = 0;
uint8_t rotation_flg = 0;
uint8_t last_pathflag = 0;

double delay_star = 0, delay_end = 0;

void outputdata(const geometry_msgs::PoseStamped::ConstPtr& laserOdometry)
{
    // currentHeader = laserOdometry->header;
    //  timeLaserOdometry = laserOdometry->header.stamp.toSec();
    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = laserOdometry->pose.orientation;
    //tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);
    tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);
    double xxx, yyy, zzz,heading;
    //  yyy = laserOdometry->pose.pose.position.x;
    //  zzz = laserOdometry->pose.pose.position.y;
    // xxx = laserOdometry->pose.pose.position.z;    

    xxx = laserOdometry->pose.position.x;
    yyy = laserOdometry->pose.position.y;
    zzz = laserOdometry->pose.position.z; 
    locX = xxx;
    locY = yyy;

    curC.yaw =  yaw;
    heading  = yaw*180/3.1415926;

    if ( heading >= 0)
    {
        heading = heading;
    }
    else
    heading = 360 + heading;
    ROS_INFO_STREAM("[car xxx : " << xxx<< "] [car yyy : " << yyy <<"] [car heading : " << heading  <<"] [car yaw : " << yaw  << "]");
    
     float errdis = sqrt( (  locX - laslocX )* (  locX - laslocX) + ( locY - laslocY )* ( locY - laslocY ) );  
    laslocX = locX;
    laslocY = locY;
    if (errdis > 0.15)
    {
        spleedcoef = 0;
        loc_statu = 1;
    }
    if (ObsSignal.bit.DI3 == 1)
    {
        spleedcoef = 1;
        loc_statu = 0;
    }
     std_msgs::UInt8 loc_st;
     loc_st.data = loc_statu;//填充IO out数据
	 publoc_stau.publish(loc_st);
/*
    std::cout << "  xxx:  " << xxx << "  "
    << "yyy:  " << yyy << "  "
        << "zzz:  " << zzz << "  "
        << "heading:  " << heading  << "yaw:  " << yaw << std::endl;
*/
    if(ExcuteActiondata == 0)
    {
        pubdataCarStu.data = 1;
    }
    curC.pos.x = xxx;
    curC.pos.y = yyy;
    float speedVs = 0;

    secs11 =ros::Time::now().toSec();

    double secscc  = fabs( secs11-secs22);
    if ( secscc>= pathpoint2[pathflag][8] )
    {
        starcarmod=1;
    }
	
	std_msgs::Int16MultiArray  punavidata;
    //printf("secscc %f  \n", secscc);    
    if (pathleng<12)
    {   
        punavidata.data.push_back(0);
        punavidata.data.push_back(0);
        punavidata.data.push_back(1);
        punavidata.data.push_back(ExcuteActiondata);//新增升降标志1上升 2下降 0车辆运行中，标志修改在回调函数ExcuteActionHandle()
        pubnavigdata.publish(punavidata);  
        printf("pathleng %d  \n", pathleng);      
        return;
    }

    uint16_t pathnums = Path_Point.data[0];
    uint16_t pointnum =  pathleng/9;
    // printf("pointnum %d  \n", pointnum); 

    if (pathupflag == 1)
    {
        pathupflag = 0;
        //starcarend = 1;
        for (size_t i = 0; i < pathleng/9; i++)
        {
            for (size_t j = 0; j < 9; j++)
            {
                pathpoint2[i][j] = Path_Point.data[ i*9+j+1 ];
                printf("  %f  ", pathpoint2[i][j]);  
            }
                printf("  \n");  
        }
            //Path_Point.data.clear();
    }
    
    if(initflg == 0)
    {
		if(pathpoint2[1][3] == 3 || pathpoint2[1][3] == 4)
		{
		  std_msgs::UInt8 initdata;
		  
		  if(pathpoint2[1][1] < pathpoint2[2][1])
		  {
		  	 
			 initdata.data = 1;//填充IO out数据
			 //pubctrl_init.publish(initdata);
			 initflg = 1;
		  }
		  else
		  {
		  	 initdata.data = 2;//填充IO out数据
			 //pubctrl_init.publish(initdata);
			 initflg = 1;
		  }
		  
		  printf("pathpoint2[1][1] %f  \n", pathpoint2[1][1]); 
		  printf("pathpoint2[2][1] %f  \n", pathpoint2[2][1]);
		
		}
	}	

    printf("starcar %d  \n", starcar); 
		printf("starcarend %d  \n", starcarend); 
		printf("starcarmod %d  \n", starcarmod); 
		
		std_msgs::UInt32MultiArray  pubTargetTaskdata;
		pubTargetTaskdata.data.push_back(pathpoint2[pathflag+2][3]);
		pubTargetTaskID.publish(pubTargetTaskdata);

    

    if (starcarend==0|| starcarmod == 0)
    {
        punavidata.data.push_back(0);
        punavidata.data.push_back(0);
        punavidata.data.push_back(1);
        punavidata.data.push_back(ExcuteActiondata);//新增升降标志1上升 2下降 0车辆运行中，标志修改在回调函数ExcuteActionHandle()
        pubnavigdata.publish(punavidata);   
        return;
    }

    ExcuteActiondata = 0;
    printf("runflag %d  \n", runflag);   
    printf("pathflag %d \n", pathflag); 
    pubdataCarStu.data = 2; 

    if (runflag == pathflag)
    {
        
        //Path_Point.data[1*( pathflag+1 )]
        //Path_Point.data[1*( pathflag+1 )+1]
        starA.x =pathpoint2[pathflag][0];
        starA.y = pathpoint2[pathflag][1];

        endB.x =  pathpoint2[pathflag+1][0];
        endB.y =  pathpoint2[pathflag+1][1];
        //double star_point1[2] = { pathpoint2[pathflag][0] ,pathpoint2[pathflag][1] };
        //double end_point1[2] = { pathpoint2[pathflag+1][0] ,pathpoint2[pathflag+1][1] };
        std::cout << "  starA.x:  " << starA.x << "  " << "starA.y:  " << starA.y << "  " << "  endB.x:  " << endB.x << "  " << "endB.y:  " << endB.y << std::endl;


        ROS_INFO("rotation_flg %d \n", rotation_flg); 
        last_pathflag = pathflag;
        bool lrdirrunflag = true;
        printf("pathpoint2[pathflag+1][5] %f \n", pathpoint2[pathflag+1][5]); 
        if (pathpoint2[pathflag+1][5] >= 5.9 && pathpoint2[pathflag+1][5] <= 6.1 && rotation_flg ==0)
        {
                lrdirrunflag = false;
                tagA.yaw = pathpoint2[pathflag+1][2];
                printf("tagA.yaw %f \n", tagA.yaw);  
        }
  
        
        ROS_INFO("lrdirrunflag %d \n", lrdirrunflag);  

          std_msgs::UInt8MultiArray vehstat;

        float star_end = sqrt( (  starA.x - endB.x )* (  starA.x - endB.x) + ( starA.y - endB.y )* ( starA.y - endB.y ) );     // 路径长度，为调用函数判断使用
        float remainder_distance = sqrt( ( curC.pos.x -  endB.x)* (curC.pos.x -  endB.x) + (curC.pos.y -  endB.y)* (curC.pos.y -  endB.y) );     // 车辆距离终点长度，为调用函数判断使用
        float driving_distance = sqrt( (curC.pos.x -  starA.x)* (curC.pos.x -  starA.x) + (curC.pos.y -  starA.y )* ( curC.pos.y -  starA.y ) );     // 车辆在AB路径已经行使的距离，为调用函数判断使用

            //calculateP(starA,endB,curC.pos);
        
        short flagend = 0;
        float spleedakm = 0;
        uint8_t starstatu=0;
        uint8_t dicrstatu=direcVe;

        // if (star_end < driving_distance )//||  starVe==0 )
        // {
        //     flagend = 1;
        //     starstatu = 0;
        //     spleedakm = 0;
        //     cunntt++;
        // }
        // else
        // {
        //     flagend = 0;
        //     starstatu = 1;
        //     spleedakm = speedVs;
        // }

        // vehstat.data.push_back(starstatu);
        // vehstat.data.push_back(dicrstatu);
        // pubVehStatus.publish( vehstat );

        if (stopflag==1)
        {
            stopflag=0;
            secs22 =ros::Time::now().toSec();
            starcarmod=0;
        }   else
        {
            spleedtak = pathpoint2[pathflag+1][4] ;
        }    
        
        
        /******************新增自转航向计算************************** */
        // lrdirrunflag车辆自转命令，false允许自转
        double TargetHead = 0;
        if (spleedtak < 0 && lrdirrunflag == false)
        {
            HstarA.x =pathpoint2[pathflag][0];
            HstarA.y = pathpoint2[pathflag][1];

            HendB.x =  pathpoint2[pathflag+1][0];
            HendB.y =  pathpoint2[pathflag+1][1];

            double DetX = HstarA.x - HendB.x;
            double DetY = HstarA.y - HendB.y;

            TargetHead = atan2(DetY,DetX);
            printf("TargetHead************ %f \n", TargetHead); 

            tagA.yaw = TargetHead;
        }
        /***************************航向自转结束************************************* */
        
                                
        printf("driving_distance %f  \n", driving_distance);   
        printf("star_end %f \n", star_end);  

        
         if(erry_map_cut < 0)
        {
            spleedtak = 0;
        }

        float ForwardDis = 1.5;
        if (spleedtak < 0)
        {
            ForwardDis = 0.7;
        }
        printf("ForwardDis %f \n", ForwardDis); 

        delay_end = ros::Time::now().toSec();
        if(fabs( delay_star-delay_end) < 2)
        {
            spleedtak = 0;
        }

        
        Point GG = calculateQ( calculateP(starA,endB,curC.pos),endB,ForwardDis );
        VehicleControl Ctrldata2 = PathTrackingControl2 ( starA ,endB  ,GG  ,curC, ForwardDis ,0.9 , spleedtak);//预瞄圆方式计算
        
        if(lrdirrunflag == false)
        {
        	Ctrldata2 = ReverseParking ( tagA ,curC, 1 , 0.3);//预瞄圆方式计算
        	if(fabs(Ctrldata2.erangle) < 1)
        	{
        	  //pathpoint2[pathflag+1][5] = -1.0;
            rotation_flg = 1;
            count_err = 0;
            delay_star = ros::Time::now().toSec();
        	}
        	
        }
        ROS_INFO("count_err %d \n", count_err);   
        
        float driving_Qdistance = sqrt( (curC.pos.x -  GG.x)* (curC.pos.x -  GG.x) + (curC.pos.y -  GG.y )* ( curC.pos.y -  GG.y ) ); 

        std::cout << "  GG.X:  " << GG.x << "  " << "GG.Y:  " << GG.y << "  " << "driving_Qdistance:  " << driving_Qdistance << std::endl;

        std::cout << "  star_end:  " << star_end << "  "<< "remainder_distance:  " << remainder_distance << "  "
        << "driving_distance:  " << driving_distance << "  "
        << "Ctrldata.angle:  " << Ctrldata2.angle  << " Ctrldata.spleed:  " << Ctrldata2.spleed << std::endl;

        std_msgs::UInt8 pubdataOutA;
        if ((int)pathpoint2[pathflag+1][7] == 0 )//pathflag+1 ==pointnum-1 )
        { 
            pubdataOutA.data = 0;//填充IO out数据
            pubDout3.publish(pubdataOutA);
            pubDout4.publish(pubdataOutA);
            pubDout5.publish(pubdataOutA);
        }
        else if ((int)pathpoint2[pathflag+1][7] == 1 )//pathflag+1 ==pointnum-1 )
        { 
            pubdataOutA.data = 1;//填充IO out数据
            pubDout3.publish(pubdataOutA);
            pubdataOutA.data = 0;//填充IO out数据
            pubDout4.publish(pubdataOutA);
            pubDout5.publish(pubdataOutA);
        }
        else if ((int)pathpoint2[pathflag+1][7] == 2 )//pathflag+1 ==pointnum-1 )
        { 
            pubdataOutA.data = 1;//填充IO out数据
            pubDout4.publish(pubdataOutA);
            pubdataOutA.data = 0;//填充IO out数据
            pubDout3.publish(pubdataOutA);
            pubDout5.publish(pubdataOutA);
        }
        else
        {
            pubdataOutA.data = 1;//填充IO out数据
            pubDout3.publish(pubdataOutA);
            pubDout4.publish(pubdataOutA);
            pubDout5.publish(pubdataOutA);
        }

        float lanechdis = 0.1;
        if(pathflag < pointnum-2)
        {
        	lanechdis = 0.03;
        }
        else
        {
        	lanechdis = 0.03;
        }
         printf("lanechdis %f  \n", lanechdis); 
         printf("pointnum %d  \n", pointnum); 
         printf("(int)pathpoint2[pathflag+1][7]-------- %d  \n",  (int)pathpoint2[pathflag+1][7]); 
         EndTask=0;

        if ( ObsSignal.bit.DI6 == 0 && (pathpoint2[pathflag+1][6] >= 6.9 && pathpoint2[pathflag+1][6] <= 7.1) )
        {
            lanechdis = 0.140;
        }

         if (pathpoint2[pathflag+1][6] >= 6.9 && pathpoint2[pathflag+1][6] <= 7.1)//pathflag+1 ==pointnum-1 )
        { 
            Ctrldata2.spleed = spleedctrl2((star_end - driving_distance),0.125 , lanechdis ,Ctrldata2.spleed, 0.1);
            printf("spleedctrl2.......... %f  \n", Ctrldata2.spleed); 
        }
                
        printf("lanechdis---------------- %f  \n", lanechdis); 
         
        if (driving_distance > star_end-lanechdis || CloseRoute ==1)//&& pathflag < pointnum-1)
        {
            //新增发布目的地ID
            if(CloseRoute ==0)
            {
                std_msgs::UInt32MultiArray  pubEndTaskdata;
                pubEndTaskdata.data.push_back(pathpoint2[pathflag+1][3]);
                pubEndTaskID.publish(pubEndTaskdata);
            }

            spleedtak = 0;
            runflag = 1 + runflag;
            pathflag = 1 + pathflag;
            rotation_flg = 0;
           if (pathflag==pointnum-1 || CloseRoute ==1)
           { 
             runflag = 0;
             pathflag = 0;
             //starcar=0;
             starcarend=0;
             //car_war=1;  
             runcarstar = 0;   
             EndTask=1;
             CloseRoute = 0;
             
             LastPath_Point.data.clear();
             Path_Point.data.clear();
             for (size_t i = 0; i < pathleng/9; i++)
		     {
		        for (size_t j = 0; j < 9; j++)
		        {
		            //pathpoint2[i][j] = 0;
		            printf("  %f  ", pathpoint2[i][j]);  
		        }
		            printf("  \n");  
		     }
                   
           }
            
            stopflag = 1;
        }
        
        
       /* else if (driving_distance > star_end-0.1 && pathflag == pointnum-1)
        {
        	 runflag = 0;
             pathflag = 0;
             //starcar=0;
             starcarend=0;
             car_war=1;
        
        }
        
        if(Ctrldata2.angle > -0.5 && Ctrldata2.angle < 0.5 )
        {
            Ctrldata2.angle = 0;
        }*/
        
        punavidata.data.push_back((short )(Ctrldata2.angle * 100));
        punavidata.data.push_back((short) (Ctrldata2.spleed * 100 * spleedcoef));
        punavidata.data.push_back(0);
        punavidata.data.push_back(0);

        pubnavigdata.publish(punavidata);   

        
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
    printf("pathleng2 %d  \n", pathleng); 
    CloseRoute = 0;
    if (paraMsg.data == LastPath_Point.data)
    {
        return;
    }
    LastPath_Point.data.clear();
    LastPath_Point.data =  paraMsg.data;
    
	//Path_Point.data.clear();
    //Path_Point.data =  paraMsg.data;
    printf("EndTask %d  \n", EndTask); 
    if (EndTask==1)
    {
        EndTask = 0;
        Path_Point.data.clear();
        Path_Point.data .push_back(0);
        Path_Point.data .push_back(locX);
        Path_Point.data .push_back(locY);
        Path_Point.data .push_back(0);
        Path_Point.data .push_back(0);
        Path_Point.data .push_back(0);
        Path_Point.data .push_back(0);
        Path_Point.data .push_back(0);
        Path_Point.data .push_back(0);
        Path_Point.data .push_back(0);
    }

    uint32_t pathleng2 = paraMsg.data.size();
    for (size_t i = 1; i < pathleng2; i++)
    {
        Path_Point.data.push_back(paraMsg.data[i]);
    }
    pathleng = Path_Point.data.size();
    initflg = 0; 
    ExcuteActiondata = 0;
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

void carrun_star2Handler(const std_msgs::UInt32 &paraMsg)
{
   runcarstar = paraMsg.data;
   starcarend = runcarstar;
   printf("runcarstar %d  \n", runcarstar);  
}
//---------------------------------------------------------------------------------------------------------------------
void carrun_starHandler(const std_msgs::UInt8MultiArray &paraMsg)
{
    direcVe=paraMsg.data[0];
     starVe=paraMsg.data[1];

    printf("direcVe %d  \n", direcVe);  
    printf("starVe %d  \n", starVe);  
}
//---------------------------------------------------------------------------------------------------------------------
void carrun_safeHandler(const std_msgs::UInt8 &paraMsg)
{
     starcar = paraMsg.data;
     if ( starcar == 0)
     {
        spleedcoef = 1.0;
     }
     else if ( starcar == 1)
     {
        spleedcoef = 0;
     }
      else if ( starcar == 2)
     {
        spleedcoef = 0.1;
     }
     else if ( starcar ==4)
     {
        spleedcoef = 0.5;
     }
     
     
     
     printf("starcar2 %d  \n", starcar);  
}

//----------------------------------------------------------------------------------------------------------------------
uint8_t sstop = 0;
void SafetylevelHandler(const std_msgs::UInt8 &paraMsg)
{
     sstop = paraMsg.data;
        
     printf("sstop %d  \n", sstop);  
}

//---------------------------------------------------------------------------------------------------------------------
void carrun_warHandler(const std_msgs::UInt32 &paraMsg)
{
     car_war = paraMsg.data;
     printf("car_war %d  \n", car_war);  
}
//---------------------------------------------------------------------------------------------------------------------
uint8_t DIN4rst = 1;
uint32_t delayflg = 0;
void DataINHandler(const std_msgs::UInt16 &paraMsg)
{
    uint16_t ObstacleSignal = paraMsg.data;
    ObsSignal.DataIn = ObstacleSignal;
    
    uint8_t runsound = 0;
    uint8_t warningsound = 0;
    delayflg ++;
    if ( delayflg > 150 )
    {
        delayflg = 150;
        spleedcoef = 1;
    }
    
    if ( ObsSignal.bit.DI9 == 1 || ObsSignal.bit.DI12 == 1 )
    {
       //spleedcoef = 0.5;
    }
    if ( ObsSignal.bit.DI10 == 1 || ObsSignal.bit.DI13 == 1 )
    {
       if (delayflg == 150)
       {
        spleedcoef = 0.5; 
       }       
       warningsound = 0; 
       runsound =1;        
    }
    if ( ObsSignal.bit.DI11 == 1 || ObsSignal.bit.DI14 == 1 || sstop == 1)
    {
       spleedcoef = 0;
       runsound =0;
       warningsound = 1;
       delayflg = 0;
    }

    if ( ObsSignal.bit.DI4 == 1 )
    {
        DIN4rst = 0 ;
    }

    if ( ObsSignal.bit.DI3 == 1 )
    {
        DIN4rst = 1 ;
    }

    if(backCarflag == 1)//倒车
    {
      warningsound = 0; 
      runsound =0;
      spleedcoef = 1.0; 
    }

    if ( DIN4rst == 0 )
    {
        spleedcoef = 0;
        runsound =0;
        warningsound = 1;
        delayflg = 0;
    }
    
    std_msgs::UInt8 pubdataOut7;
	pubdataOut7.data = runsound;//填充IO out数据
	pubDout7.publish(pubdataOut7);
		 
	std_msgs::UInt8 pubdataOut8;
	pubdataOut8.data = warningsound;//填充IO out数据
	pubDout8.publish(pubdataOut8);
	/*	 
	printf("ObsSignal.bit.DI3--------------- %d  \n",  ObsSignal.bit.DI3);  
    printf("ObsSignal.bit.DI4--------------- %d  \n",  ObsSignal.bit.DI4);  
    printf("ObsSignal.bit.DI5--------------- %d  \n",  ObsSignal.bit.DI5);   
    printf("ObsSignal.bit.DI6--------------- %d  \n",  ObsSignal.bit.DI6);  
    printf("ObsSignal.bit.DI7--------------- %d  \n",  ObsSignal.bit.DI7);  
    printf("ObsSignal.bit.DI9--------------- %d  \n",  ObsSignal.bit.DI9);  
    printf("ObsSignal.bit.DI10--------------- %d  \n",  ObsSignal.bit.DI10);  
    printf("ObsSignal.bit.DI11--------------- %d  \n",  ObsSignal.bit.DI11);  
    printf("ObsSignal.bit.DI12--------------- %d  \n",  ObsSignal.bit.DI12);  
    printf("ObsSignal.bit.DI13--------------- %d  \n",  ObsSignal.bit.DI13);  
    printf("ObsSignal.bit.DI14--------------- %d  \n",  ObsSignal.bit.DI14); */
    ROS_INFO("ObstacleSignal %d  \n",  ObstacleSignal);  
}
//---------------------------------------------------------------------------------------------------------------------

void ExcuteActionHandle(const std_msgs::UInt8 &paraMsg)
{
   
    ExcuteActiondata = paraMsg.data;
   
   //printf("TargetSiteID %d  \n", SiteID); 
}
//---------------------------------------------------------------------------------------------------------------------

void CloseRouteHandle(const std_msgs::UInt8 &paraMsg)
{
   
    CloseRoute = paraMsg.data;
   
   //printf("TargetSiteID %d  \n", SiteID); 
}
//---------------------------------------------------------------------------------------------------------------------
void ndt_reliabilityHandler(const std_msgs::Float32 &paraMsg)
{
    erry_map_data2 = paraMsg.data;
    if(erry_map_data2 > 30.0)
    {
       erry_map_cut --;    
       if(erry_map_cut < 0 )
       {
           erry_map_cut = -1;
       }
    }
    else
    {
       erry_map_cut = 30.0;
    }

    printf("erry_map_data2 %f  \n", erry_map_data2);
    printf("erry_map_cut %d  \n", erry_map_cut);
}
//---------------------------------------------------------------------------------------------------------------------
int main(int argc, char** argv)
{
    ros::init(argc, argv, "AckermannVehicleNavigation");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle n;
     ros::NodeHandle private_nh("~");

    private_nh.getParam("starAX", _starAX);
    private_nh.getParam("starAY", _starAY);
    private_nh.getParam("endBX", _endBX);
    private_nh.getParam("endBY", _endBY);
    private_nh.getParam("endP1X", _endP1X);
    private_nh.getParam("endP1Y", _endP1Y);
    private_nh.getParam("endP2X", _endP2X);
    private_nh.getParam("endP2Y", _endP2Y);
    private_nh.getParam("AreaSele", _AreaSele);
    private_nh.getParam("spleed", _spleed);

    std_msgs::UInt8 AreaSele;

    std::cout << " _starAX:  " << _starAX << "  "
            << "_starAY:  " << _starAY << "  " << std::endl;
    std::cout << " _endBX:  " << _endBX << "  "
            << "_endBY:  " << _endBY << "  "    << "_spleed:  " << _spleed << "  "<< std::endl;
   // ros::Publisher tcp_jie =n.advertise<navgation_pack::androidyaokong>("/android_para", 100);
    pubAreasele =  n.advertise<std_msgs::UInt8>("/SelectionProtectiveAreas", 100);
    pubctrl_init =  n.advertise<std_msgs::UInt8>("/ctrl_init", 100);

    pubVehStatus = n.advertise<std_msgs::UInt8MultiArray>("/VehStatus", 100);
    pubEndTaskID = n.advertise<std_msgs::UInt32MultiArray>("/EndTaskID", 100);
    pubTargetTaskID = n.advertise<std_msgs::UInt32MultiArray>("/TargetTaskID", 100);
    pubDout7 =  n.advertise<std_msgs::UInt8>("/DO7", 100);
    pubDout8 =  n.advertise<std_msgs::UInt8>("/DO8", 100);
    pubDout1 =  n.advertise<std_msgs::UInt8>("/DO1", 100);
    pubDout2 =  n.advertise<std_msgs::UInt8>("/DO2", 100);

    pubDout3 =  n.advertise<std_msgs::UInt8>("/DO3", 100);
    pubDout4 =  n.advertise<std_msgs::UInt8>("/DO4", 100);
    pubDout5 =  n.advertise<std_msgs::UInt8>("/DO5", 100);

    pubCarStu =  n.advertise<std_msgs::UInt8>("/CarStu", 100);
    publoc_stau =  n.advertise<std_msgs::UInt8>("/loc_stau", 100);
    
    ros::Subscriber  subLaserOdometry = n.subscribe<geometry_msgs::PoseStamped>("/ndt_pose", 5,  outputdata);
    //ros::Subscriber  subLaserOdometry = n.subscribe<nav_msgs::Odometry>("/integrated_to_init", 5,  outputdata);
    ros::Subscriber arer_date = n.subscribe("/lidardate_para", 1, arer_dateHandler);
    pubnavigdata = n.advertise<std_msgs::Int16MultiArray>("/navigdata", 100);
    
    ros::Subscriber path_point = n.subscribe("/pathpoint", 1, pathpointeHandler);
    ros::Subscriber carrun_num = n.subscribe("/car_run_num", 1, carrun_numHandler);
    ros::Subscriber carrun_2star = n.subscribe("/car_run_star", 1, carrun_star2Handler);
     
    ros::Subscriber carrun_star = n.subscribe("/starstop", 1, carrun_starHandler);
    ros::Subscriber carrun_safe = n.subscribe("/ObstacleDetectionResults", 1, carrun_safeHandler);
    ros::Subscriber Safetylevel = n.subscribe("/Safetylevel", 1, SafetylevelHandler);

    ros::Subscriber carrun_war = n.subscribe("/car_waring", 1, carrun_warHandler);

    ros::Subscriber  erry_Map = n.subscribe("/erry_map", 10, erry_mapHandler);
    ros::Subscriber  loadMap = n.subscribe("/loadmap", 10, loadmapHandler);
    ros::Subscriber  subDIN = n.subscribe("/DataIN", 10, DataINHandler);
    ros::Subscriber subExcuteAction = n.subscribe("/ExcuteAction", 5, ExcuteActionHandle);
    ros::Subscriber subCloseRoute = n.subscribe("/CloseRoute", 5, CloseRouteHandle);
    ros::Subscriber  sub_ndt_reliability = n.subscribe("/ndt_reliability", 10, ndt_reliabilityHandler);
//pubpathpoint = n.advertise<std_msgs::Float64MultiArray>("/pathpoint", 100);

    ros::Rate loop_rate(20);
    uint16_t tcount = 0; 
    uint16_t tcount2 = 200;
   while (ros::ok()) 
   {
        AreaSele.data = (uint8_t) _AreaSele;
        pubAreasele.publish( AreaSele );
        
        std_msgs::UInt8 pubdataOut1;
        std_msgs::UInt8 pubdataOut2;
        
        if(ExcuteActiondata == 3)
        {
		    tcount = 0;
		    tcount2 = 200;
		    pubdataOut1.data = 1;//填充IO out数据
		    pubDout1.publish(pubdataOut1);
		    printf("ExcuteActiondata1 %d  \n",  ExcuteActiondata); 
		    if( ObsSignal.bit.DI5 ==0 )
		    {
		        printf("ExcuteActiondata2 %d  \n",  ExcuteActiondata); 
		        pubdataOut2.data = 1;//填充IO out数据
		        pubDout2.publish(pubdataOut2);
		        pubdataCarStu.data = 3;
		    }
        }
        else if(ExcuteActiondata == 4)
        {
		    pubdataOut1.data = 0;//填充IO out数据
		    pubDout1.publish(pubdataOut1);
		    tcount ++;
		    if( tcount > 200 )
		    {
		        tcount = 200;
		        pubdataOut2.data = 0;//填充IO out数据
		        pubDout2.publish(pubdataOut2);
		    }
		     if( ObsSignal.bit.DI5 ==1 )
		    {
		        pubdataCarStu.data = 1;//填充IO out数据
		       /* tcount2--
				if( tcount2 < 0 )
				{
				    tcount = -1;
				    ExcuteActiondata = 0;
				}
		        //ExcuteActiondata = 0;*/
		        
		    }
        }
        
        pubCarStu.publish(pubdataCarStu);
        // std::cout << " AreaSele.data:  " << AreaSele.data  << std::endl;          
        ros::spinOnce(); 
        loop_rate.sleep(); 
     }
        
    return 0;
}
