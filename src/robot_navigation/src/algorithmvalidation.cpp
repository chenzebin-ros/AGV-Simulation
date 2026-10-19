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

#include <unistd.h> //对系统API操作
#include <iostream> //IO数据流
#include <fstream>  //文件读写

using namespace std;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "algorithmvalidation");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle n;

    char *path = get_current_dir_name(); //调用系统API获取当前路径
    string currentPath = path;
    free(path);  //释放指针
    path = NULL; //指针重新赋值
 
   string  starpoint;
   string endpoint;
   string cuepoint;
   string halder;
   string frontdis;
   string vvvo;
 
   cout << "请输入起点： ";
   cin >> starpoint;
   cerr << "输入起点是： " << starpoint << endl;

    cout << "请输入终点： ";
   cin >> endpoint;
   cerr << "输入终点是： " << endpoint << endl;

    cout << "请输入当前点： ";
   cin >> cuepoint;
   cerr << "输入当前点是： " << cuepoint << endl;

    cout << "请输入车辆航向： ";
   cin >> halder;
   cerr << "输入当前航向是： " << halder << endl;

   cout << "请输入前是距离： ";
   cin >> frontdis;
   cerr << "输入当前是距离： " << frontdis << endl;

   cout << "请输入车辆速度： ";
   cin >> vvvo;
   cerr << "输入当前速度是： " << vvvo << endl;

   int indexParent = starpoint.find_last_of(",");
   string starpoint1 = starpoint.substr(0, indexParent);
   string starpoint2 = starpoint.substr(indexParent+1);
   cout << "starpoint1:" <<atof( starpoint1.c_str() )<< "  starpoint2:" <<atof(starpoint2.c_str() )<< endl;

  int indexParent2 = endpoint.find_last_of(",");
   string endpoint1 = endpoint.substr(0, indexParent2);
   string endpoint2 = endpoint.substr(indexParent2+1);
   cout << "endpoint1:" <<atof( endpoint1.c_str() )<< "  endpoint2:" <<atof(endpoint2.c_str() )<< endl;  

    int indexParent3 = cuepoint.find_last_of(",");
   string cuepoint1 = cuepoint.substr(0, indexParent3);
   string cuepoint2 = cuepoint.substr(indexParent3+1);
   cout << "cuepoint1:" << atof( cuepoint1.c_str() ) << "  cuepoint2:" << atof(cuepoint2.c_str()  )<<  "  heading:" << atof(halder.c_str()  ) << endl;  

    double star_point[2] = {atof( starpoint1.c_str() ), atof(starpoint2.c_str() )};
    double end_point[2] = {atof( endpoint1.c_str() ), atof(endpoint2.c_str() )};
    double cur_pose[3] = {atof( cuepoint1.c_str() ), atof(cuepoint2.c_str() ) ,atof(halder.c_str()  ) };

    double  pd_con = atof( frontdis.c_str());
    double Va = atof( vvvo.c_str());

    cout << "pd_con:" <<pd_con<< "  Va:" <<Va<< endl;  
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

    float vector_axb = vector_ax*vector_by - vector_bx*vector_ay;   //a x b乘积
    float vector_lal = sqrt(vector_ax*vector_ax + vector_ay*vector_ay );   //AB路径a向量模长
    float err_distance = vector_axb / vector_lal ;   //求偏距，包括方向

    float vector_axn = vector_ax*car_ny - car_nx*vector_ay;   //a x n乘积,求偏角
    float vector_a_n = vector_ax*car_nx + car_ny*vector_ay;   //a 点 n乘积,求偏角
 
    float err_angle = atan2( vector_axn, vector_a_n);                     //追踪线与车辆夹角，，范围正负180
	float dis_L2 = err_distance / sin(err_angle);							//求车辆距离瞬心的距离
    float err_angle2 = err_angle *180/3.1415926; 
	
	
    float dis_OO4 = dis_L2 + pd_con;
	float dis_O34 = dis_OO4*tan(err_angle);
	float angle_a =atan2(pd_con ,dis_O34);

    cout << "vector_ax:" << vector_ax << endl;
    cout << "vector_ay:" << vector_ay << endl;

    cout << "vector_bx:" << vector_bx << endl;
    cout << "vector_by:" << vector_by << endl;

   cout << "vector_axb:" << vector_axb << endl;
    cout << "vector_lal:" << vector_lal << endl;

  cout << "err_distance:" << err_distance << endl;


   cout << "err_distance:" <<err_distance << " err_angle:" <<err_angle<< "  dis_L2:" <<dis_L2<< "  angle_a:" <<angle_a<< endl;  
	//float dis_O12 = err_distance / sin(3.1415926/2 - err_angle);			//可以简写cos()
	//float dis_O2s = pd_con / tan(err_angle);			
	//float dis_O1s = dis_O12 + dis_O2s;
	//float dis_O13 = sqrt(pd_con*pd_con + dis_O1s*dis_O1s);
	//float angle_a = asin(pd_con / dis_O13);
	
	float Vx = Va * sin(angle_a);
	float Vy = Va * cos(angle_a);
	float Ws =  Vy / (dis_L2 + pd_con);

    // int indexParent = currentPath.find_last_of("/");
    // string parentPath = currentPath.substr(0, indexParent);
    // string targetFile = currentPath + "/resource/P1icking.txt"; //  操作文件路径
    
    cout << "Vx:" << Vx << endl;
    cout << "Vy:" << Vy << endl;
    cout << "Ws:" << Ws << endl;

    ros::Rate loop_rate(40);
   while (ros::ok()) 
   {
                ros::spinOnce(); 
                loop_rate.sleep(); 
     }
        
    return 0;
}
