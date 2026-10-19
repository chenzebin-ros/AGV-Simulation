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
    ros::init(argc, argv, "readtxt");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle n;

    char *path = get_current_dir_name(); //调用系统API获取当前路径
    string currentPath = path;
    free(path);  //释放指针
    path = NULL; //指针重新赋值
 
 char name[50];
 
//    cout << "请输入您的名称： ";
//    cin >> name;
//    cerr << "您的名称是： " << name << endl;

    int indexParent = currentPath.find_last_of("/");
    string parentPath = currentPath.substr(0, indexParent);
    string targetFile = currentPath + "/resource/P1icking.txt"; //  操作文件路径
    
    cout << "currentPath:" << currentPath << endl;
    cout << "parentPath:" << parentPath << endl;
    cout << "targetFile:" << targetFile << endl;

    ifstream readStream;
    double redbuf[1024] = {};
    readStream.open(targetFile, ios::in); //文件路径  输入至内存
    if (readStream.is_open())
    {
        string readStr;
        indexParent = 0;
        while (readStream >> readStr)
        {
            //cout << "readStream:"  << readStream << endl;
            //以空格为单位输出
            redbuf[indexParent] = stod(readStr);
            indexParent++;
            cout << indexParent << "   :   " << readStr << endl;


        }
        readStream.close();
    }
    else
    {
        cout << "Open Error!" << endl;
    }
   double pathpoint[indexParent/4][5]={0};
    for (size_t i = 0; i <  indexParent/4; i++)
    {
       for (size_t j = 0; j < 4; j++)
       {
           pathpoint[i][j] = redbuf[ i*4+j ];
           
       }

        cout  <<  pathpoint[i][0] << "  " <<  pathpoint[i][1] << "  "<<  pathpoint[i][2] << "  "<<  pathpoint[i][3]  << "  "<< endl;
    }
    

   // ros::Publisher tcp_jie =n.advertise<navgation_pack::androidyaokong>("/android_para", 100);


    ros::Rate loop_rate(40);
   while (ros::ok()) 
   {
   
                  
                ros::spinOnce(); 
                loop_rate.sleep(); 
     }
        
    return 0;
}
