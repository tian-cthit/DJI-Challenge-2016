#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <dji_sdk/dji_drone.h>

#include <time.h>

#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Twist.h>//position
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <std_msgs/Float32.h>

#include <math.h>
#include <opencv/cxcore.h>
#include  <opencv/cvaux.h>
#include <sensor_msgs/Range.h>

#include"wqf_math.h" //for header file
#include"pid_set.h"
FILE *fp_speed=fopen("/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk_demo/src/M100v.txt","w");
FILE *fp=fopen("/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk_demo/src/GPS.txt","w");
using namespace DJI::onboardSDK;

//This is used for Gps test****************************************************************
int main(int argc, char **argv)
{
   
    ros::init(argc, argv, "flight_control");
    ros::NodeHandle nh;
    char c;	
    DJIDrone* drone = new DJIDrone(nh);  
    int i=0;
    float angle=500.0;
    float back_1=0.0;
    float back_2=0.0;
    float back=0.0;
    float v=0.0;
    float time1=0.0;
    float time2=0.0;
    float s=0.0;	
    ros::Rate r(100);
    while(ros::ok())
    {
    
           
    //  printf("i==%d\t latitude===%.12lf\t longitude==%.12lf\t altitude===%.12lf\t height===%.12lf\thealth==%d\n",i,drone->global_position.latitude,drone->global_position.longitude,drone->global_position.altitude,drone->global_position.height,drone->global_position.health);

      fprintf(fp,"%d\t %.12lf\t %.12lf\t %.12lf\t %0.12lf\t %d\n",i,drone->global_position.latitude,drone->global_position.longitude,drone->global_position.altitude,drone->global_position.height,drone->global_position.health);
          fprintf(fp_speed,"%f\t %f\t %f\t %d\t",drone->velocity.vx,drone->velocity.vy,drone->velocity.vz,drone->velocity.health_flag);
      ros::spinOnce();
      r.sleep();
                i++;
    }
     fclose(fp_speed);
     fclose(fp);
	return 0;
}

