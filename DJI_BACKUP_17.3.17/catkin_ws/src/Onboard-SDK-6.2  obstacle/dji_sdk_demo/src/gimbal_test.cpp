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
FILE *fp=fopen("/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk_demo/src/gimbal.txt","w");
using namespace DJI::onboardSDK;
int main(int argc, char **argv)
{
   
    ros::init(argc, argv, "flight_control");
    ros::NodeHandle nh;

    	char c;
	ros::Publisher pub = nh.advertise<sensor_msgs::Range>("iarc007/errorx",2);
        
    sensor_msgs::Range errorx_pub;
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
    while(i<200)
    {
    	ROS_INFO("START\n");
    	back_1=drone->gimbal.pitch;
    	time1=drone->gimbal.ts;
    //	printf("time1-time2====%f\n",time1-time2);
    	//ROS_INFO("STOP\n");
    	
        	back=back_1-back_2;
        	//v=back/(0.005+s); 
        	// printf("back=====%f\n",back);
        	 v=back/((time2-time1)*0.001); 
        
        	ROS_INFO("output==%f\n",v);
        	// printf("output=%f\n",v);
        	fprintf(fp,"%f\n",v);
       
    	back_2=back_1;
         time2=time1;
    	drone->gimbal_speed_control(0.0,-angle,0.0);
    	
    	
    	
    	errorx_pub.header.stamp = ros::Time::now();
                        errorx_pub.range = v;	
			pub.publish(errorx_pub);
    	
    	i++;
    	ros::spinOnce();
   	r.sleep();
   	ROS_INFO("STOP\n");
    }
    fclose(fp);
	return 0;
}

