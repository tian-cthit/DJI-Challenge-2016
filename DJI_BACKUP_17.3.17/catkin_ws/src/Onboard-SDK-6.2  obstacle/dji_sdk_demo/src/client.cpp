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


FILE *fpwny=fopen("/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk_demo/data/control_cmd.txt","w");
using namespace DJI::onboardSDK;

ros::Subscriber Ford150_position_sub;
geometry_msgs::Twist  Ford_150;



int main(int argc, char **argv)
{

    ros::init(argc, argv, "flight_control");
    ros::NodeHandle nh;
    	char c;

		float yaw_uav;								//define Euler Angels
        	float pitch_uav;
          	float roll_uav;
    DJIDrone* drone = new DJIDrone(nh);

       for(int i=0;i<10;i++)
		{
			drone->activate();
			usleep(10000);
			printf("adrone->activation==%d\n",drone->activation);
			if(!drone->activation)
			{
			    ROS_INFO("active success ok !!");
				break;

			}
		}

		float v_x=0.0;
		float v_y=0.0;

			/*
			if (forward < 2.0||backword < 2.0)
			{
				v_x = 0.0;

				if (right>3.0)
				{
					v_y = 1.0;
					sleep(3);
					v_y = 0.0;
				}
				else
				{
					v_y = 0.0;
				}
				if (forward < 2.0&&backword > 2.0)
				{
					v_x = -1.0;
				}
				else if(forward > 2.0&&backword < 2.0)
				{
					v_x = 1.0;
				}
				else
				{
				}
			}
			else
			{

			}
			*/



			while( drone->ultrasonic.ranges[2]>3.0 && drone->obstacle_distance.ranges[2] > 3.0)
			{
				while(drone->ultrasonic.ranges[1] > 2.0 && drone->obstacle_distance.ranges[1] > 2.0)
				{
					v_x = 1.0;

				}
				v_x = 0.0;


				int count1=0;
				ros::Rate p(50);
				while(count1<100)
				{
					v_y=1.0;
					count1++;
					ros::spinOnce();
					p.sleep();
				}
				v_y = 0.0;

				while(drone->ultrasonic.ranges[3] > 2.0 && drone->obstacle_distance.ranges[3] > 2.0)
				{
					v_x = -1.0;
				}
				v_x = 0.0;

				int count2=0;
				ros::Rate q(50);
				while(count2<100)
				{
					v_y=1.0;
					count2++;
					ros::spinOnce();
					q.sleep();
				}
				v_y = 0.0;
			}

			ros::Rate r(50); 			  //		F=50Hz
			while(ros::ok())
			{
				if(drone->rc_channels.gear>-5000)
				{
					drone->request_sdk_permission_control();
					usleep(1000);
					//printf("wait for control permission.\n");


				drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
                            Flight::VerticalLogic::VERTICAL_VELOCITY |
                            Flight::YawLogic::YAW_PALSTANCE |
                            Flight::HorizontalCoordinate::HORIZONTAL_BODY |			//BODY or GROUND
                            Flight::SmoothMode::SMOOTH_ENABLE,
                            v_x,v_y,0.0,0.0);		// v_x,v_y,v_h,v_yaw
				}
				else
				{
				}

				ros::spinOnce();
				r.sleep();
			}

	return 0;
}
