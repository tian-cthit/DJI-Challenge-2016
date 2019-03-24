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

		int a=0;
		int count=0;
		int c = 0;
		int d = 0;
		int number=1;
		float yaw_begin=0.0;
		float yaw_end=0.0;
		float distance=3.0;
		float v_h=0.0;


while(ros::ok())
{
	ros::spinOnce();
	Quaternion_To_Euler(drone->attitude_quaternion.q0,drone->attitude_quaternion.q1,drone->attitude_quaternion.q2,drone->attitude_quaternion.q3,&yaw_uav,&pitch_uav,&roll_uav);
	//printf("yaw=%f\n",yaw_uav);
	if(drone->rc_channels.gear==-4545.0)
	{
		drone->request_sdk_permission_control();
		usleep(1000);
		
				ros::Rate m(50);
				while(drone->ultrasonic.ranges[2] > distance || drone->ultrasonic.ranges[2] < 0.2||drone->ultrasonic.ranges[3] < 2.0) // no obstacle front , go front
				{
					ros::spinOnce();
					printf("gear=%f\n",drone->rc_channels.gear);
					if(drone->ultrasonic.ranges[3] < 1.5 )
					{
                        v_x = 0.05;
					}
					else if(drone->ultrasonic.ranges[0] > 1.5 )
					{
                        v_x = -0.05;
					}
					else
					{
                        v_x = 0.0;
					}

			if(drone->ultrasonic.ranges[0] < 0.5 )
					{
                        v_h = 0.05;
					}
					else if(drone->ultrasonic.ranges[0] > 0.7 )
					{
                        v_h = -0.05;
					}
					else
					{
                        v_h = 0.0;
					}

				drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
                                Flight::VerticalLogic::VERTICAL_VELOCITY |
                                Flight::YawLogic::YAW_PALSTANCE |
                                Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                                Flight::SmoothMode::SMOOTH_ENABLE,
                                v_x,0.3,v_h,0.0);
					m.sleep();
				}
				//number++;
				
				//printf("WTF!!!\n");
				
				//Quaternion_To_Euler(drone->attitude_quaternion.q0,drone->attitude_quaternion.q1,drone->attitude_quaternion.q2,drone->attitude_quaternion.q3,&yaw_uav,&pitch_uav,&roll_uav);
				//yaw_begin=yaw_uav;
				/*for(a=0;a<360;a++)
				{
					//Quaternion_To_Euler(drone->attitude_quaternion.q0,drone->attitude_quaternion.q1,drone->attitude_quaternion.q2,drone->attitude_quaternion.q3,&yaw_uav,&pitch_uav,&roll_uav);
					//yaw_end=yaw_uav;
					//if((yaw_end-yaw_begin)<)
					//{

						drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
                            Flight::VerticalLogic::VERTICAL_VELOCITY |
                            Flight::YawLogic::YAW_PALSTANCE |
                            Flight::HorizontalCoordinate::HORIZONTAL_BODY |			//BODY or GROUND
                            Flight::SmoothMode::SMOOTH_ENABLE,
                            0.0,0.0,0.0,12.5);
						usleep(20000);
					//}
					//else
					//{
						//break;
					//}
				}*/
				ros::Rate k(50);
				while(c < 250)
				{
					ros::spinOnce();
					drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
                            Flight::VerticalLogic::VERTICAL_VELOCITY |
                            Flight::YawLogic::YAW_PALSTANCE |
                            Flight::HorizontalCoordinate::HORIZONTAL_BODY |			//BODY or GROUND
                            Flight::SmoothMode::SMOOTH_ENABLE,
                            0.0,0.3,0.0,0.0);
                    c++;
                    k.sleep();
                }

				ros::Rate n(50);
				while(count < 358) // no obstacle front , go front
				{
					ros::spinOnce();
					drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
                            Flight::VerticalLogic::VERTICAL_VELOCITY |
                            Flight::YawLogic::YAW_PALSTANCE |
                            Flight::HorizontalCoordinate::HORIZONTAL_BODY |			//BODY or GROUND
                            Flight::SmoothMode::SMOOTH_ENABLE,
                            0.0,0.0,0.0,12.5);
                    count++;
                    n.sleep();
                }
	
				ros::Rate j(50);
				while(d < 250)
				{
					ros::spinOnce();
					drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
                            Flight::VerticalLogic::VERTICAL_VELOCITY |
                            Flight::YawLogic::YAW_PALSTANCE |
                            Flight::HorizontalCoordinate::HORIZONTAL_BODY |			//BODY or GROUND
                            Flight::SmoothMode::SMOOTH_ENABLE,
                            0.0,0.3,0.0,0.0);
                    d++;
                    j.sleep();
                }
				/*count=0;
				if(number>3&&number<9)
				{
					distance=3.5;
				}
				else if(number<=3)
				{
					distance=3.0;
				}
				else
				{
					ros::Rate n(50);
					while(number>1)
					{	
						ros::spinOnce();		
						drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
                            Flight::VerticalLogic::VERTICAL_VELOCITY |
                            Flight::YawLogic::YAW_PALSTANCE |
                            Flight::HorizontalCoordinate::HORIZONTAL_BODY |			//BODY or GROUND
                            Flight::SmoothMode::SMOOTH_ENABLE,
                            0.0,0.0,0.0,0.0);
                        n.sleep();
					}
				}*/
	}
	else
	{
	}
		//printf("gear=%f\n",drone->rc_channels.gear);

}

	return 0;
}
