/*
 * main_sdk0428.cpp
 *
 *  Created on: Apr 29, 2015
 *      Author: craig
 */
#include <stdio.h>
#include <string.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include "Quatanion2Matrix.h"
#include <geometry_msgs/TransformStamped.h> //IMU
#include <geometry_msgs/Vector3Stamped.h> //velocity
ros::Subscriber depth_image_sub;
ros::Publisher pub_pointcloud2;
ros::Subscriber imu_sub;
ros::Subscriber position_sub;
using namespace cv;
#define WIDTH 320
#define HEIGHT 240
const float camera_cu = 154.1;
const float camera_cv = 118.744;
const float focal = 251.683;
const float baseline = 0.150067;
const float camera_factor=255;


CvMat *q_guidance=cvCreateMat(4,1,CV_32FC1);
 float roll_test;
 float pitch_test;
 float yaw_test;
 float guidance_position_x=0.0;
  float guidance_position_y=0.0;
   float guidance_position_z=0.0;
    float forward_camera2ground=-0.135;  //forward camera distance to ground
/* motion */
void position_callback(const geometry_msgs::Vector3Stamped& g_pos)
{
       guidance_position_x=g_pos.vector.x;
         guidance_position_y=g_pos.vector.y;
         guidance_position_z=g_pos.vector.z;
	//printf("frame_id: %s stamp: %d\n", g_pos.header.frame_id.c_str(), g_pos.header.stamp.sec);

		printf("global position: [%f %f %f]\n", guidance_position_x, guidance_position_y, guidance_position_z);
}

void imu_callback(const geometry_msgs::TransformStamped& g_imu)
{ 
   // printf( "frame_id: %s stamp: %d\n", g_imu.header.frame_id.c_str(), g_imu.header.stamp.sec );
    //printf( "imu: [%f %f %f %f %f %f %f]\n", g_imu.transform.translation.x, g_imu.transform.translation.y, g_imu.transform.translation.z, 
						//g_imu.transform.rotation.x, g_imu.transform.rotation.y, g_imu.transform.rotation.z, g_imu.transform.rotation.w );
  
    cvmSet(q_guidance,0,0,g_imu.transform.rotation.w);
    cvmSet(q_guidance,1,0,g_imu.transform.rotation.x);
    cvmSet(q_guidance,2,0,g_imu.transform.rotation.y);
    cvmSet(q_guidance,3,0,g_imu.transform.rotation.z);
/*
    cvmSet(q_guidance,0,0,1);
    cvmSet(q_guidance,1,0,0);
    cvmSet(q_guidance,2,0,0);
    cvmSet(q_guidance,3,0,0);
*/

 //Quaternion_To_Euler2(g_imu.transform.rotation.w,g_imu.transform.rotation.x,g_imu.transform.rotation.y,g_imu.transform.rotation.z,&roll_test,&pitch_test,&yaw_test);
  // printf("roll pitch yaw========%f  %f   %f\n",yaw_test,pitch_test,roll_test);
}

 CvMat *Rotation_guidance=cvCreateMat(3,3,CV_32FC1);
/* depth greyscale image */
void depth_image_callback(const sensor_msgs::ImageConstPtr& depth_img)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(depth_img, sensor_msgs::image_encodings::MONO16);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat depth8(HEIGHT, WIDTH, CV_8UC1);
    cv_ptr->image.convertTo(depth8, CV_8UC1);
    cv::imshow("depth_image", depth8);
    cv::waitKey(1);
       
  //before rotation
  typedef pcl::PointXYZ PointT;                  
    typedef pcl::PointCloud<PointT> PointCloud;   
     PointCloud::Ptr cloud ( new PointCloud );
    CvMat *P0=cvCreateMat(3,1,CV_32FC1);
    CvMat *P=cvCreateMat(3,1,CV_32FC1);

     //************************for rviz
     CvMat *R_rviz=cvCreateMat(3,3,CV_32FC1); 
       cvmSet(R_rviz,0,0,1); cvmSet(R_rviz,0,1,0); cvmSet(R_rviz,0,2,0); cvmSet(R_rviz,1,0,0); cvmSet(R_rviz,1,1,1); cvmSet(R_rviz,1,2,0); cvmSet(R_rviz,2,0,0); cvmSet(R_rviz,2,1,0); cvmSet(R_rviz,2,2,-1);
       //************************for rviz
     //pcl::PCLPointCloud2 cloud_filtered;
    // 遍历深度图
      // printf("depth====....%f   wwwwwwwwwwww=%d\n",float(  cv_ptr->image.at<short>( HEIGHT/2,WIDTH/2))/128,depth8.ptr<ushort>(HEIGHT/2)[WIDTH/2]);
    
     for (int m = 0; m < depth8.rows; m++)
 {
     for (int n=0; n < depth8.cols; n++)
        {
            // 获取深度图中(m,n)处的值
           float d =float( cv_ptr->image.at<short>(m,n))/128;
            
            // 筛选d的范围 ，若如此，跳过此点
            if (d<1||d>10)
             { 
                continue;
             }
            // d 存在值，则向点云增加一个点
            PointT p;
                  //printf("d=============%d\n",d/128);
            // 计算这个点的空间坐标 and rotation
                  q_to_dcm(q_guidance, Rotation_guidance,1); 
                  cvmSet(P0,0,0,(n - camera_cu) * d/ focal);
		  cvmSet(P0,1,0,(m - camera_cv) * d/ focal);
		  cvmSet(P0,2,0,d);    
                 cvGEMM(Rotation_guidance,P0,1,NULL,0,P);
                  cvGEMM(R_rviz,P,1,NULL,0,P);  //for rviz
 	    p.x = cvmGet(P,0,0)+guidance_position_x;
            p.y = cvmGet(P,1,0)+guidance_position_y;
            p.z = cvmGet(P,2,0)-guidance_position_z-forward_camera2ground; //for rviz
               // printf("d=============%f\n",p.z);
           // printf("x=============%f\n",p.x);
          //  printf("y=============%f\n",p.y);      
            // 把p加入到点云中
            cloud->points.push_back( p );
        }
  } 
    cvReleaseMat(&P0);
         cvReleaseMat(&P);
          cvReleaseMat(&R_rviz); //for rviz       
 // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
   //transfrom pcl::PointXYZ to sensor_msgs::PointCloud2   http://docs.ros.org/hydro/api/pcl_conversions/html/namespacepcl.html#abb8b3a2632e07dae0b541a257898c8a8
  pcl::toROSMsg(*cloud,output);
   output.header.frame_id = "guidance";
   output.header.stamp    = ros::Time::now();
  
/*VoexlGrid filter
 // Container for original & filtered data
  pcl::PCLPointCloud2* cloud2 = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud2);
  pcl::PCLPointCloud2 cloud_filtered;
 
  // Convert to PCL data type
  pcl_conversions::toPCL(output, *cloud2);
  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (cloud_filtered);
  // Convert to ROS data type
  sensor_msgs::PointCloud2 output2;
  pcl_conversions::moveFromPCL(cloud_filtered, output2);
  //set frame_id
   output2.header.frame_id = "guidance";
   output2.header.stamp    = ros::Time::now();



//pcl::StatisticalOutlierRemoval
// Container for original & filtered data
   pcl::PCLPointCloud2* cloud2 = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud2);
  pcl::PCLPointCloud2 cloud_filtered;
// Convert to PCL data type
  pcl_conversions::toPCL(output, *cloud2);
// Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (cloud_filtered);
 // Convert to ROS data type
  sensor_msgs::PointCloud2 output2;
  pcl_conversions::moveFromPCL(cloud_filtered, output2);
  //set frame_id


   output2.header.frame_id = "guidance";
   output2.header.stamp    = ros::Time::now();
  // Publish the data

*/

//Removing outliers using a Conditional or RadiusOutlier removal
// Container for original & filtered data
   pcl::PCLPointCloud2* cloud2 = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud2);
  pcl::PCLPointCloud2 cloud_filtered;
// Convert to PCL data type
  pcl_conversions::toPCL(output, *cloud2);
// Create the filtering object
   pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setRadiusSearch(0.1);
  sor.setMinNeighborsInRadius (200);
  sor.filter (cloud_filtered);
 // Convert to ROS data type
  sensor_msgs::PointCloud2 output2;
  pcl_conversions::moveFromPCL(cloud_filtered, output2);
  //set frame_id
   output2.header.frame_id = "guidance";
   output2.header.stamp    = ros::Time::now();
  

//Removing outliers using a Conditional removal


 
   // Publish the data
  pub_pointcloud2.publish (output2);
  
}
 
int main(int argc, char** argv)
{
   
  
   
   
    
    ros::init(argc, argv, "GuidanceNodeTest");
    ros::NodeHandle my_node;
    depth_image_sub       = my_node.subscribe("/guidance/depth_image_0", 10, depth_image_callback);
     imu_sub               = my_node.subscribe("/guidance/imu", 1, imu_callback);
    position_sub = my_node.subscribe("/guidance/position", 1, position_callback);
         // Create a ROS publisher for the output point cloud
   pub_pointcloud2 = my_node.advertise<sensor_msgs::PointCloud2> ("pointcloud_output", 1);
//pub_pointcloud2 = my_node.advertise<sensor_msgs::PointCloud2> ("cloud_in", 1);
          ros::Rate r(10);	
    while (ros::ok())
       {
              
                             
                /* cvmSet(P0,0,0,0);
		  cvmSet(P0,1,0,0);
		  cvmSet(P0,2,0,1);
             
                 cvGEMM(Rotation_guidance,&cloud,1,NULL,0,cloud_rotation);
           printf("P[111111]====%f\n",cvmGet(P,0,0));
              printf("P[22222222]====%f\n",cvmGet(P,1,0));
                 printf("P333333]====%f\n",cvmGet(P,2,0));
          */
         // cvGEMM(Rotation_guidance,&cloud,1,NULL,0,cloud_rotation);
        ros::spinOnce();
        r.sleep();
        }
        
         cvReleaseMat(&q_guidance);
        
	cvReleaseMat(&Rotation_guidance);
    return 0;
}
