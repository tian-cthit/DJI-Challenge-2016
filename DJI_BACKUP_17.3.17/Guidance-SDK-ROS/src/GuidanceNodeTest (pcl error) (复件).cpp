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

ros::Subscriber depth_image_sub;
ros::Publisher pub_pointcloud2;

using namespace cv;
#define WIDTH 320
#define HEIGHT 240
const float camera_cu = 154.1;
const float camera_cv = 118.744;
const float focal = 251.683;
const float baseline = 0.150067;
const float camera_factor=255;

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
   // cv::imshow("depth_image", depth8);
  //  cv::waitKey(1);
       
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

     PointCloud::Ptr cloud ( new PointCloud );
     //pcl::PCLPointCloud2 cloud_filtered;
    // 遍历深度图
       printf("depth====....%f   wwwwwwwwwwww=%d\n",float(  cv_ptr->image.at<short>( HEIGHT/2,WIDTH/2))/128,depth8.ptr<ushort>(HEIGHT/2)[WIDTH/2]);
    for (int m = 0; m < depth8.rows; m++)
        for (int n=0; n < depth8.cols; n++)
        {
            // 获取深度图中(m,n)处的值
            ushort d = depth8.ptr<ushort>(m)[n];
            
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            PointT p;
                  printf("d=============%d\n",d/128);
            // 计算这个点的空间坐标
            p.z =  float( cv_ptr->image.at<short>(m,n))/128;
               // printf("d=============%f\n",p.z);
            p.x = (n - camera_cu) * p.z / focal;
            p.y = (m - camera_cv) * p.z / focal;          
            // 把p加入到点云中
            cloud->points.push_back( p );
        }
          
  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
   //transfrom pcl::PointXYZRGBA to sensor_msgs::PointCloud2   http://docs.ros.org/hydro/api/pcl_conversions/html/namespacepcl.html#abb8b3a2632e07dae0b541a257898c8a8
  pcl::toROSMsg(*cloud, output);
  //set frame_id
   output.header.frame_id = "guidance";
   output.header.stamp    = ros::Time::now();
  // Publish the data
   pub_pointcloud2.publish (output);
  
}

/*

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering 体素化网格滤波http://www.pclcn.org/study/shownews.php?lang=cn&id=67
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::moveFromPCL(cloud_filtered, output);

  // Publish the data
  pub_pointcloud2.publish (output);
}
*/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "GuidanceNodeTest");
    ros::NodeHandle my_node;
    depth_image_sub       = my_node.subscribe("/guidance/depth_image", 10, depth_image_callback);
         // Create a ROS publisher for the output point cloud
   pub_pointcloud2 = my_node.advertise<sensor_msgs::PointCloud2> ("pointcloud_output", 1);
    while (ros::ok())
        ros::spinOnce();

    return 0;
}
