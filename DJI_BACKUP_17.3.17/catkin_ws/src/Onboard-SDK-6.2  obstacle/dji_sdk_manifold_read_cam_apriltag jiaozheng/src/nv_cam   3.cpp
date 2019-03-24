#include "Apriltag_detection_wqf.h"
//#include "stdafx.h"
 #include "stdlib.h"
 #include"stdio.h"
 #include "math.h"
using namespace std;
//using namespace AprilTags;
float a=-1;
float b=-1;
float a1;
float b1;
float f=0.0;
float d=0.0;
int meiyou=0;
int zhongxinx=0;
int zhongxiny=0;
int zuoshang=0;
int youshang=0;
int chazhi=0;//
int chazhi1=0;
double BufferX[10]={1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0};
double BufferY[10];
double BufferZ[10];
int numb=0;//shibie shuliang
// utility function to provide current system time (used below in
// determining frame rate at which images are being processed)/


#include <iostream>
#include <cstring>
#include <vector>
#include <list>
#include <sys/time.h>
const string usage = "\n"
  "Usage:\n"
  "  apriltags_demo [OPTION...] [IMG1 [IMG2...]]\n"
  "\n"
  "Options:\n"
  "  -h  -?          Show help options\n"
  "  -a              Arduino (send tag ids over serial port)\n"
  "  -d              Disable graphics\n"
  "  -t              Timing of tag extraction\n"
  "  -C <bbxhh>      Tag family (default 36h11)\n"
  "  -D <id>         Video device ID (if multiple cameras present)\n"
  "  -F <fx>         Focal length in pixels\n"
  "  -W <width>      Image width (default 640, availability depends on camera)\n"
  "  -H <height>     Image height (default 360, availability depends on camera)\n"
  "  -S <size>       Tag size (square black frame) in meters\n"
  "  -E <exposure>   Manually set camera exposure (default auto; range 0-10000)\n"
  "  -G <gain>       Manually set camera gain (default auto; range 0-255)\n"
  "  -B <brightness> Manually set the camera brightness (default 128; range 0-255)\n"
  "\n";

const string intro = "\n"
    "April tags test code\n"
    "(C) 2012-2014 Massachusetts Institute of Technology\n"
    "Michael Kaess\n"
    "\n";


#ifndef __APPLE__
#define EXPOSURE_CONTROL // only works in Linux
#endif

#ifdef EXPOSURE_CONTROL
#include <libv4l2.h>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <errno.h>
#endif

// OpenCV library for easy access to USB camera and drawing of images
// on screen
#include "opencv2/opencv.hpp"

// April tags detector and various families that can be selected by command line option
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag16h5.h"
#include "AprilTags/Tag25h7.h"
#include "AprilTags/Tag25h9.h"
#include "AprilTags/Tag36h9.h"
#include "AprilTags/Tag36h11.h"
#include "AprilTags/TagFamily.h"
#include "AprilTags/TagDetection.h"
#include "AprilTags/Edge.h"
#include "AprilTags/FloatImage.h"
#include "AprilTags/GLine2D.h"
#include "AprilTags/GLineSegment2D.h"
#include "AprilTags/Gaussian.h"
#include "AprilTags/GrayModel.h"
#include "AprilTags/Gridder.h"
#include "AprilTags/Homography33.h"
#include "AprilTags/MathUtil.h"
#include "AprilTags/Quad.h"
#include "AprilTags/Segment.h"
#include "AprilTags/UnionFindSimple.h"
#include "AprilTags/XYWeight.h"
#include "AprilTags/pch.h"
// Needed for getopt / command line options processing
#include <unistd.h>
extern int optind;
extern char *optarg;

const char* windowName = "apriltags_demo";
int biubiu=0;
double tic() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}
#include <cmath>

#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;

/**
 * Normalize angle to be within the interval [-pi,pi].
 */
inline double standardRad(double t) {
  if (t >= 0.) {
    t = fmod(t+PI, TWOPI) - PI;
  } else {
    t = fmod(t-PI, -TWOPI) + PI;
  }
  return t;
}

/**
 * Convert rotation matrix to Euler angles
 */
void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll) {
    yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
    double c = cos(yaw);
    double s = sin(yaw);
    pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
    roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
}

class Demo {

  AprilTags::TagDetector* m_tagDetector;
  AprilTags::TagCodes m_tagCodes;

  bool m_draw; // draw image and April tag detections?
  bool m_arduino; // send tag detections to serial port?
  bool m_timing; // print timing information for each tag extraction call

  int m_width; // image size in pixels
  int m_height;
  double m_tagSize; // April tag side length in meters of square black frame
  double m_fx; // camera focal length in pixels
  double m_fy;
  double m_px; // camera principal point
  double m_py;

  int m_deviceId; // camera id (in case of multiple cameras)

  list<string> m_imgNames;

  cv::VideoCapture m_cap;

  int m_exposure;
  int m_gain;
  int m_brightness;

 // Serial m_serial;

public:

  // default constructor
  Demo() :
    // default settings, most can be modified through command line options (see below)
    
    m_tagDetector(NULL),
    m_tagCodes(AprilTags::tagCodes16h5),


    m_draw(true),
    m_arduino(false),
    m_timing(true),

    m_width(640),
    m_height(360),
    m_tagSize(0.166),
    m_fx(600),
    m_fy(600),
    m_px(m_width/2),
    m_py(m_height/2),

    m_exposure(-1),
    m_gain(-1),
    m_brightness(-1),

    m_deviceId(0)
  {}

  // changing the tag family
  void setTagCodes(string s) {
    if (s=="16h5") {
      m_tagCodes = AprilTags::tagCodes16h5;
    } else if (s=="25h7") {
      m_tagCodes = AprilTags::tagCodes25h7;
    } else if (s=="25h9") {
      m_tagCodes = AprilTags::tagCodes25h9;
    } else if (s=="36h9") {
      m_tagCodes = AprilTags::tagCodes36h9;
    } else if (s=="36h11") {
      m_tagCodes = AprilTags::tagCodes36h11;
    } else {
     // cout << "Invalid tag family specified" << endl;
      exit(1);
    }
  }

  // parse command line options to change default behavior
  void parseOptions(int argc, char* argv[]) {
    int c;
    while ((c = getopt(argc, argv, ":h?adtC:F:H:S:W:E:G:B:D:")) != -1) {
      // Each option character has to be in the string in getopt();
      // the first colon changes the error character from '?' to ':';
      // a colon after an option means that there is an extra
      // parameter to this option; 'W' is a reserved character
      switch (c) {
      case 'h':
      case '?':
        cout << intro;
        cout << usage;
        exit(0);
        break;
      case 'a':
        m_arduino = true;
        break;
      case 'd':
        m_draw = false;
        break;
      case 't':
        m_timing = true;
        break;
      case 'C':
        setTagCodes(optarg);
        break;
      case 'F':
        m_fx = atof(optarg);
        m_fy = m_fx;
        break;
      case 'H':
        m_height = atoi(optarg);
        m_py = m_height/2;
         break;
      case 'S':
        m_tagSize = atof(optarg);
        break;
      case 'W':
        m_width = atoi(optarg);
        m_px = m_width/2;
        break;
      case 'E':
#ifndef EXPOSURE_CONTROL
        cout << "Error: Exposure option (-E) not available" << endl;
        exit(1);
#endif
        m_exposure = atoi(optarg);
        break;
      case 'G':
#ifndef EXPOSURE_CONTROL
        cout << "Error: Gain option (-G) not available" << endl;
        exit(1);
#endif
        m_gain = atoi(optarg);
        break;
      case 'B':
#ifndef EXPOSURE_CONTROL
        cout << "Error: Brightness option (-B) not available" << endl;
        exit(1);
#endif
        m_brightness = atoi(optarg);
        break;
      case 'D':
        m_deviceId = atoi(optarg);
        break;
      case ':': // unknown option, from getopt
        cout << intro;
        cout << usage;
        exit(1);
        break;
      }
    }
if (argc > optind) {
      for (int i=0; i<argc-optind; i++) {
        m_imgNames.push_back(argv[optind+i]);
      }
    }
  }
    
  void setup() {
    m_tagDetector = new AprilTags::TagDetector(m_tagCodes);     //BUG

    // prepare window for drawing the camera images
   if (m_draw) {
     cv::namedWindow(windowName, 1);
    }

  }
   float maxm (float shu1,float shu2,float shu3,float shu4)const
{
		float maxx=shu1;
		if(shu2>shu1)
	{
		maxx=shu2;
	}
		if(shu3>maxx)
	{
		maxx=shu3;
	}
		if(shu4>maxx)
	{
		maxx=shu4;
	}
	return maxx;
}
double max(float shu1,float shu2,float shu3,float shu4)const
{
	double maxx;
	maxx=(sqrt(shu1*shu1+shu2*shu2+shu3*shu3+shu4*shu4))/2.0;
	return maxx;
}
 void print_detection(AprilTags::TagDetection& detection) const {
  /*  cout << "  Id: " << detection.id
         << " (Hamming: " << detection.hammingDistance << ")";
*/
    // recovering the relative pose of a tag:
 
	a = detection.cxy.first;
	b = detection.cxy.second;
	if(a==-1)
	{
	}
	else
	{	
		a1=a;
		b1=b;
	d = maxm(detection.p[0].first,detection.p[1].first,detection.p[2].first,detection.p[3].first);
	f = maxm(detection.p[0].second,detection.p[1].second,detection.p[2].second,detection.p[3].second);
	}
	//printf("The x of the center point is:%f\n",a);
	//printf("The y of the center point is:%f\n",b);
       // printf("c = detection.p[0].first:%f\n",c);
	//printf("d = detection.p[3].first:%f\n",b);
    // NOTE: for this to be accurate, it is necessary to use the
    // actual camera parameters here as well as the actual tag size
    // (m_fx, m_fy, m_px, m_py, m_tagSize)/
/*
    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;
    detection.getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,
                                             translation, rotation);

    Eigen::Matrix3d F;
    F <<
      1, 0,  0,
      0,  -1,  0,
      0,  0,  1;
    Eigen::Matrix3d fixed_rot = F*rotation;
    double yaw, pitch, roll;
    wRo_to_euler(fixed_rot, yaw, pitch, roll);

    cout << "  distance=" << translation.norm()
         << "m, x=" << translation(0)
         << ", y=" << translation(1)
         << ", z=" << translation(2)
         << ", yaw=" << yaw
         << ", pitch=" << pitch
         << ", roll=" << roll
         << endl;
*/
    // Also note that for SLAM/multi-view application it is better to
    // use reprojection error of corner points, because the noise in
    // this relative pose is very non-Gaussian; see iSAM source code
    // for suitable factors.//
  }
  void processImage(cv::Mat& image) {

    double t0;
    if (m_timing) {
      t0 = tic();
    }
     a=-1;
     b=-1;
vector<AprilTags::TagDetection> detections= m_tagDetector->extractTags(image);
    if (m_timing) {
      double dt = tic()-t0;
     // cout << "Extracting tags took " << dt << " seconds." << endl;
    }
    if(detections.size()==1)
    {
    	meiyou=3;
    }
    else{
         meiyou=meiyou-1;
            // zuoshang=youshang=0;
    }
    //printf("meiyou1111=%d\n",meiyou);
    // print out each detection
    cout << detections.size() << " tags detected:" << endl;
    for (int i=0; i<detections.size(); i++) {
      print_detection(detections[i]);
    }
   
    // show the current image including any detections
   if (m_draw) {
      for (int i=0; i<detections.size(); i++) {
        // also highlight in the image
       // printf("1111001\n");
        detections[i].draw(image);
      }
      //imshow(windowName, image); // OpenCV /call
    }

  }
  // Load and process a single image
  void loadImages() {               //DEBUG
    cv::Mat image;
    cv::Mat image_gray;
    IplImage *img=cvLoadImage("/home/ubuntu/apriltags/example/123.jpg");
    image=cv::Mat(img);
   
  }
 
}; // Demo
//////rotation

#include <stdio.h>
#include <stdlib.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include  <opencv/cvaux.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <linux/fb.h>
#include <string.h>
#include <opencv/cv.h>
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <malloc.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <poll.h>
#include <signal.h>
#include <assert.h>
#include <sys/types.h>
#include <unistd.h>
#include "cv.h"
#include "highgui.h"
#include "djicam.h" 
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <time.h>
#include <ctype.h>
#include <dji_sdk/dji_drone.h>
#include <geometry_msgs/Twist.h>
#include "image2world.h"
#include "Apriltag_detection_wqf.h"
//#include <cstdlib>
//#include <actionlib/client/simple_action_client.h>
//#include <actionlib/client/terminal_state.h>

#include <geometry_msgs/TransformStamped.h> //IMU
#include <geometry_msgs/Vector3Stamped.h> //velocity
#include <sensor_msgs/LaserScan.h> //obstacle distance && ultrasonic

ros::Publisher Ford_pub;   //publish Ford150 message
typedef unsigned char   BYTE;
#define IMAGE_W 1280
#define IMAGE_H 720
#define FRAME_SIZE              IMAGE_W*IMAGE_H*3

unsigned char buffer[FRAME_SIZE] = {0};
unsigned int frame_size = 0;
unsigned int nframe = 0;
//FILE *fp;


#define mode GETBUFFER_MODE


ros::Subscriber ultrasonic_sub;
ros::Subscriber position_sub;
double uav_sonor=0.0;
void ultrasonic_callback(const sensor_msgs::LaserScan& g_ul)
{ 
    //printf( "frame_id: %s stamp: %d\n", g_ul.header.frame_id.c_str(), g_ul.header.stamp.sec );
   // for (int i = 0; i < 5; i++)
      //  printf( "ultrasonic distance: [%f]  reliability: [%d]\n", g_ul.ranges[i], (int)g_ul.intensities[i] );
     
         uav_sonor=Low_Pass(g_ul.ranges[0]);
       // printf("hhhhhhh===%f\n",uav_sonor);
}
   
/* motion */
void position_callback(const geometry_msgs::Vector3Stamped& g_pos)
{
	//printf("frame_id: %s stamp: %d\n", g_pos.header.frame_id.c_str(), g_pos.header.stamp.sec);
	//for (int i = 0; i < 5; i++)
		//printf("global position: [%f %f %f]\n", g_pos.vector.x, g_pos.vector.y, g_pos.vector.z);
}

struct sRGB{
	int r;
	int g;
	int b;
};

sRGB yuvTorgb(int Y, int U, int V)
{
	sRGB rgb;
	rgb.r = (int)(Y + 1.4075 * (V-128));
	rgb.g = (int)(Y - 0.3455 * (U-128) - 0.7169*(V-128));
	rgb.b = (int)(Y + 1.779 * (U-128));
	rgb.r =(rgb.r<0? 0: rgb.r>255? 255 : rgb.r);
	rgb.g =(rgb.g<0? 0: rgb.g>255? 255 : rgb.g);
	rgb.b =(rgb.b<0? 0: rgb.b>255? 255 : rgb.b);
	return rgb;
}
unsigned char * NV12ToRGB(unsigned char * src, unsigned char * rgb, int width, int height){
	int numOfPixel = width * height;
	int positionOfU = numOfPixel;
	int startY,step,startU,Y,U,V,index,nTmp;
	sRGB tmp;

	for(int i=0; i<height; i++){
		startY = i*width;
		step = i/2*width;
		startU = positionOfU + step;
		for(int j = 0; j < width; j++){
			Y = startY + j;
			if(j%2 == 0)
				nTmp = j;
			else
				nTmp = j - 1;
			U = startU + nTmp;
			V = U + 1;
			index = Y*3;
			tmp = yuvTorgb((int)src[Y], (int)src[U], (int)src[V]);
			rgb[index+0] = (char)tmp.b;
			rgb[index+1] = (char)tmp.g;
			rgb[index+2] = (char)tmp.r;
		}
	}
	return rgb;
}
bool YUV420_To_BGR24(unsigned char *puc_y, unsigned char *puc_u, unsigned char *puc_v, unsigned char *puc_rgb, int width_y, int height_y)
{
	if (!puc_y || !puc_u || !puc_v || !puc_rgb)
	{
		return false;
	}
	int baseSize = width_y * height_y;
	int rgbSize = baseSize * 3;

	BYTE* rgbData = new BYTE[rgbSize];
	memset(rgbData, 0, rgbSize);

	int temp = 0;

	BYTE* rData = rgbData; 
	BYTE* gData = rgbData + baseSize;
	BYTE* bData = gData + baseSize;

	int uvIndex =0, yIndex =0;


	for(int y=0; y < height_y; y++)
	{
		for(int x=0; x < width_y; x++)
		{
			uvIndex = (y>>1) * (width_y>>1) + (x>>1);
			yIndex = y * width_y + x;

			temp = (int)(puc_y[yIndex] + (puc_v[uvIndex] - 128) * 1.4022);
			rData[yIndex] = temp<0 ? 0 : (temp > 255 ? 255 : temp);

			temp = (int)(puc_y[yIndex] + (puc_u[uvIndex] - 128) * (-0.3456) +
					(puc_v[uvIndex] - 128) * (-0.7145));
			gData[yIndex] = temp < 0 ? 0 : (temp > 255 ? 255 : temp);

			temp = (int)(puc_y[yIndex] + (puc_u[uvIndex] - 128) * 1.771);
			bData[yIndex] = temp < 0 ? 0 : (temp > 255 ? 255 : temp);
		}
	}

	int widthStep = width_y*3;
	for (int y = 0; y < height_y; y++)
	{
		for (int x = 0; x < width_y; x++)
		{
			puc_rgb[y * widthStep + x * 3 + 2] = rData[y * width_y + x]; //R
			puc_rgb[y * widthStep + x * 3 + 1] = gData[y * width_y + x]; //G
			puc_rgb[y * widthStep + x * 3 + 0] = bData[y * width_y + x]; //B
		}
	}

	if (!puc_rgb)
	{
		return false;
	}
	delete [] rgbData;
	return true;
}

IplImage* YUV420_To_IplImage(unsigned char* pYUV420, int width, int height)
{
	if (!pYUV420)
	{
		return NULL;
	}

	int baseSize = width*height;
	int imgSize = baseSize*3;
	BYTE* pRGB24 = new BYTE[imgSize];
	memset(pRGB24, 0, imgSize);

	int temp = 0;

	BYTE* yData = pYUV420; 
	BYTE* uData = pYUV420 + baseSize; 
	BYTE* vData = uData + (baseSize>>2); 

	if(YUV420_To_BGR24(yData, uData, vData, pRGB24, width, height) == false || !pRGB24)
	{
		return NULL;
	}

	IplImage *image = cvCreateImage(cvSize(width, height), 8,3);
	memcpy(image->imageData, pRGB24, imgSize);

	if (!image)
	{
		return NULL;
	}

	delete [] pRGB24;
	return image;
}


#define ParaBuffer(Buffer,Row,Col) (*(Buffer + (Row) * (SizeSrc + 1) + (Col)))


static int PrintPara(double* Para, int SizeSrc)
 {
         int i, j;
         for (i = 0; i < SizeSrc; i++)
         {
                 for (j = 0; j <= SizeSrc; j++)
                         printf("%10.6lf ", ParaBuffer(Para, i, j));
                 printf("\r\n");
         }
         printf("\r\n");
         return 0;
 }


static int ParalimitRow(double* Para, int SizeSrc, int Row)
 {
         int i;
         double Max, Min, Temp;
         for (Max = abs(ParaBuffer(Para, Row, 0)), Min = Max, i = SizeSrc; i; i--)
         {
                 Temp = abs(ParaBuffer(Para, Row, i));
                 if (Max < Temp)
                         Max = Temp;
                 if (Min > Temp)
                         Min = Temp;
         }
         Max = (Max + Min) * 0.000005;
         for (i = SizeSrc; i >= 0; i--)
                 ParaBuffer(Para, Row, i) /= Max;
         return 0;
 }


static int Paralimit(double* Para, int SizeSrc)
 {
         int i;
         for (i = 0; i < SizeSrc; i++)
                 if (ParalimitRow(Para, SizeSrc, i))
                         return -1;
         return 0;
 }


static int ParaPreDealA(double* Para, int SizeSrc, int Size)
 {
         int i, j;
         for (Size -= 1, i = 0; i < Size; i++)
         {
                 for (j = 0; j < Size; j++)
                         ParaBuffer(Para, i, j) = ParaBuffer(Para, i, j) * ParaBuffer(Para, Size, Size) - ParaBuffer(Para, Size, j) * ParaBuffer(Para, i, Size);
                 ParaBuffer(Para, i, SizeSrc) = ParaBuffer(Para, i, SizeSrc) * ParaBuffer(Para, Size, Size) - ParaBuffer(Para, Size, SizeSrc) * ParaBuffer(Para, i, Size);
                 ParaBuffer(Para, i, Size) = 0;
                 ParalimitRow(Para, SizeSrc, i);
         }
         return 0;
 }


static int ParaDealA(double* Para, int SizeSrc)
 {
         int i;
         for (i = SizeSrc; i; i--)
                 if (ParaPreDealA(Para, SizeSrc, i))
                         return -1;
         return 0;
 }


static int ParaPreDealB(double* Para, int SizeSrc, int OffSet)
 {
         int i, j;
         for (i = OffSet + 1; i < SizeSrc; i++)
         {
                 for (j = OffSet + 1; j <= i; j++)
                         ParaBuffer(Para, i, j) *= ParaBuffer(Para, OffSet, OffSet);
                 ParaBuffer(Para, i, SizeSrc) = ParaBuffer(Para, i, SizeSrc) * ParaBuffer(Para, OffSet, OffSet) - ParaBuffer(Para, i, OffSet) * ParaBuffer(Para, OffSet, SizeSrc);
                 ParaBuffer(Para, i, OffSet) = 0;
                 ParalimitRow(Para, SizeSrc, i);
         }
         return 0;
 }


static int ParaDealB(double* Para, int SizeSrc)
 {
         int i;
         for (i = 0; i < SizeSrc; i++)
                 if (ParaPreDealB(Para, SizeSrc, i))
                         return -1;/////
         for (i = 0; i < SizeSrc; i++)
         {
                 if (ParaBuffer(Para, i, i))
                 {
                         ParaBuffer(Para, i, SizeSrc) /= ParaBuffer(Para, i, i);
                         ParaBuffer(Para, i, i) = 1.0;
                 }
         }
         return 0;
 }


static int ParaDeal(double* Para, int SizeSrc)
 {
         PrintPara(Para, SizeSrc);
         Paralimit(Para, SizeSrc);
         PrintPara(Para, SizeSrc);
         if (ParaDealA(Para, SizeSrc))
                 return -1;
         PrintPara(Para, SizeSrc);
         if (ParaDealB(Para, SizeSrc))
                 return -1;
         return 0;
 }


static int GetParaBuffer(double* Para, const double* X, const double* Y, int Amount, int SizeSrc)
 {
         int i, j;
         for (i = 0; i < SizeSrc; i++)
                 for (ParaBuffer(Para, 0, i) = 0, j = 0; j < Amount; j++)
                         ParaBuffer(Para, 0, i) += pow(*(X + j), 2 * (SizeSrc - 1) - i);
         for (i = 1; i < SizeSrc; i++)
                 for (ParaBuffer(Para, i, SizeSrc - 1) = 0, j = 0; j < Amount; j++)
                         ParaBuffer(Para, i, SizeSrc - 1) += pow(*(X + j), SizeSrc - 1 - i);
         for (i = 0; i < SizeSrc; i++)
                 for (ParaBuffer(Para, i, SizeSrc) = 0, j = 0; j < Amount; j++)
                         ParaBuffer(Para, i, SizeSrc) += (*(Y + j)) * pow(*(X + j), SizeSrc - 1 - i);
         for (i = 1; i < SizeSrc; i++)
                 for (j = 0; j < SizeSrc - 1; j++)
                         ParaBuffer(Para, i, j) = ParaBuffer(Para, i - 1, j + 1);
         return 0;
 }


int Cal(const double* BufferX, const double* BufferY, int Amount, int SizeSrc, double* ParaResK)
 {
         double* ParaK = (double*)malloc(SizeSrc * (SizeSrc + 1) * sizeof(double));
         GetParaBuffer(ParaK, BufferX, BufferY, Amount, SizeSrc);
         ParaDeal(ParaK, SizeSrc);
         for (Amount = 0; Amount < SizeSrc; Amount++, ParaResK++)
                 *ParaResK = ParaBuffer(ParaK, Amount, SizeSrc);
         free(ParaK);
         return 0;
 }


 int zuixiaoerchengfa(double BufferX[10], double BufferY[10])
 {

        int Amount;
        // BufferX[10]={1,2,3,4,5,6,7,8,9,10};
         //BufferY[10]={1,2,3,4,5,6,7,8,9,10};
         double ParaK[6];
         Amount =10;
        Cal((const double*)BufferX, (const double*)BufferY, Amount, sizeof(ParaK) / sizeof(double), (double*)ParaK);
         
        for (Amount = 0; Amount < sizeof(ParaK) / sizeof(double); Amount++)
                 {printf("ParaK[%d] = %lf!\r\n", Amount, ParaK[Amount]);}
	int fanhuizhi=ParaK[0]*11*11*11*11*11+ParaK[1]*11*11*11*11+ParaK[2]*11*11*11+ParaK[3]*11*11+ParaK[4]*11+ParaK[5];

       // system("pause");
         return fanhuizhi;
 }
FILE *fp=fopen("/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk_manifold_read_cam_apriltag/data/fps.txt","w");
int main(int argc, char **argv)
{       
	
	
        Demo demo;
  // process command line options
      //  demo.parseOptions(argc, argv);
        demo.setup();
	    ros::init(argc,argv,"image_raw");
        ros::NodeHandle nh;
        DJIDrone* drone = new DJIDrone(nh);
       // drone->request_sdk_permission_control();//obtain control
	int ret,nKey;
	int nState = 1;
	int nCount = 1;
	int gray_or_rgb = 0;

	IplImage *pRawImg;
	IplImage *pImg;
	unsigned char *pData;
	ros::NodeHandle nh_private("~");
      nh_private.param("gray_or_rgb", gray_or_rgb, 0);

	//printf("gray_or_rgb==%d\n",gray_or_rgb);
	if(gray_or_rgb){
		pRawImg = cvCreateImage(cvSize(IMAGE_W, IMAGE_H),IPL_DEPTH_8U,3);
		pImg = cvCreateImage(cvSize(640, 360),IPL_DEPTH_8U,3);
		pData  = new unsigned char[1280 * 720 * 3];
	} else{
		pRawImg = cvCreateImage(cvSize(IMAGE_W, IMAGE_H),IPL_DEPTH_8U,1);
		pImg = cvCreateImage(cvSize(640, 360),IPL_DEPTH_8U,1);
	}
        
	 ros::NodeHandle nh_Ford;
	 Ford_pub=nh_Ford.advertise<geometry_msgs::Twist>("Ford150/info",1);//Ford150
	 geometry_msgs::Twist Ford_info;
	  // Ford_info.header.frame_id="/ground";
	    
	    
	  ros::NodeHandle my_node;
	  ultrasonic_sub = my_node.subscribe("/guidance/ultrasonic", 1, ultrasonic_callback);
    	position_sub = my_node.subscribe("/guidance/position", 1, position_callback);
    
        ros::NodeHandle node;
	image_transport::ImageTransport transport(node);
	image_transport::Publisher image_pub = transport.advertise("dji_sdk/image_raw", 1);
	ros::Publisher caminfo_pub = node.advertise<sensor_msgs::CameraInfo>("dji_sdk/camera_info",1);
	
	ros::Time time=ros::Time::now();

	cv_bridge::CvImage cvi;
	sensor_msgs::Image im;
	sensor_msgs::CameraInfo cam_info;
	cam_info.header.frame_id = "/camera";
	cam_info.height = IMAGE_H/2;
	cam_info.width = IMAGE_W/2;
	cam_info.distortion_model = "";
	cam_info.D.push_back(-0.1297646493949856);
	cam_info.D.push_back(0.0946885697670611);
	cam_info.D.push_back(-0.0002935002712265514);
	cam_info.D.push_back(-0.00022663675362156343);
	cam_info.D.push_back(0.0);
	cam_info.K = {340.005646, 0.0, 326.819977, 0.0, 333.288971, 171.340088, 0.0, 0.0, 1.0};
	cam_info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
	cam_info.P = {340.005646, 0.0, 326.819977, 0.0, 0.0, 333.288971, 171.340088, 0.0, 0.0, 0.0, 1.0, 0.0};
	cam_info.binning_x = 0;
	cam_info.binning_x = 0;

	cam_info.roi.x_offset = 0;
	cam_info.roi.y_offset = 0;
	cam_info.roi.height = 0;
	cam_info.roi.width = 0;
	cam_info.roi.do_rectify = false;
	int frame=1;
	double last_t=tic();
	ret = manifold_cam_init(mode);
	if(ret == -1)
	{
		//printf("manifold init error \n");
		return -1;
	}
   CvMat *pos_apriltag=cvCreateMat(3,1,CV_32FC1);
   CvMat *Rotation_camera2body=cvCreateMat(3,3,CV_32FC1);
	while(1)
	{
	   
	  	frame++;
		ret = manifold_cam_read(buffer, &nframe, 1);
		if(ret != -1)
		{

			if(gray_or_rgb){
				NV12ToRGB(buffer,pData,1280,720);
				memcpy(pRawImg->imageData,pData,FRAME_SIZE);
			}else{
				memcpy(pRawImg->imageData,buffer,FRAME_SIZE/3);
			}
			cvResize(pRawImg,pImg,CV_INTER_LINEAR);

			//time=ros::Time::now();
			//cvi.header.stamp = time;
			//cvi.header.frame_id = "image";

			if(gray_or_rgb){
				cvi.encoding = "bgr8";
			}else{
				cvi.encoding = "mono8";
			}
			//cvi.image = pImg;
			//cvi.toImageMsg(im);
			//cam_info.header.seq = nCount;
			//cam_info.header.stamp = time;
			//caminfo_pub.publish(cam_info);
			//image_pub.publish(im);
            
             		cv::Mat img;
                 	cv::Mat ppImg;
             	 	img=cv::Mat(pImg); 
              		int zhongxinxf,zhongxinxz,zhongxinyf,zhongxinyz;
               		cv::namedWindow("balala", 2);
               		cv::namedWindow(windowName, 1);
		if(meiyou==0||meiyou<0)
		{	
			//cvDestroyWindow("balala");//
           		demo.processImage(img);
            		zhongxinx=zhongxiny=0;
            		//imshow(windowName, img);
            		//printf("meiyou====0000\n");
            		zuoshang=youshang=0;

    		}
    		else
    		{
			chazhi=2*(4.5-meiyou)*abs((d-a));
			chazhi1=2*(4.5-meiyou)*abs((f-b));
			//printf("chazhi===%d\n",chazhi);
			if(chazhi<20)
			{
				chazhi=20;
			}
   			if(chazhi1<20)
			{
				chazhi1=20;
			}
    			if(a1+zuoshang-chazhi>0)
    			{
    				zhongxinxf=a1+zuoshang-chazhi-1;
    			}
    			else
    			{
    			zhongxinxf=1+zuoshang-1;
    			}
    			if(b1+youshang-chazhi1>0)/////////////////////////////////////////////////////bbbb
    			{
    				zhongxinyf=b1+youshang-chazhi1-1;
    			}
    			else
    			{
    				zhongxinyf=1+youshang-1;
    			}
    			if(b1+youshang+chazhi1<479)////////////
    			{
    				zhongxinyz=b1+youshang+chazhi1-1;
    			}
    			else
    			{
    				zhongxinyz=479;
    			}
    			if(a1+chazhi+zuoshang<639)/////////////////////////////
    			{
    				zhongxinxz=a1+zuoshang+chazhi-1;
    			}
    			else
    			{
    				zhongxinxz=639;
    			}
    			//printf("%f %f %d %d %d %d\n",a1,b1,zhongxinxf,zhongxinxz,zhongxinyf,zhongxinyz);
    			IplImage *spub=cvGetImage(cvGetSubRect(pImg,cvCreateMatHeader((zhongxinxz-zhongxinxf),(zhongxinyz-zhongxinyf),CV_8UC1),cvRect(zhongxinxf,zhongxinyf,(zhongxinxz-zhongxinxf),(zhongxinyz-zhongxinyf))),cvCreateImageHeader(cvSize((zhongxinxz-zhongxinxf),(zhongxinyz-zhongxinyf)),IPL_DEPTH_8U,1));
    		        
    			ppImg=cv::Mat(spub);
    				//printf("%d",ppImg.cols);
  		
    	       		demo.processImage(ppImg);
    	        	cvRectangle(pImg,cvPoint(zhongxinxf,zhongxinyf),cvPoint(zhongxinxz,zhongxinyz),CV_RGB(255,250,250),2,8,0);
    	        	cv::Mat pppImg;
    	        	pppImg=cv::Mat(pImg);
    	       		if(a==-1||b==-1)
    	       	 	{
    	            		zuoshang=youshang=0;
    	            	}
    	           	 else
    	            	{
    	             		 zuoshang=zhongxinxf;
    	             		 youshang=zhongxinyf;
    	        	}
    	       		//imshow(windowName, pppImg);
    	       		//imshow("balala", ppImg); // OpenCV call
    	            	///cvReleaseImage(&spub);
    	            	//cvReleaseMat(&pppImg);
    	        	//cvWaitKey(10);
    		}
         		//printf("chazhi===%d\n",chazhi);
    			//printf("zhongxinx==%d\n",zhongxinx);
			//camera frame to body frame
		    	// cvReleaseImage(&pImg);
		        // cvReleaseImage(&pRawImg);
		        //cvReleaseMat(&img);
		        // cvReleaseMat(&ppImg);
		     //	printf("u=%f,v=%f\n",a+zuoshang,b+youshang);
		 	Euler_To_Matrix(drone->gimbal.roll,drone->gimbal.pitch,drone->gimbal.yaw,Rotation_camera2body);
		    	//Euler_To_Matrix(0,0,0,Rotation_camera2body);
		    	if(a==-1&&b==-1)
		    	{
		          //printf("not detect  apriltags\n");
                           Ford_info.linear.x=-1;
                           Ford_info.linear.y=-1;
		    	}
		    	else
		    	{
		     	dianbianhuan(Rotation_camera2body,pos_apriltag,uav_sonor*100+3.5,a+zuoshang,b+youshang);
		      	//dianbianhuan(Rotation_camera2body,pos_apriltag,81.4,a+zuoshang,b+youshang);
                         Ford_info.linear.x=cvmGet(pos_apriltag,0,0); //Ford 150 position in x frame(pingxing to world frame's x,cm)
		      	Ford_info.linear.y=cvmGet(pos_apriltag,1,0);  //Ford 150 position in y frame(pingxing to world frame's y,cm)
	        	}	  
			        	
		      	Ford_info.linear.z=uav_sonor*100;  //Ford 150 position in y frame(pingxing to world frame's y,cm)
		      	Ford_info.angular.x=a+zuoshang; //apriltag that detected in u(pixel) 
		      	Ford_info.angular.y=b+youshang;//apriltag that detected in v(pixel) 
	          	Ford_pub.publish(Ford_info);
			if(frame % 10 ==0)
			{
				double t =tic();
				cout << " " << 10./(t-last_t) << " fps" << endl;
				fprintf(fp,"fps=%f",10./(t-last_t) );    
				last_t=t;
			}
			//printf("x=%f\t y=%f\t z=%f\t u=%f\t v=%f\t\n",Ford_info.linear.x,Ford_info.linear.y,Ford_info.linear.z,Ford_info.angular.x,Ford_info.angular.y);   
                        //fprintf(fp,"x=%f\t y=%f\t z=%f\t u=%f\t v=%f\t\n",Ford_info.linear.x,Ford_info.linear.y,Ford_info.linear.z,Ford_info.angular.x,Ford_info.angular.y);                       
			nCount++;                          
           		//cvWaitKey(10);
           		ros::spinOnce();//
		}
		else 
			break;

		//usleep(1000);
	}
	while(!manifold_cam_exit())
	{
		sleep(1);
	}
    		cvReleaseMat(&Rotation_camera2body);
     		cvReleaseMat(&pos_apriltag);
		fclose(fp);
		sleep(1);
	return 0;
}

