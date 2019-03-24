
using namespace std;
float a=-1;
float b=-1;
int n_boards=10;
const int board_dt=20;
int board_w=11;
int board_h=8;
int meiyou=0;
int zhongxinx=0;
int zhongxiny=0;
// utility function to provide current system time (used below in
// determining frame rate at which images are being processed)


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
  "  -H <height>     Image height (default 480, availability depends on camera)\n"
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
    m_tagCodes(AprilTags::tagCodes36h11),


    m_draw(true),
    m_arduino(false),
    m_timing(true),

    m_width(640),
    m_height(480),
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
      cout << "Invalid tag family specified" << endl;
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
 
  void setupVideo() {

#ifdef EXPOSURE_CONTROL
    // manually setting camera exposure settings; OpenCV/v4l1 doesn't
    // support exposure control; so here we manually use v4l2 before
    // opening the device via OpenCV; confirmed to work with Logitech
    // C270; try exposure=20, gain=100, brightness=150

    string video_str = "/dev/video0";
    video_str[10] = '0' + m_deviceId;
    int device = v4l2_open(video_str.c_str(), O_RDWR | O_NONBLOCK);

    if (m_exposure >= 0) {
      // not sure why, but v4l2_set_control() does not work for
      // V4L2_CID_EXPOSURE_AUTO...
      struct v4l2_control c;
      c.id = V4L2_CID_EXPOSURE_AUTO;
      c.value = 1; // 1=manual, 3=auto; V4L2_EXPOSURE_AUTO fails...
      if (v4l2_ioctl(device, VIDIOC_S_CTRL, &c) != 0) {
        cout << "Failed to set... " << strerror(errno) << endl;
      }
      cout << "exposure: " << m_exposure << endl;
      v4l2_set_control(device, V4L2_CID_EXPOSURE_ABSOLUTE, m_exposure*6);
    }
    if (m_gain >= 0) {
      cout << "gain: " << m_gain << endl;
      v4l2_set_control(device, V4L2_CID_GAIN, m_gain*256);
    }
    if (m_brightness >= 0) {
      cout << "brightness: " << m_brightness << endl;
      v4l2_set_control(device, V4L2_CID_BRIGHTNESS, m_brightness*256);
    }
    v4l2_close(device);
#endif 

    // find and open a USB camera (built in laptop camera, web cam etc)
    m_cap = cv::VideoCapture(m_deviceId);
        if(!m_cap.isOpened()) {
      cerr << "ERROR: Can't find video device " << m_deviceId << "\n";
      exit(1);
    }
    m_cap.set(CV_CAP_PROP_FRAME_WIDTH, m_width);
    m_cap.set(CV_CAP_PROP_FRAME_HEIGHT, m_height);
    cout << "Camera successfully opened (ignore error messages above...)" << endl;
    cout << "Actual resolution: "
         << m_cap.get(CV_CAP_PROP_FRAME_WIDTH) << "x"
         << m_cap.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;


  }
 void print_detection(AprilTags::TagDetection& detection) const {
    cout << "  Id: " << detection.id
         << " (Hamming: " << detection.hammingDistance << ")";

    // recovering the relative pose of a tag:
 
	a = detection.cxy.first;
	b = detection.cxy.second;
	printf("The x of the center point is:%f/n",a);
	printf("The y of the center point is:%f/n",b);
    // NOTE: for this to be accurate, it is necessary to use the
    // actual camera parameters here as well as the actual tag size
    // (m_fx, m_fy, m_px, m_py, m_tagSize)

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

    // Also note that for SLAM/multi-view application it is better to
    // use reprojection error of corner points, because the noise in
    // this relative pose is very non-Gaussian; see iSAM source code
    // for suitable factors.
  }

  void processImage(cv::Mat& image) {

    double t0;
    if (m_timing) {
      t0 = tic();
    }
     a=-1;
     b=-1;
vector<AprilTags::TagDetection> detections= m_tagDetector->extractTags(image);
    /*if (m_timing) {
      double dt = tic()-t0;
      cout << "Extracting tags took " << dt << " seconds." << endl;
    }*/
    if(detections.size()==0)
    {
    	meiyou=0;
    }
    else{meiyou=1;}
    printf("%d",meiyou);
    // print out each detection
    cout << detections.size() << " tags detected:" << endl;
    for (int i=0; i<detections.size(); i++) {
      print_detection(detections[i]);
    }
   
    // show the current image including any detections
   if (m_draw) {
      for (int i=0; i<detections.size(); i++) {
        // also highlight in the image
        detections[i].draw(image);
      }
      //imshow(windowName, image); // OpenCV call
    }

  }


  // Load and process a single image
  void loadImages() {               //DEBUG
    cv::Mat image;
    cv::Mat image_gray;
    IplImage *img=cvLoadImage("/home/ubuntu/apriltags/example/123.jpg");
    image=cv::Mat(img);
   
  }
// Video or image processing?
  bool isVideo() {
    return m_imgNames.empty();
  }

  // Video or image processing?
  void loop() {
    cv::Mat image;
    //cv::Mat image_gray;
    int frame = 0;
    double last_t = tic();
    while (true) {

      // capture frame
    //image=Aimage;
         m_cap >> image;
    //  processImage(image);
      // print out the frame rate at which image frames are being processed
      frame++;
      if (frame % 10 == 0) {
        double t = tic();
        cout << "  " << 10./(t-last_t) << " fps" << endl;
        last_t = t;
      }

      // exit if any key is pressed
      if (cv::waitKey(1) >= 0) break;
    }
  }

}; // Demo
//////rotation

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include  <opencv/cvaux.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <linux/fb.h>
#include <string.h>

#include <string.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <geometry_msgs/TransformStamped.h> //IMU
#include <geometry_msgs/Vector3Stamped.h> //velocity
#include <sensor_msgs/LaserScan.h> //obstacle distance && ultrasonic
ros::Subscriber ultrasonic_sub;
ros::Subscriber position_sub;
double uav_sonor=0.0;
void ultrasonic_callback(const sensor_msgs::LaserScan& g_ul)
{ 
    //printf( "frame_id: %s stamp: %d\n", g_ul.header.frame_id.c_str(), g_ul.header.stamp.sec );
    for (int i = 0; i < 5; i++)
      //  printf( "ultrasonic distance: [%f]  reliability: [%d]\n", g_ul.ranges[i], (int)g_ul.intensities[i] );
        uav_sonor=g_ul.ranges[0];
        printf("hhhhhhh===%f",uav_sonor);
}
   
/* motion */
void position_callback(const geometry_msgs::Vector3Stamped& g_pos)
{
	//printf("frame_id: %s stamp: %d\n", g_pos.header.frame_id.c_str(), g_pos.header.stamp.sec);
	//for (int i = 0; i < 5; i++)
		//printf("global position: [%f %f %f]\n", g_pos.vector.x, g_pos.vector.y, g_pos.vector.z);
}

void Quaternion_To_Euler(CvMat *q_att, CvMat *att);
void Euler_To_Matrix(float roll, float pitch, float yaw, CvMat *R);
void dianbianhuan(CvMat* R,CvMat* P,double h,int u,int v);
/*
void Quaternion_To_Euler(CvMat *q_att, CvMat *att)
{
		float r11,r12,r21,r31,r32,r1,r2,r3;
		float q[4] = { cvmGet(q_att,0,0),  cvmGet(q_att,1,0), cvmGet(q_att,2,0), cvmGet(q_att,3,0)};
    	r11 = 2.0f *(q[1] * q[2] + q[0] * q[3]);
    	r12 = q[0] * q[0] + q[1] * q[1] - q[2] * q[2]  - q[3] * q[3] ;
    	r21 = -2.0f * (q[1] * q[3] - q[0] * q[2]);
    	r31 = 2.0f *( q[2] * q[3] + q[0]  * q[1]);
    	r32 = q[0] * q[0] - q[1] * q[1]  - q[2] * q[2] + q[3] * q[3] ;
    	float yaw= atan2( r11, r12 );
    	float pitch = asin( r21 );
    	float roll = atan2( r31, r32 );
    	cvmSet(att, 0, 0, roll);
    	cvmSet(att, 1, 0, pitch);
    	cvmSet(att, 2, 0, yaw);
//printf("%f  \n",asin(r21));
//printf("%f  \n",r21);
}
*/
//euler to matrix  the matrix is from body to ground
void Euler_To_Matrix(float roll, float pitch, float yaw, CvMat *R)
{
    pitch=pitch/180.0*PI;
    roll=roll/180.0*PI;
    yaw=yaw/180.0*PI;
	float cp = cosf(pitch);
	float sp = sinf(pitch);
	float sr = sinf(roll);
	float cr = cosf(roll);
	float sy = sinf(yaw);
	float cy = cosf(yaw);
	
	cvmSet(R,0, 0,cp * cy);
	cvmSet(R,0, 1, ((sr * sp * cy) - (cr * sy)));
	cvmSet(R,0, 2, ((cr * sp * cy) + (sr * sy)));
	cvmSet(R,1, 0, (cp * sy));
	cvmSet(R,1,1, ((sr * sp * sy) + (cr * cy)));
	cvmSet(R,1, 2, ( (cr * sp * sy) - (sr * cy)));
	cvmSet(R,2, 0,  -sp);
	cvmSet(R,2, 1,  sr * cp);
	cvmSet(R,2, 2,  cr * cp);
   // printf("%f	%f	%f\n",cvmGet(R,0,0),cvmGet(R,0,1),cvmGet(R,0,2));

}
void dianbianhuan(CvMat* R,CvMat* P,double h,int u,int v)
{
	//CvMat* intrinsic=(CvMat*)cvLoad("/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk_manifold_read_cam_apriltag/calibration/Intrinsics.xml");
	CvMat* UV=cvCreateMat(3,1,CV_32FC1);
	CvMat* vc=cvCreateMat(3,3,CV_32FC1);
	CvMat* MR=cvCreateMat(3,3,CV_32FC1);
	CvMat* inverse=cvCreateMat(3,3,CV_32FC1);
	CvMat* UV_inverse=cvCreateMat(3,1,CV_32FC1);
	CvMat* R_add=cvCreateMat(3,3,CV_32FC1);
	CvMat* R_new=cvCreateMat(3,3,CV_32FC1);
	
	
	cvmSet(R_add,0,0,0);
	cvmSet(R_add,0,1,1);
	cvmSet(R_add,0,2,0);
	cvmSet(R_add,1,0,0);
	cvmSet(R_add,1,1,0);
	cvmSet(R_add,1,2,1);
	cvmSet(R_add,2,0,1);
	cvmSet(R_add,2,1,0);
	cvmSet(R_add,2,2,0);

	
	double s;
	CvMat* M=cvCreateMat(3,3,CV_32FC1);
	 
	cvmSet(M,0,0,384.226074);
	cvmSet(M,0,1,0);
	cvmSet(M,0,2,330.588959);
	cvmSet(M,1,0,0);
	cvmSet(M,1,1,505.548035);
	cvmSet(M,1,2,238.461639);
	cvmSet(M,2,0,0);
	cvmSet(M,2,1,0);
	cvmSet(M,2,2,1);
	cvTranspose(R,R);
	cvGEMM(R_add,R,1,NULL,0,R_new);
	//printf("R_new\n ");
	/*for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			     printf("%f\t",cvmGet(R_new,i,j));
		}
		printf("\n");
	}*/
    cvGEMM(M,R_new,1,NULL,0,MR);
    
	//为图像坐标赋值
	cvmSet(UV,0,0,u);
	cvmSet(UV,1,0,v);
	cvmSet(UV,2,0,1);
      
      
	//为加权阵赋值
	cvZero(vc);
	cvInvert(MR,inverse,CV_LU);
	 // printf("inverse\n");
    /* for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			     printf("%f\t",cvmGet(inverse,i,j));
		}
		printf("\n");
	}*/
	//cvGEMM(intrinsic,UV,1,vc,0,UV);
	cvGEMM(inverse,UV,1,vc,0,UV_inverse);
	 // printf("UV_inverse\n");
   /*for(int i=0;i<3;i++)
	{
		for(int j=0;j<1;j++)
		{
			     printf("%f\t",cvmGet(UV_inverse,i,j));
		}
		printf("\n");
	}*/
	double uvx,uvy,uvz;
	uvx=cvmGet(UV_inverse,0,0);
	uvy=cvmGet(UV_inverse,1,0);
	uvz=cvmGet(UV_inverse,2,0);
	s=uvz/(h-8);
	cvmSet(P,0,0,uvx/s);
	cvmSet(P,1,0,uvy/s);
	cvmSet(P,2,0,uvz/s);

	printf("P\n");
	for(int i=0;i<3;i++)
	{
		
			printf("%f\t",cvmGet(P,i,0));
		
	}
	printf("h====%f\n",h);
		printf("uvx====%f\n",uvx);
		printf("uvy====%f\n",uvy);
		  printf("s====%f\n",s);
		printf("\n");
	cvReleaseMat(&inverse);
	cvReleaseMat(&UV);
	cvReleaseMat(&vc);
	cvReleaseMat(&MR);
	cvReleaseMat(&UV_inverse);
}


#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <stdio.h>
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
//#include <cstdlib>
//#include <actionlib/client/simple_action_client.h>
//#include <actionlib/client/terminal_state.h>
// int calibrate();
ros::Publisher Ford_pub;   //publish Ford150 message
typedef unsigned char   BYTE;
#define IMAGE_W 1280
#define IMAGE_H 720
#define FRAME_SIZE              IMAGE_W*IMAGE_H*3


unsigned char buffer[FRAME_SIZE] = {0};
unsigned int frame_size = 0;
unsigned int nframe = 0;
FILE *fp;


#define mode GETBUFFER_MODE


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


/*int calibrate()
{
             int argc;
             char **argv;
             int n_boards=10;
             const int board_dt=1;
             int board_w=11;
       int board_h=8;

	int board_n=board_w*board_h;
	CvSize board_sz=cvSize(board_w,board_h);
	cvNamedWindow("Calibration");
	CvMat* image_points=cvCreateMat(n_boards*board_n,2,CV_32FC1);
	CvMat* object_points=cvCreateMat(n_boards*board_n,3,CV_32FC1);
	CvMat* point_counts=cvCreateMat(n_boards,1,CV_32SC1);
	CvMat* intrinsic_matrix=cvCreateMat(3,3,CV_32FC1);
	CvMat* distortion_coeffs=cvCreateMat(5,1,CV_32FC1);

	CvPoint2D32f* corners=new CvPoint2D32f[board_n];
	int corner_count;
	int successes=0;
	int step,frame=0;
	char c;
	//IplImage* gray_image=cvCreateImage(cvGetSize(image),8,1);
	     Demo demo;

  // process command line options
      //  demo.parseOptions(argc, argv);
           demo.setup();
	ros::init(argc,argv,"image_raw");
        ros::NodeHandle nh;
              DJIDrone* drone = new DJIDrone(nh);
              drone->request_sdk_permission_control();//obtain control
	int ret,nKey;
	int nState = 1;
	int nCount = 1;
	int gray_or_rgb = 0;

	IplImage *pRawImg;
	IplImage *pImg;
	unsigned char *pData;
	ros::NodeHandle nh_private("~");
      nh_private.param("gray_or_rgb", gray_or_rgb, 0);

	printf("gray_or_rgb==%d\n",gray_or_rgb);
	if(gray_or_rgb){
		pRawImg = cvCreateImage(cvSize(IMAGE_W, IMAGE_H),IPL_DEPTH_8U,3);
		pImg = cvCreateImage(cvSize(640, 480),IPL_DEPTH_8U,3);
		pData  = new unsigned char[1280 * 720 * 3];
	} else{
		pRawImg = cvCreateImage(cvSize(IMAGE_W, IMAGE_H),IPL_DEPTH_8U,1);
		pImg = cvCreateImage(cvSize(640, 480),IPL_DEPTH_8U,1);
	}
        
	 ros::NodeHandle nh_Ford;
	 Ford_pub=nh_Ford.advertise<geometry_msgs::Twist>("Ford150/info",1);//Ford150
	 geometry_msgs::Twist Ford_info;
	  // Ford_info.header.frame_id="/ground";
	    Ford_info.linear.x=0;
	    Ford_pub.publish(Ford_info);
	  
	  
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
	cam_info.K = {388.40923066779754, 0.0, 318.06257844065226, 0.0, 518.1538449374815, 241.17339016626644, 0.0, 0.0, 1.0};
	cam_info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
	cam_info.P = {373.5429992675781, 0.0, 317.51131336952494, 0.0, 0.0, 504.4360656738281, 240.6131009245937, 0.0, 0.0, 0.0, 1.0, 0.0};
	cam_info.binning_x = 0;
	cam_info.binning_x = 0;

	cam_info.roi.x_offset = 0;
	cam_info.roi.y_offset = 0;
	cam_info.roi.height = 0;
	cam_info.roi.width = 0;
	cam_info.roi.do_rectify = false;

	ret = manifold_cam_init(mode);
	if(ret == -1)
	{
		printf("manifold init error \n");
		return -1;
	}
   

	while(successes<n_boards&&ros::ok())
	{
		//image=cvQueryFrame(capture);
		
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

			time=ros::Time::now();
			cvi.header.stamp = time;
			cvi.header.frame_id = "image";

			if(gray_or_rgb){
				cvi.encoding = "bgr8";
			}else{
				cvi.encoding = "mono8";
			}
			cvi.image = pImg;
			cvi.toImageMsg(im);
			cam_info.header.seq = nCount;
			cam_info.header.stamp = time;
			caminfo_pub.publish(cam_info);
			image_pub.publish(im);
             
             cv::Mat img;
              img=cv::Mat(pImg); 

           // demo.processImage(img);
			ros::spinOnce();
			nCount++;                 
   
           drone->gimbal_speed_control(0,5*(240-b),-5*(320-a));       
           printf("speed yaw=%f, speed_pitch=%f\n",-(320-a),(240-b));
           cvWaitKey(10);
		}
		else 
			break;

		if(frame++%board_dt==0)
		{
			int found=cvFindChessboardCorners(pImg,board_sz,corners,&corner_count,CV_CALIB_CB_ADAPTIVE_THRESH|CV_CALIB_CB_FILTER_QUADS);

			//cvCvtColor(image,gray_image,CV_BGR2GRAY);
			cvFindCornerSubPix(pImg,corners,corner_count,cvSize(11,11),cvSize(-1,-1),cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,0.1));
			cvDrawChessboardCorners(pImg,board_sz,corners,corner_count,found);
			
			printf("Num of images collected = %d\n",successes);

			cvShowImage("Calibration",pImg);

			c = cvWaitKey(0);
			if(c!='p'){
				continue;
			}

			if(corner_count==board_n)
			{
				step=successes*board_n;
				for(int i=step,j=0;j<board_n;i++,j++)
				{
					CV_MAT_ELEM(*image_points,float,i,0)=corners[j].x;
					CV_MAT_ELEM(*image_points,float,i,1)=corners[j].y;
					CV_MAT_ELEM(*object_points,float,i,0)=j/board_w;
					CV_MAT_ELEM(*object_points,float,i,1)=j%board_w;
					CV_MAT_ELEM(*object_points,float,i,2)=0.0f;
				}
				CV_MAT_ELEM(*point_counts,int,successes,0)=board_n;
				successes++;
			}
		}

		if(c=='q'){
			cvReleaseMat(&intrinsic_matrix);
			cvReleaseMat(&distortion_coeffs);

			cvReleaseMat(&object_points);
			cvReleaseMat(&image_points);
			cvReleaseMat(&point_counts);

			delete[] corners;

			//cvReleaseImage(&gray_image);

			cvDestroyAllWindows();

			return -1;
		}
	}

	CvMat* object_points2=cvCreateMat(successes*board_n,3,CV_32FC1);
	CvMat* image_points2=cvCreateMat(successes*board_n,2,CV_32FC1);
	CvMat* point_counts2=cvCreateMat(successes,1,CV_32SC1);

	for(int i=0;i<successes*board_n;i++)
	{
		CV_MAT_ELEM(*image_points2,float,i,0)=CV_MAT_ELEM(*image_points,float,i,0);
		CV_MAT_ELEM(*image_points2,float,i,1)=CV_MAT_ELEM(*image_points,float,i,1);
		CV_MAT_ELEM(*object_points2,float,i,0)=CV_MAT_ELEM(*object_points,float,i,0);
		CV_MAT_ELEM(*object_points2,float,i,1)=CV_MAT_ELEM(*object_points,float,i,1);
		CV_MAT_ELEM(*object_points2,float,i,2)=CV_MAT_ELEM(*object_points,float,i,2);

	}

	for(int i=0;i<successes;i++)
	{
		CV_MAT_ELEM(*point_counts2,int,i,0)=CV_MAT_ELEM(*point_counts,int,i,0);

	}
	cvReleaseMat(&object_points);
	cvReleaseMat(&image_points);
	cvReleaseMat(&point_counts);

	CvMat* cam_rotation_all = cvCreateMat( successes, 3, CV_32FC1);
	CvMat* cam_translation_vector_all = cvCreateMat( successes,3, CV_32FC1);
	cvCalibrateCamera2(object_points2,image_points2,point_counts2,cvGetSize(pImg),intrinsic_matrix,distortion_coeffs,NULL,NULL,0);
	
	cvSave("/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk_manifold_read_cam_apriltag/calibration/Intrinsics.xml",intrinsic_matrix);
	cvSave("/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk_manifold_read_cam_apriltag/calibration/Distortion.xml",distortion_coeffs);
	cvSave("/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk_manifold_read_cam_apriltag/calibration/cam_rotation_all.xml",cam_rotation_all);
	cvSave("/home/ubuntu/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk_manifold_read_cam_apriltag/calibration/cam_translation_vector_all.xml",cam_translation_vector_all);
	
	cvReleaseMat(&intrinsic_matrix);
	cvReleaseMat(&distortion_coeffs);

	cvReleaseMat(&object_points2);
	cvReleaseMat(&image_points2);
	cvReleaseMat(&point_counts2);

	cvReleaseMat(&cam_rotation_all);
	cvReleaseMat(&cam_translation_vector_all);

	delete[] corners;



	//cvReleaseImage(&gray_image);

	cvDestroyAllWindows();
   while(!manifold_cam_exit())
	{
		sleep(1);
	}

	fclose(fp);
	sleep(1);
	return 0;
}
*/

/*int main()
{
	 int m=calibrate();
	 return 0;
}*/
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

	printf("gray_or_rgb==%d\n",gray_or_rgb);
	if(gray_or_rgb){
		pRawImg = cvCreateImage(cvSize(IMAGE_W, IMAGE_H),IPL_DEPTH_8U,3);
		pImg = cvCreateImage(cvSize(640, 480),IPL_DEPTH_8U,3);
		pData  = new unsigned char[1280 * 720 * 3];
	} else{
		pRawImg = cvCreateImage(cvSize(IMAGE_W, IMAGE_H),IPL_DEPTH_8U,1);
		pImg = cvCreateImage(cvSize(640, 480),IPL_DEPTH_8U,1);
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
	cam_info.K = {388.40923066779754, 0.0, 318.06257844065226, 0.0, 518.1538449374815, 241.17339016626644, 0.0, 0.0, 1.0};
	cam_info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
	cam_info.P = {373.5429992675781, 0.0, 317.51131336952494, 0.0, 0.0, 504.4360656738281, 240.6131009245937, 0.0, 0.0, 0.0, 1.0, 0.0};
	cam_info.binning_x = 0;
	cam_info.binning_x = 0;

	cam_info.roi.x_offset = 0;
	cam_info.roi.y_offset = 0;
	cam_info.roi.height = 0;
	cam_info.roi.width = 0;
	cam_info.roi.do_rectify = false;

	ret = manifold_cam_init(mode);
	if(ret == -1)
	{
		printf("manifold init error \n");
		return -1;
	}
   CvMat *pos_apriltag=cvCreateMat(3,1,CV_32FC1);
   CvMat *Rotation_camera2body=cvCreateMat(3,3,CV_32FC1);
	while(1)
	{
	   
	  
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

			time=ros::Time::now();
			cvi.header.stamp = time;
			cvi.header.frame_id = "image";

			if(gray_or_rgb){
				cvi.encoding = "bgr8";
			}else{
				cvi.encoding = "mono8";
			}
			cvi.image = pImg;
			cvi.toImageMsg(im);
			cam_info.header.seq = nCount;
			cam_info.header.stamp = time;
			caminfo_pub.publish(cam_info);
			image_pub.publish(im);
             
             cv::Mat img;
              img=cv::Mat(pImg); 
             
            demo.processImage(img);
          
			//camera frame to body fram     
		     	
		 Euler_To_Matrix(drone->gimbal.roll,drone->gimbal.pitch,drone->gimbal.yaw,Rotation_camera2body);
		    //Euler_To_Matrix(0,0,0,Rotation_camera2body);
		    if(a==-1&&b==-1)
		    {
		          //printf("not detect apriltags\n");
		    }
		    else
		    {
		      dianbianhuan(Rotation_camera2body,pos_apriltag,uav_sonor*100-5,a,b);
		      //dianbianhuan(Rotation_camera2body,pos_apriltag,76.5,a,b);
	        }
	          Ford_info.linear.x=cvmGet(pos_apriltag,0,0); //Ford 150 position in x frame(pingxing to world frame's x,cm)
		      Ford_info.linear.y=cvmGet(pos_apriltag,1,0);  //Ford 150 position in y frame(pingxing to world frame's y,cm)
		      Ford_info.angular.x=a; //apriltag that detected in u(pixel) 
		      Ford_info.angular.y=b;//apriltag that detected in v(pixel) 
	          Ford_pub.publish(Ford_info);
			nCount++;                 
   
            
         //  printf("speed yaw=%f, speed_pitch=%f\n",-(320-a),(240-b));
           cvWaitKey(10);
           ros::spinOnce();
		}
		else 
			break;
		usleep(1000);
	}
 // cvDestroyWindow("result");
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

