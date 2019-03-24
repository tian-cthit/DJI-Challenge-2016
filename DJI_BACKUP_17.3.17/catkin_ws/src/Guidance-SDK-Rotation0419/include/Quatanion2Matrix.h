#include "highgui.h"
#include "cv.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

void Quaternion_To_Euler2(float q0,float q1,float q2,float q3,  float *yaw,float *pitch,float *roll);
void q_to_dcm(CvMat *q2, CvMat *R, int camera_ID) ;
