#include "Quatanion2Matrix.h"
/**
 * create rotation matrix for the quaternion
 * R: from guidance frame to inertial frame
 * q: from inertial frame to guidance frame
 */
void q_to_dcm(CvMat *q2, CvMat *R, int camera_ID)  {
	//float q[4] = { cvmGet(&q_init,0,0),  cvmGet(&q_init,1,0), cvmGet(&q_init,2,0), cvmGet(&q_init,3,0)};
       float q[4] = { cvmGet(q2,0,0),  cvmGet(q2,1,0), cvmGet(q2,2,0), cvmGet(q2,3,0)};
	float aSq = q[0] * q[0];
	float bSq = q[1] * q[1];
	float cSq = q[2] * q[2];
	float dSq = q[3] * q[3];

        CvMat* R_add=cvCreateMat(3,3,CV_32FC1);
        CvMat* R_guidance=cvCreateMat(3,3,CV_32FC1);
	cvmSet(R_guidance,0, 0,aSq + bSq - cSq - dSq);
	cvmSet(R_guidance,0, 1,2.0f * (q[1] * q[2] - q[0] * q[3]));
	cvmSet(R_guidance,0, 2,2.0f * (q[0] * q[2] + q[1] * q[3]));
	cvmSet(R_guidance,1, 0, 2.0f * (q[1] * q[2] + q[0] * q[3]));
	cvmSet(R_guidance,1, 1, aSq - bSq + cSq - dSq);
	cvmSet(R_guidance,1, 2, 2.0f * (q[2] * q[3] - q[0] * q[1]));
	cvmSet(R_guidance,2, 0, 2.0f * (q[1] * q[3] - q[0] * q[2]));
	cvmSet(R_guidance,2, 1,  2.0f * (q[0] * q[1] + q[2] * q[3]));
	cvmSet(R_guidance,2, 2,  aSq - bSq - cSq + dSq);
   //camera111
          
        if(camera_ID==1)
         {
             cvmSet(R_add,0,0,0);
	     cvmSet(R_add,0,1,0);
	    cvmSet(R_add,0,2,1);
	   cvmSet(R_add,1,0,1);
	   cvmSet(R_add,1,1,0);
	  cvmSet(R_add,1,2,0);
	  cvmSet(R_add,2,0,0);
	  cvmSet(R_add,2,1,1);
	  cvmSet(R_add,2,2,0);
           }
        //TODO camera_ID==2,3,4,5
       
         cvGEMM(R_guidance,R_add,1,NULL,0,R);
          // cvTranspose(R,R);
/*  
printf("Radd\n");
for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			     printf("%f\t",cvmGet(R_add,i,j));
		}
	printf("\n");
	}
		
     printf("Rguidance\n");
for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			     printf("%f\t",cvmGet(R_guidance,i,j));
		}
	printf("\n");
	}
		
   printf("R\n");
for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			     printf("%f\t",cvmGet(R,i,j));
		}
		printf("\n");
     
	}   
    
	 cvReleaseMat(&R_add);
         cvReleaseMat(&R_guidance);
*/
}

void Quaternion_To_Euler2(float q0,float q1,float q2,float q3,  float *yaw,float *pitch,float *roll)
{
		float r11,r12,r21,r31,r32,r1,r2,r3;
		
    	r11 = 2.0f *(q1 * q2 + q0 * q3);
    	r12 = q0 * q0 + q1 * q1 - q2 * q2  - q3 * q3 ;
    	r21 = -2.0f * (q1 * q3 - q0 * q2);
    	r31 = 2.0f *( q2 * q3 + q0  * q1);
    	r32 = q0 * q0 - q1 * q1  - q2 * q2 + q3 * q3 ;
    	/*float yaw= atan2( r11, r12 );
    	float pitch = asin( r21 );
    	float roll = atan2( r31, r32 );
           */    	
    	*yaw= atan2( r11, r12 );
        *pitch = asin( r21 );
        *roll = atan2( r31, r32 );
    	/*cvmSet(att, 0, 0, roll);
    	cvmSet(att, 1, 0, pitch);
    	cvmSet(att, 2, 0, yaw);*/

printf("%f\n",atan2(r11,r12));
printf("%f\n",asin(r21));
printf("%f\n",atan2(r31,r32));
printf("=====================");
//printf("%f  \n",asin(r21));
//printf("%f  \n",r21);
//printf("hhhhhhhhhhhhhhhhhhhhhhhh\n");
}
        