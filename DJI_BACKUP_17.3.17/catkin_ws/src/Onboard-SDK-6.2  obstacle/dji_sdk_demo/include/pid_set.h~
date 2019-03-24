#include <stdio.h>
#include <stdlib.h>

struct pid_x				//define x_position parameters
{
	float kp,ki,kd;			
	float error;			
	float error_last;		
	float integral;			
	float v;
};
struct pid_y				//define y_position parameters
{
	float kp,ki,kd;			
	float error;			
	float error_last;		
	float integral;			
	float v;
};
struct pid_yaw				//define yaw_panament
{
	float kp,ki,kd;			
	float error;			
	float error_last;		
	float integral;			
	float v;
};
struct pid_u				//define u parameters
{
	float kp,ki,kd;			
	float error;			
	float error_last;		
	float integral;			
	float v;
};
struct pid_v				//define v parameters
{
	float kp,ki,kd;			
	float error;			
	float error_last;		
	float integral;			
	float v;
};
struct pid_h				//define h parameters
{
	float kp,ki,kd;			
	float error;			
	float error_last;		
	float integral;			
	float v;
};
struct pid_vx				//define x_velocity parameters
{
	float kp,ki,kd;
	float error;
	float error_last;
	float integral;
	float a;
};
struct pid_vy				//define y_velocity parameters
{
	float kp,ki,kd;
	float error;
	float error_last;
	float integral;
	float a;
};
struct att				//define attitude parameters
{
	float pitch;
	float roll;
};

void PID_set(struct pid_x* p_x,struct pid_y* p_y,struct pid_yaw* p_yaw,struct pid_u* p_u,struct pid_v* p_v,struct pid_h* p_h,struct pid_vx* p_vx,struct pid_vy* p_vy,struct att* att);
