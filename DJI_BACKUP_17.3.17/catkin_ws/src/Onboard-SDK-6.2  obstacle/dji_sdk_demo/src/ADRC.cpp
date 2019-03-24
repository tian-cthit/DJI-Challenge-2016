#include"adrc.h"
#include<math.h>

#define sign(a) ((a)>0.0?1.0:(a)<0.0?-1.0:0.0)



CADRC::CADRC()
{
	TD_r=0.0; 
	TD_delta=0.0;

	NF_belta1=0.0; 
	NF_belta2=0.0;
	NF_alpha1=0.0;
	NF_alpha2=0.0;
	NF_b=0.0;
	NF_delta=0.0;
	NF_limited=0.0;
	NF_wc=0.0;	

	ESO_belta1=0.0;
	ESO_belta2=0.0;
	ESO_belta3=0.0;
	ESO_alpha1=0.0;
	ESO_alpha2=0.0;
	ESO_alpha3=0.0;
	ESO_b=0.0;
	ESO_delta=0.0;
	ESO_w0=0.0;
	
	Sys_a1=0.0;
	Sys_a2=0.0;
	Sys_b=0.0;
	Sys_T=0.0;
	
	TD_x1=0.0;
	TD_x2=0.0;
	ESO_z1=0.0;
	ESO_z2=0.0;
	ESO_z3=0.0;
	NF_cmd=0.0;
	fp=NULL;

//	printf("adrc build!\n");
}
CADRC::~CADRC()
{
	if(fp!=NULL)
	{
		fclose(fp);
		fp=NULL;
	}
	//printf("ADRC controller exit!\n");
}

int CADRC::Open( const char *filename)
{
	if(fp!=NULL)
	{
		fclose(fp);
		fp=NULL;
	}
	fp=fopen(filename,"w");
	if(fp==NULL)
	{
		printf("CADRC:  file open failed! %s\n",filename);
	}
	return 0;
}
#define LINUX_PLATFORM
#ifdef LINUX_PLATFORM
#include <sys/time.h>

long long CADRC::GetTime(void)
{
	  struct timeval t;
	  gettimeofday(&t, NULL);
	  return t.tv_sec*1000+t.tv_usec/1000;
//          return (int)(((double)t.tv_sec+(double)t.tv_usec/1000000.0)*1000.0);
}
#else

int CADRC::GetTime(void)
{
	return 0;
}

#endif

int CADRC::SetParamsAll(real_T p_TD_r,real_T p_TD_delta,real_T p_NF_belta1,real_T p_NF_belta2,real_T p_NF_alpha1, real_T p_NF_alpha2,real_T p_NF_b,real_T p_NF_delta,real_T p_NF_limited,real_T p_ESO_belta1,real_T p_ESO_belta2,real_T p_ESO_belta3,real_T p_ESO_alpha1,real_T p_ESO_alpha2,real_T p_ESO_alpha3,real_T p_ESO_b,real_T p_ESO_delta,real_T p_Sys_a1,real_T p_Sys_a2,real_T p_Sys_b,real_T p_Sys_T)
{
	this->TD_r=p_TD_r;//以后添加参数校验
	this->TD_delta=p_TD_delta;
	this->NF_belta1=p_NF_belta1;
	this->NF_belta2=p_NF_belta2;
	this->NF_alpha1=p_NF_alpha1;
	this->NF_alpha2=p_NF_alpha2;
	this->NF_b=p_NF_b;
	this->NF_delta=p_NF_delta;
	this->NF_limited=p_NF_limited;
	this->ESO_belta1=p_ESO_belta1;
	this->ESO_belta2=p_ESO_belta2;
	this->ESO_belta3=p_ESO_belta3;
	this->ESO_alpha1=p_ESO_alpha1;
	this->ESO_alpha2=p_ESO_alpha2;
	this->ESO_alpha3=p_ESO_alpha3;
	this->ESO_b=p_ESO_b;
	this->ESO_delta=p_ESO_delta;
	this->Sys_a1=p_Sys_a1;
	this->Sys_a2=p_Sys_a2;
	this->Sys_b=p_Sys_b;
	this->Sys_T=p_Sys_T;
	return 0;
}
int CADRC::SetParamsQuick(real_T p_TD_r,real_T p_TD_delta,real_T p_NF_wc, real_T p_NF_alpha1,real_T p_NF_alpha2,real_T p_NF_b,real_T p_NF_delta,real_T p_NF_limited,real_T p_ESO_w0,real_T p_ESO_alpha1,real_T p_ESO_alpha2,real_T p_ESO_alpha3,real_T p_ESO_b,real_T p_ESO_delta,real_T p_Sys_a1,real_T p_Sys_a2,real_T p_Sys_b,real_T p_Sys_T)
{
	this->TD_r=p_TD_r;//以后添加参数校验
	this->TD_delta=p_TD_delta;
	this->NF_wc=p_NF_wc;
	this->NF_belta1=this->NF_wc*this->NF_wc;
	this->NF_belta2=2.0*this->NF_wc;
	this->NF_alpha1=p_NF_alpha1;
	this->NF_alpha2=p_NF_alpha2;
	this->NF_b=p_NF_b;
	this->NF_delta=p_NF_delta;
	this->NF_limited=p_NF_limited;
	this->ESO_w0=p_ESO_w0;
	this->ESO_belta1=3*this->ESO_w0;
	this->ESO_belta2=3*this->ESO_w0*this->ESO_w0;
	this->ESO_belta3=3*this->ESO_w0*this->ESO_w0*this->ESO_w0;
	this->ESO_alpha1=p_ESO_alpha1;
	this->ESO_alpha2=p_ESO_alpha2;
	this->ESO_alpha3=p_ESO_alpha3;
	this->ESO_b=p_ESO_b;
	this->ESO_delta=p_ESO_delta;
	this->Sys_a1=p_Sys_a1;
	this->Sys_a2=p_Sys_a2;
	this->Sys_b=p_Sys_b;
	this->Sys_T=p_Sys_T;
	return 0;
}

int CADRC::ADRC_Diff(const real_T u[2],const real_T x[5], real_T dx[5])
{
	//u[0]:aim_pos, u[1]:feed_pos
	//x[0]:v1,x[1]:dv1, x[2]:y, x[3]:dy,x[4]:w
	real_T eso_err=x[2]-u[1];
	real_T y_last=0.0;

	ADRC_Output(x, &y_last);

	dx[0]=x[1];
	dx[1]=-TD_r*FST2(x[0],x[1],u[0],TD_r,TD_delta);
	dx[2]=x[3]-ESO_belta1*eso_err;
	dx[3]=x[4]-Sys_a2*x[3]-ESO_belta2*FAL(eso_err,ESO_alpha2,ESO_delta)+ESO_b*y_last;
	dx[4]=-ESO_belta3*FAL(eso_err,ESO_alpha3,ESO_delta);
	return 0;
}

int CADRC::ADRC_Diff2(const real_T u[3],const real_T x[5], real_T dx[5])
{
	//u[0]:aim_pos, u[1]:feed_pos
	//x[0]:v1,x[1]:dv1, x[2]:y, x[3]:dy,x[4]:w
	real_T eso_err=x[2]-u[1];
	real_T eso_err2=x[3]-u[2];

	real_T y_last=0.0;

	ADRC_Output2(x, &y_last);

	dx[0]=x[1];
	dx[1]=-TD_r*FST2(x[0],x[1],u[0],TD_r,TD_delta);
	dx[2]=x[3]-ESO_belta1*eso_err;
	dx[3]=x[4]-Sys_a2*x[3]-ESO_belta2*eso_err2+ESO_b*y_last;
	dx[4]=-ESO_belta3*FAL(eso_err2,ESO_alpha3,ESO_delta);

	return 0;
}

int CADRC::ADRC_Update(real_T x[5],const real_T dx[5])
{
	x[0]=x[0]+dx[0]*Sys_T;
	x[1]=x[1]+dx[1]*Sys_T;
	x[2]=x[2]+dx[2]*Sys_T;
	x[3]=x[3]+dx[3]*Sys_T;
	x[4]=x[4]+dx[4]*Sys_T;
	return 0;
}
int CADRC::ADRC_Output(const real_T x[5], real_T *y)
{
	real_T nf_e1=0.0;
	real_T nf_e2=0.0;
	real_T nf_cmd=0.0;

	nf_e1=x[0]-x[2];
	nf_e2=x[1]-x[3];
	nf_cmd=1/NF_b*(NF_belta1*nf_e1+NF_belta2*nf_e2-x[4]);

	//printf("%.4f %.4f [%.4f %.4f %.4f]\n",nf_e1,nf_e2,NF_b,NF_belta1,NF_belta2);
	if(nf_cmd>NF_limited)
	{
		nf_cmd=NF_limited;
	}
	else if(nf_cmd<-NF_limited)
	{
		nf_cmd=-NF_limited;
	}
	*y=nf_cmd;

	return 0;
}

int CADRC::ADRC_Output2(const real_T x[5], real_T *y)
{
	real_T nf_e1=0.0;
	real_T nf_e2=0.0;
	real_T nf_cmd=0.0;

	nf_e1=x[0]-x[2];
	nf_e2=x[1]-x[3];
	nf_cmd=1/NF_b*(NF_belta1*FAL(nf_e1,NF_alpha1,NF_delta)+NF_belta2*FAL(nf_e2,NF_alpha2,NF_delta)-x[4]);

	printf("%5.2f %5.2f [%5.2f %5.2f %5.2f][%5.2f %5.2f]\n",nf_e1,nf_e2,NF_b,NF_belta1,NF_belta2,FAL(nf_e1,NF_alpha1,NF_delta),FAL(nf_e2,NF_alpha2,NF_delta));
	if(nf_cmd>NF_limited)
	{
		nf_cmd=NF_limited;
	}
	else if(nf_cmd<-NF_limited)
	{
		nf_cmd=-NF_limited;
	}
	*y=nf_cmd;

	return 0;
}

int CADRC::ADRC_Once(real_T pos_aim,real_T pos_feedback,real_T *vel_cmd)
{
	real_T u[2],y;
	u[0]=pos_aim;
	u[1]=pos_feedback;
	
	real_T y1[5];
	real_T dy[5];
	real_T h=Sys_T;

	y1[0]=TD_x1;
	y1[1]=TD_x2;
	y1[2]=ESO_z1;
	y1[3]=ESO_z2;
	y1[4]=ESO_z3;
		
	this->ADRC_Diff(u,y1,dy);
	this->ADRC_Update(y1,dy);
	this->ADRC_Output(y1,&y);
	
	TD_x1=y1[0];
	TD_x2=y1[1];
	ESO_z1=y1[2];
	ESO_z2=y1[3];
	ESO_z3=y1[4];

	this->ADRC_Output(y1,&y);

	NF_cmd=y;
	*vel_cmd=NF_cmd;
	return 0;
}
int CADRC::ADRC_RK42(real_T pos_aim,real_T pos_feedback,real_T vel_feedback,real_T *vel_cmd)
{
	real_T u[3],y;
	u[0]=pos_aim;
	u[1]=pos_feedback;
	u[2]=vel_feedback;

	real_T k1[5],k2[5],k3[5],k4[5];
	real_T y0[5],y1[5],yt[5];
	real_T h=Sys_T;

	k1[0]=k1[1]=k1[2]=k1[3]=k1[4]=0.0;
	k2[0]=k2[1]=k2[2]=k2[3]=k2[4]=0.0;
	k3[0]=k3[1]=k3[2]=k3[3]=k3[4]=0.0;
	k4[0]=k4[1]=k4[2]=k4[3]=k4[4]=0.0;
	y1[0]=y1[1]=y1[2]=y1[3]=y1[4]=0.0;
	yt[0]=yt[1]=yt[2]=yt[3]=yt[4]=0.0;

	y0[0]=TD_x1;
	y0[1]=TD_x2;
	y0[2]=ESO_z1;
	y0[3]=ESO_z2;
	y0[4]=ESO_z3;
	
	//y0[0]=u[0];
	//y0[1]=0;
	//yt[0]=u[0];
	//yt[1]=0;

	this->ADRC_Diff2(u,y0,k1);

	for(int i=0; i<5; i++)
	{
		yt[i]=y0[i]+h/2*k1[i];
	}	
	this->ADRC_Diff2(u,yt,k2);
	
	for(int i=0; i<5; i++)
	{
		yt[i]=y0[i]+h/2*k2[i];
	}
	this->ADRC_Diff2(u,yt,k3);

	for(int i=0; i<5; i++)
	{
		yt[i]=y0[i]+h*k3[i];
	}
	this->ADRC_Diff2(u,yt,k4);

	for(int i=0; i<5; i++)
	{
		y1[i]=y0[i]+h/6*(k1[i]+2*k2[i]+2*k3[i]+k4[i]);
	}


	//this->ADRC_Diff(u,x,dx);	
	//this->ADRC_Update(x,dx);
	//this->ADRC_Output(x,&y);

	//y1[0]=u[0];
	//y1[1]=0;

	if(fabs(y1[4])>NF_limited*NF_b)
	{
		y1[4]=sign(y1[4])*NF_limited*NF_b;
	}

	TD_x1=y1[0];
	TD_x2=y1[1];
	ESO_z1=y1[2];
	ESO_z2=y1[3];
	ESO_z3=y1[4];

	this->ADRC_Output2(y1,&y);

	NF_cmd=y;
	*vel_cmd=NF_cmd;

	if(fp!=NULL)
	{
		long long timenow=0;
		timenow=GetTime();
		fprintf(fp,"%lld %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n",timenow,y1[0],y1[1],y1[2],y1[3],y1[4],u[0],u[1],y,u[2]);		
	}
	return 0;
}
int CADRC::ADRC_RK4(real_T pos_aim,real_T pos_feedback,real_T *vel_cmd)
{
	real_T u[2],y;
	u[0]=pos_aim;
	u[1]=pos_feedback;
	
	real_T k1[5],k2[5],k3[5],k4[5];
	real_T y0[5],y1[5],yt[5];
	real_T h=Sys_T;

	k1[0]=k1[1]=k1[2]=k1[3]=k1[4]=0.0;
	k2[0]=k2[1]=k2[2]=k2[3]=k2[4]=0.0;
	k3[0]=k3[1]=k3[2]=k3[3]=k3[4]=0.0;
	k4[0]=k4[1]=k4[2]=k4[3]=k4[4]=0.0;
	y1[0]=y1[1]=y1[2]=y1[3]=y1[4]=0.0;
	yt[0]=yt[1]=yt[2]=yt[3]=yt[4]=0.0;

	y0[0]=TD_x1;
	y0[1]=TD_x2;
	y0[2]=ESO_z1;
	y0[3]=ESO_z2;
	y0[4]=ESO_z3;
	
	y0[0]=u[0];
	y0[1]=0;
	yt[0]=u[0];
	yt[1]=0;

	this->ADRC_Diff(u,y0,k1);

	for(int i=2; i<5; i++)
	{
		yt[i]=y0[i]+h/2*k1[i];
	}	
	this->ADRC_Diff(u,yt,k2);
	
	for(int i=2; i<5; i++)
	{
		yt[i]=y0[i]+h/2*k2[i];
	}
	this->ADRC_Diff(u,yt,k3);

	for(int i=2; i<5; i++)
	{
		yt[i]=y0[i]+h*k3[i];
	}
	this->ADRC_Diff(u,yt,k4);

	for(int i=2; i<5; i++)
	{
		y1[i]=y0[i]+h/6*(k1[i]+2*k2[i]+2*k3[i]+k4[i]);
	}


	//this->ADRC_Diff(u,x,dx);	
	//this->ADRC_Update(x,dx);
	//this->ADRC_Output(x,&y);

	y1[0]=u[0];
	y1[1]=0;

	TD_x1=y1[0];
	TD_x2=y1[1];
	ESO_z1=y1[2];
	ESO_z2=y1[3];
	ESO_z3=y1[4];

	this->ADRC_Output(y1,&y);

	NF_cmd=y;
	*vel_cmd=NF_cmd;

	if(fp!=NULL)
	{
		long long timenow=0;
		timenow=GetTime();
		fprintf(fp,"%lld %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n",timenow,y1[0],y1[1],y1[2],y1[3],y1[4],u[0],u[1],y);		
	}
	return 0;
}
real_T CADRC::FAL(real_T e,real_T alpha,real_T delta)
{
	real_T fans=0.0;
	if(fabs(e)<delta)
	{
		fans=e*pow(delta,alpha-1);
	}
	else
	{
		fans=pow(fabs(e),alpha)*sign(e);
	}
	return fans;	
	return 0.0;
}
real_T CADRC::FST2(real_T x1,real_T x2,real_T v,real_T r,real_T d)
{
	real_T temp=0.0;
	real_T fans=0.0;
	temp=x1-v+fabs(x2)*x2/(2*r);
	if(fabs(temp)>=d)
	{
		fans=sign(temp);
	}
	else
	{
		fans=temp/d;
	}
	return fans;
	return 0.0;
}

int CADRC::Setw0wc(real_T p_ESO_w0,real_T p_NF_wc)
{
	this->ESO_w0=p_ESO_w0;
	this->ESO_belta1=3*this->ESO_w0;
	this->ESO_belta2=3*this->ESO_w0*this->ESO_w0;
	this->ESO_belta3=3*this->ESO_w0*this->ESO_w0*this->ESO_w0;
	this->NF_wc=p_NF_wc;
	this->NF_belta1=this->NF_wc*this->NF_wc;
	this->NF_belta2=2.0*this->NF_wc;
	return 0;
}
int CADRC::Setb(real_T b)
{
	this->ESO_b=b;
	this->NF_b=b;
	this->Sys_b=b;
	return 0;
}
