#include<iostream>
#include<stdio.h>
#include<stdlib.h>

#include"adrc.h"

int main(void)
{
	freopen("ans.txt","w",stdout);
	real_T u=0.0;
	real_T T;
	T=0.01;

	//CADRC<real_T> theADRC;
	CADRC theADRC;	
	theADRC.SetParamsAll(10,0.01,625,50,1.75,0.55,2.6,0.1,10.0,60,1200,24000,0.0,1.0,1.25,2.6,0.01,0.0,2.6,2.6,0.05);
	theADRC.Setw0wc(20.0,25.0);
	theADRC.Setb(2.6);
	theADRC.TD_r=10.0;
	theADRC.TD_delta=0.01;		
	theADRC.Sys_T=T;

	real_T x1,x2;
	x1=0.0;
	x2=0.0;
	
	for(int k=0; k<5/T; k++)
	{
		theADRC.ADRC_Once(1.0,x1,&u);				
		//printf("TD:%.4f %.4f\n",theADRC.TD_x1,theADRC.TD_x2);
		printf("%.4f,%.4f,%.4f\n",theADRC.ESO_z1,theADRC.ESO_z2,theADRC.ESO_z3);
		//printf("%.4f\n",u);

		real_T dx1,dx2;
		dx1=x2;
		dx2=-2.6*x2+2.6*u;
		x1=x1+dx1*T;
		x2=x2+dx2*T;
		//printf("%.4f %.4f\n",x1,x2);		
	}


	return 0;
}