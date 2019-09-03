/** @file pid.cpp
 *  @version 1.0
 *  @date May, 2019
 *
 *  @brief
 *  pid class
 *
 *  @copyright 2019 RC. All rights reserved.
 *
 */

#include "pid/pid.h"

PID::PID(float p, float i, float d,float max)
{
	P=p;
	I=i;
	D=d;
	E=0;
	LE=0;
	SE=0;
	OUTPUT=0;
	MAX=max;
}
void PID::PID_SetMax(float a)
{
	MAX = a;
}
void PID::Set_PID(float p,float i,float d)
{
	P=p;
	I=i;
	D=d;
}

void PID::PID_Empty()
{
	E=0;
	LE=0;
	SE=0;
	OUTPUT=0;
}
void PID::PID_Cal(float mydesire, float curren_data)
{
	E = mydesire - curren_data;
	
	(fabs(OUTPUT)<=MAX)||(OUTPUT>MAX&&E<0)||(OUTPUT<-MAX&&E>0)?SE+=E:SE+=0;
	
	OUTPUT=P*E+I*SE+D*(E-LE);
	
	LE = E;

}
PID::~PID()
{
	P=0;
	I=0;
	D=0;
	E=0;
	LE=0;
	SE=0;
	OUTPUT=0;
	MAX=0;
}
PID::PID()
{
	P=0;
	I=0;
	D=0;
	E=0;
	LE=0;
	SE=0;
	OUTPUT=0;
	MAX=0;
}

void PID::Print()
{
      ROS_INFO("PID:%f,%f,%f,  MAX:%f", P, I, D,MAX);
}
