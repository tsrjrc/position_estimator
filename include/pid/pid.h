/** @pid.h
 *  @version 1.0
 *  @date May, 2019
 *
 *  @brief
 *  pid class
 *
 *  @copyright 2019 RC. All rights reserved.
 *
 */

#ifndef _PID_H
#define _PID_H

// ROS includes
#include <ros/ros.h>

class PID
{
	private:
		float P;
		float I;
		float D;
		float E;
		float LE;
		float SE;
		float MAX;
	public:
		PID(float p,float i,float d,float max);
		PID();
		~PID();
		void Set_PID(float p,float i,float d);
		void PID_Empty();
		void PID_Cal(float mydesire,float curren_data);
		void PID_SetMax(float a);
		void Print();
		double OUTPUT;

};



#endif 
