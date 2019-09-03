/** @file pose_node.h
 *  @version 1.0
 *  @date Sep, 2019
 *
 *  @brief
 *  
 *
 *  @copyright 2019 RC. All rights reserved.
 *
 */

#ifndef _POSE_NODE_H
#define _POSE_NODE_H
//ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h> //for px4's external pose estimate
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
//EIGEN
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
//MAVROS
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/StreamRate.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/AttitudeTarget.h>

// PID
#include <pid/pid.h>
#include "pose/PIDConfig.h"


/*!
 * @brief 
 */
//ZED
geometry_msgs::PoseStamped  zed_pose;
Eigen::Quaterniond zed_q;
double zed_yawInRad=0;
double zedX=0;
double zedY=0;
double zedZ=0;
double  zedX_V=0, zedY_V=0;


//PID
PID Pitch_PID_P(0,0,0,0.5);
PID Roll_PID_P(0,0,0,0.5);
PID Pitch_PID_V(0,0,0,0.4);
PID Roll_PID_V(0,0,0,0.4);
PID Vertial_PID_P(0.2,0,0,0.35);

//PX4
ros::Publisher px4_external_pose_estimate;
ros::Publisher fakegps_pose_estimate;
ros::Publisher set_att;

//RC
double gun_data=0;
double yaw_data=0;
double pitch_data=0;
double roll_data=0;

//desire
double desire_PosX = 0;//Opposite X postion
double desire_PosY = 0;//Opposite Y postion
double desire_PosH = 0.0;//high
double desire_AngW = 0;//Yaw

//OUTPUT
double Roll_OUTPUT = 0;
double Pitch_OUTPUT = 0;
double Vertial_OUTPUT = 0;
double Yaw_OUTPUT = 0;

//service
ros::ServiceClient set_rate_client;
ros::ServiceClient set_mode_client;
ros::ServiceClient arming_client;

//other
int arm_enable = 0;

/**
 **********************************************
 *              |X+
 *              |
 *         -----|-----Y-
 *              |
 *              |
 *              |
 *         UP:Z+   
 *****/





#endif // 
