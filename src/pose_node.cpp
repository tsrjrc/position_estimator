/** @file pose_node.cpp
 *  @version 1.0
 *  @date May, 2019
 *
 *  @brief
 *  
 *
 *  @copyright 2019 RC. All rights reserved.
 *
 */
#include "pose/pose_node.h"

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
	geometry_msgs::Vector3 ans;

	tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
	R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
	return ans;
}

void zedposeHandler(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
	static double zedX_last=zedX;
	static double zedY_last=zedY;
	static double zedZ_last=zedZ;
	
	Eigen::Vector3d zed;
	zed << pose->pose.position.x,pose->pose.position.y,pose->pose.position.z;
	zed_q.x() = pose->pose.orientation.x;
	zed_q.y() = pose->pose.orientation.y;
	zed_q.z() = pose->pose.orientation.z;
	zed_q.w() = pose->pose.orientation.w;
	zed_yawInRad = toEulerAngle(pose->pose.orientation).z;

	Eigen::Vector3d zed_temp(0,0,0);
	// 	zed_temp = (zed_q.inverse())*zed;
	Eigen::AngleAxisd Axis2 = Eigen::AngleAxisd(toEulerAngle(pose->pose.orientation).z,Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd Axis1 = Eigen::AngleAxisd(toEulerAngle(pose->pose.orientation).y,Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd Axis0 = Eigen::AngleAxisd(toEulerAngle(pose->pose.orientation).x,Eigen::Vector3d::UnitX());
	zed_temp=Axis2.inverse()*zed;
	zedX=zed_temp(0);
	zedY=zed_temp(1);
	zedZ=zed_temp(2);

	geometry_msgs::TransformStamped fakegps_pose;
	fakegps_pose.header.frame_id = pose->header.frame_id;
	fakegps_pose.header.stamp = ros::Time::now();
	fakegps_pose.transform.rotation.x = pose->pose.orientation.x;
	fakegps_pose.transform.rotation.y = pose->pose.orientation.y;
	fakegps_pose.transform.rotation.z = pose->pose.orientation.z;
	fakegps_pose.transform.rotation.w = pose->pose.orientation.w;
	fakegps_pose.transform.translation.x = pose->pose.position.x;
	fakegps_pose.transform.translation.y = pose->pose.position.y;
	fakegps_pose.transform.translation.z = pose->pose.position.z;
	fakegps_pose_estimate.publish(fakegps_pose);
	
	//
	zedX_V = (zedX - zedX_last)*30.0;
	zedY_V = (zedY - zedY_last)*30.0;
	
	//
	zedX_last = zedX;
	zedY_last = zedY;
	zedZ_last = zedZ;
// 	ROS_INFO("zedX:%f;zedY:%f;zedZ:%f",zedX,zedY,zedZ);
}

void imuDataHandler(const sensor_msgs::Imu::ConstPtr& imuData)
{

  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imuData->orientation, orientation);
//   tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

Eigen::Quaterniond to_quaternion(double roll = 0.0, double pitch = 0.0, double yaw = 0.0)
{
	Eigen::Quaterniond q_att(1,0,0,0);
	double t0 = cos(yaw * 0.5);
	double t1 = sin(yaw * 0.5);
	double t2 = cos(roll * 0.5);
	double t3 = sin(roll * 0.5);
	double t4 = cos(pitch * 0.5);
	double t5 = sin(pitch * 0.5);

	q_att.w() = t0 * t2 * t4 + t1 * t3 * t5;
	q_att.x() = t0 * t3 * t4 - t1 * t2 * t5;
	q_att.y() = t0 * t2 * t5 + t1 * t3 * t4;
	q_att.z() = t1 * t2 * t4 - t0 * t3 * t5;
	return q_att;
}

void rc_cb(const mavros_msgs::RCIn::ConstPtr& msg)
{
	if(msg->channels.size() == 0)
	{
		return;
	}
	else
	{
		if(msg->channels[2] == 1062)
		{
			return;
		}
	}
	
	int thr_raw_data = msg->channels[2];
	int yaw_raw_data = msg->channels[3];
	int pitch_raw_data = msg->channels[1];
	int roll_raw_data = msg->channels[0];
	int function_select = msg->channels[7];
	int is_shutdown = msg->channels[5];
	//Throttle
	if(thr_raw_data>=1450 && thr_raw_data<=1550)
	{
		gun_data = 0.5;
	}
	else if(thr_raw_data < 1450)
	{
		gun_data = 0.5 - (1450-thr_raw_data)*0.001;
	}
	else if(thr_raw_data > 1550)
	{
		gun_data = 0.5 + (thr_raw_data-1550)*0.001;
	}
	//Pitch
	if(pitch_raw_data>=1450 && pitch_raw_data<=1550)
	{
		pitch_data = 0.0;
	}
	else if(pitch_raw_data < 1450)
	{
		pitch_data = 0.0 + (1450-pitch_raw_data)*0.001;
	}
	else if(pitch_raw_data > 1550)
	{
		pitch_data = 0.0 - (pitch_raw_data-1550)*0.001;
	}
	//Roll
	if(roll_raw_data>=1450 && roll_raw_data<=1550)
	{
		roll_data = 0.0;
	}
	else if(roll_raw_data < 1450)
	{
		roll_data = 0.0 - (1450-roll_raw_data)*0.001;
	}
	else if(roll_raw_data > 1550)
	{
		roll_data = 0.0 + (roll_raw_data-1550)*0.001;
	}
	//Yaw
	if(yaw_raw_data>=1450 && yaw_raw_data<=1550)
	{
		yaw_data = 0.0;
	}
	else if(yaw_raw_data < 1450)
	{
		yaw_data = 0.0 + (1450-yaw_raw_data)*0.001;
	}
	else if(yaw_raw_data > 1550)
	{
		yaw_data = 0.0 - (yaw_raw_data-1550)*0.001;
	}
	//function
	if(function_select>1500)
	{
		arm_enable = 1;
	}
	else
	{
		arm_enable = 0;
	}

	if(!current_state.armed)
	{
		if(is_shutdown > 1500)
		{
			exit(0);
		}
	}
	

}

bool goto_guidedmode(bool curr_state)
{
	if(!curr_state)
	{
		return 0;
	}
	if( current_state.mode == "GUIDED_NOGPS")
	{
		return 1;
	}	
	
	static ros::Time last_request = ros::Time::now();
	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "GUIDED_NOGPS";

	if(ros::Time::now() - last_request > ros::Duration(5.0))
	{
		last_request = ros::Time::now();
		if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
		{
			ROS_INFO("Guided_NoGPS enabled");
			return 1;
		}
	}
	return 0;
}

bool goto_arm(bool curr_state)
{
	if(!curr_state)
	{
		return 0;
	}
	if(current_state.armed)
	{
		return 1;
	}
	static ros::Time last_request = ros::Time::now();
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;
	if(ros::Time::now() - last_request > ros::Duration(5.0))
	{
		if( arming_client.call(arm_cmd) && arm_cmd.response.success)
		{
			last_request = ros::Time::now();
			ROS_INFO("Vehicle armed");
			return 1;
		}
	}
	return 0;
	
}

bool gotoAltHold(bool curr_state)
{
	if(!curr_state)
	{
		return 0;
	}
	if( current_state.mode == "ALT_HOLD")
	{
		return 1;
	}
	static ros::Time last_request = ros::Time::now();
	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "ALT_HOLD";

	if(ros::Time::now() - last_request > ros::Duration(5.0))
	{
		last_request = ros::Time::now();
		if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
		{
			ROS_INFO("ALT_HOLD enabled");
			return 1;
		}
	}
	return 0;
}

bool set_datastream()
{
    mavros_msgs::StreamRate stream_rate;
    stream_rate.request.on_off = true;
    stream_rate.request.message_rate = 25;
    if(set_rate_client.call(stream_rate))
    {
      return 1;
    }
    return 0;
}

void pid_callback(dynamic_pid::PIDConfig &config, uint32_t level)
{

	Pitch_PID_P.Set_PID(config.Position_Param_P,config.Position_Param_I,config.Position_Param_D);
	Roll_PID_P.Set_PID(config.Position_Param_P,config.Position_Param_I,config.Position_Param_D);
	Pitch_PID_V.Set_PID(config.Velocity_Param_P,config.Velocity_Param_I,config.Velocity_Param_D);
	Roll_PID_V.Set_PID(config.Velocity_Param_P,config.Velocity_Param_I,config.Velocity_Param_D);

	Vertial_PID_P.Set_PID(config.Vertial_Param_P,config.Vertial_Param_I,config.Vertial_Param_D);

	Pitch_PID_P.PID_SetMax(config.Position_OUT_Max);
	Roll_PID_P.PID_SetMax(config.Position_OUT_Max);
	
	Pitch_PID_V.PID_SetMax(config.Velocity_OUT_Max);
	Roll_PID_V.PID_SetMax(config.Velocity_OUT_Max);

	Vertial_PID_P.PID_SetMax(config.Vertial_OUT_Max);
	
	ROS_INFO("*****************************************************");
	ROS_INFO("***********Pitch Position***********");
	Pitch_PID_P.Print();
	ROS_INFO("***********Roll Position***********");
	Roll_PID_P.Print();
	
	ROS_INFO("***********Pitch Velocity***********");
	Pitch_PID_V.Print();
	ROS_INFO("***********Roll Velocity***********");
	Roll_PID_V.Print();
	
	ROS_INFO("***********Vertial***********");
	Vertial_PID_P.Print();

}


void send_attdata(double th,Eigen::Quaterniond q,double y)
{
	mavros_msgs::AttitudeTarget setatt;
	setatt.header.stamp = ros::Time::now();
	setatt.type_mask = 0b00000000;
	setatt.thrust = th;
	setatt.orientation.x = q.x();
	setatt.orientation.y = q.y();
	setatt.orientation.z = q.z();
	setatt.orientation.w = q.w();
	setatt.body_rate.z = y;
	set_att.publish(setatt);
}

int main(int argc, char **argv) {


    ros::init(argc, argv, "indoor_control_node");
    ros::NodeHandle nh;

    //NOTE For px4's external pose estimate
    px4_external_pose_estimate = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",100);
    fakegps_pose_estimate = nh.advertise<geometry_msgs::TransformStamped>("/mavros/fake_gps/mocap/tf",100);
    ros::Subscriber zed_pose_sub = nh.subscribe<geometry_msgs::PoseStamped> ("/zed/zed_node/pose", 1000, zedposeHandler);
    ros::Subscriber imuDataSub = nh.subscribe<sensor_msgs::Imu> ("/mavros/imu/data", 50, imuDataHandler);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 50, rc_cb);
    set_att = nh.advertise<mavros_msgs::AttitudeTarget> ("/mavros/setpoint_raw/attitude",50);

    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    set_rate_client = nh.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");

	
	//dynamic_reconfigure
	dynamic_reconfigure::Server<dynamic_pid::PIDConfig> server;
	dynamic_reconfigure::Server<dynamic_pid::PIDConfig>::CallbackType f;
	f = boost::bind(&pid_callback, _1, _2);
	server.setCallback(f);

	ros::AsyncSpinner spinner(3); // Use 4 threads
	spinner.start();

	
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(1000);
    // wait for FCU connection
    while(ros::ok() && !current_state.connected)
    {
       ros::spinOnce();
       rate.sleep();
    }

    set_datastream();
	
	int task_ms=0;
    while(ros::ok())
    {

		if(task_ms%500 == 0)
		{
			goto_arm(goto_guidedmode(arm_enable==1));
			gotoAltHold(arm_enable==0);
		}
		if(task_ms%100 == 0)
		{
			Vertial_PID_P.PID_Cal(desire_PosH, zedZ);
			Roll_PID_P.PID_Cal(-(desire_PosY), -zedY);//Position PID
			Pitch_PID_P.PID_Cal(desire_PosX, zedX);//Position PID
		}
		if(task_ms%50 == 0)
		{
			if(gun_data!=0.5)
			{
				desire_PosH = zedZ;
				Vertial_PID_P.PID_Empty();
			}
			if((fabs(pitch_data)>=0.00001) || (fabs(roll_data)>=0.00001))
			{
				
				desire_PosY = zedY;
				desire_PosX = zedX;
				Roll_PID_P.PID_Empty();
				Roll_PID_V.PID_Empty();
				Pitch_PID_P.PID_Empty();
				Pitch_PID_V.PID_Empty();
			}
		}
		if(task_ms%20  == 0)
		{
			Roll_PID_V.PID_Cal(Roll_PID_P.OUTPUT, -zedY_V);//Velocity PID dt *100 & /0.01 former run speed faster unit:m/s
			Pitch_PID_V.PID_Cal(Pitch_PID_P.OUTPUT, zedX_V);//Velocity PID dt *100 & /0.01 former run speed faster unit:m/s
			
			Vertial_OUTPUT = gun_data + Vertial_PID_P.OUTPUT;
			Roll_OUTPUT = roll_data + Roll_PID_V.OUTPUT;
			Pitch_OUTPUT = pitch_data + Pitch_PID_V.OUTPUT;
			Yaw_OUTPUT = yaw_data;
// 			ROS_INFO("Roll:%f;Pitch:%f;Yaw:%f;Ver:%f",Roll_PID_V.OUTPUT,Pitch_PID_V.OUTPUT,Yaw_OUTPUT,Vertial_OUTPUT);
			send_attdata(Vertial_OUTPUT,to_quaternion(Roll_OUTPUT,Pitch_OUTPUT,Yaw_OUTPUT),Yaw_OUTPUT);
		}

		if(task_ms%10 == 0)
		{

			ros::spinOnce();
		}
		rate.sleep();
		task_ms++;
		if(task_ms%1000 == 0) task_ms =0;

    }

    return 0;
}
