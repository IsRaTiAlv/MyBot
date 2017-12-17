#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

class TwistToMotors
{
public:
	TwistToMotors();
	void spin();
private:
	ros::NodeHandle n;	
	ros::Publisher pub_lmotor;
	ros::Publisher pub_rmotor;
	ros::Subscriber cmd_vel_sub;
	float left;
	float right;
	float ticks_since_target;
	double timeout_ticks;
	double w;
	double rate;
	float dx,dy,dr;
	void init_variables();
	void get_parameters();
	void spinOnce();
	void twistCallback(const geometry_msgs::Twist &twist_aux);
};
TwistToMotors::TwistToMotors()
{
	init_variables();
	get_parameters();
	ROS_INFO("Started Twist to Motor node");	
	cmd_vel_sub = n.subscribe("cmd_vel",10, &TwistToMotors::twistCallback, this);	
	pub_lmotor = n.advertise<std_msgs::Int16>("motor1", 50);
	pub_rmotor = n.advertise<std_msgs::Int16>("motor2", 50);	
}

void TwistToMotors::init_variables()
{
	left = 0;
	right = 0;
	dx = dy = dr =0;
	w = 0.2 ;
	rate = 20;
	timeout_ticks = 1;
}


void TwistToMotors::get_parameters()
{	
        if(n.getParam("rate", rate)){	 
		ROS_INFO_STREAM("Rate from param" << rate);	       
	}
        if(n.getParam("timeout_ticks", timeout_ticks)){	 
		ROS_INFO_STREAM("timeout_ticks from param" << timeout_ticks);	       
	}	
        if(n.getParam("base_width", w)){	 
		ROS_INFO_STREAM("Base_width from param" << w);	       
	}
}
void TwistToMotors::spin()
{
	ros::Rate r(rate);
	ros::Rate idle(10);
	ros::Time then = ros::Time::now();	
	ticks_since_target = timeout_ticks;
	while (ros::ok())
	{
	while (ros::ok() && (ticks_since_target <= timeout_ticks))	
		{		
		spinOnce();
		r.sleep();
		}
	ros::spinOnce();
        idle.sleep();	
	}
}

void TwistToMotors::spinOnce()
{
	right = ( 1.0 * dx )*5 + (dr * w /2)*4;
	left = ( 1.0 * dx )*5 - (dr * w /2)*4;
	//right=left*2;
	//left=right*2;
	int maxPWM=255;
	std_msgs::Int16 left_;
	std_msgs::Int16 right_;
	if(right>=0){
		if(right>1){right=1;}
		right_.data = right*maxPWM;
	}
	else{
		if(right<-1){right=-1;}
		right_.data = right*-maxPWM+maxPWM;

	}
	if(left>=0){
		if(left>1){left=1;}
		left_.data = left*maxPWM;
	}
	else{
		if(left<-1){left=-1;}
		left_.data = left*-maxPWM+maxPWM;
	}

	pub_lmotor.publish(left_);
	pub_rmotor.publish(right_);
	ticks_since_target += 1;
	ros::spinOnce();
}

void TwistToMotors::twistCallback(const geometry_msgs::Twist &msg)
{
	ticks_since_target = 0;	
	dx = msg.linear.x;
	dy = msg.linear.y;
	dr = msg.angular.z;
}
int main(int argc, char **argv)
{
	ros::init(argc, argv,"twist_to_motor");
	TwistToMotors obj;
	obj.spin();
}
