/*
 * =====================================================================================
 *
 *       Filename:  turtlesim_P_controller.cpp
 *
 *    Description:  Proportional Controller for Turtlesim
 *
 *        Version:  1.0
 *        Created:  Thursday 10 May 2018 08:49:41  IST
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Shailesh Chandra Prusty, shailesh.prusty@gmail.com
 *   Organization:  
 *
 * =====================================================================================
 */

#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<iostream>
#include<turtlesim/Pose.h>
#include<cmath>

using namespace std;
geometry_msgs::Twist msg;

double x;
double k = 1.5 , kv = 0.5;
double y, d;
double theta, phi, beta;
double X=1, Y=1;

void control(turtlesim::Pose data)
{
	x = data.x;
	y = data.y;
	theta = data.theta;
    	d = sqrt((X-x)*(X-x)+(Y-y)*(Y-y));
	phi = atan2((Y-y),(X-x));
        beta = phi - theta;
	if (beta >= 3.14)
		beta = beta - 2*3.14;
	if (beta <= -3.14)
		beta = 2*3.14 + beta;
	cout<<"beta=  "<<beta<<"phi=  "<<phi<<"theta=  "<<theta<<endl;
	
	//proportional navigation
	
	if(d <= 0.2)
	{
		msg.linear.x=0;
		msg.angular.z=0;
	}

	else
    	{
		msg.linear.x=1;
		msg.angular.z=k*beta;
    	}
}	

int main(int argv, char **argc)
{

	ros::init(argv,argc,"turtlesimNode");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/turtle1/pose",1000,control);

	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1000);

	ros::Rate loop_rate(1);

	while(ros::ok())
	{
		ros::spinOnce();
		pub.publish(msg);
		loop_rate.sleep();
	}

	return 0;
}
