/*
 * =====================================================================================
 *
 *       Filename:  copter_P_control.cpp
 *
 *    Description:  Proportional Controller for ArduCopter
 *
 *        Version:  1.0
 *        Created:  Thursday 10 May 2018 09:55:12  IST
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Shailesh Chandra Prusty, shailesh.prusty@gmail.com
 *   Organization:  
 *
 * =====================================================================================
 */

#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Twist.h"
#include "math.h"

using namespace std;

#define Vk 500

#define kp 0.8

std_msgs::Float64 target_LO;
std_msgs::Float64 target_LA;
std_msgs::Float64 position_LO;
std_msgs::Float64 position_LA;
std_msgs::Float64 theta; 		//local orient...
std_msgs::Float64 alpha;		//LOS_angle
std_msgs::Float64 beta;  	//Angle b/w LOS & orientation
std_msgs::Float64 LOS_dist;

geometry_msgs::Twist velocities;

void P_Control()
{
  LOS_dist.data = sqrt((target_LO.data-position_LO.data)*(target_LO.data-position_LO.data)+(target_LA.data-position_LA.data)*(target_LA.data-position_LA.data));

  alpha.data = atan2((target_LO.data-position_LO.data),(target_LA.data-position_LA.data));

  beta.data = 3.14*theta.data/180.0 - alpha.data;
  if (beta.data >= 3.14)
    beta.data -= 2*3.14;
  if (beta.data <= -3.14)
    beta.data += 2*3.14;

  cout<<"\nAngle b/w LOS & orientation: "<<beta.data*180.0/3.14<<"\nLOS angle: "<<alpha.data*180.0/3.14<<"\nOrientation angle: "<<theta.data<<"\nDistance:"<<LOS_dist.data<<endl;

  velocities.linear.z = 0;
  velocities.angular.x = 0;
  velocities.angular.y = 0;

  if (LOS_dist.data >= 0.00092)
  {
    velocities.linear.x = (Vk*sin(3.14*theta.data/180.0));
    velocities.linear.y = (Vk*cos(3.14*theta.data/180.0));
    velocities.angular.z = kp*beta.data;
  }
  else
  {
    velocities.linear.x = 0;
    velocities.linear.y = 0;
    velocities.angular.z = 0;
  }
}

void global_Callback(const sensor_msgs::NavSatFix::ConstPtr& cordinate)
{
  position_LA.data = cordinate->latitude;
  position_LO.data = cordinate->longitude;
}

void compass_Callback(const std_msgs::Float64::ConstPtr& angle)
{
  theta.data = angle->data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "copter_P_control");

  ros::NodeHandle l, m, n;

  ros::Subscriber global_sub = l.subscribe("/mavros/global_position/global", 1, global_Callback);
  ros::Subscriber compass_sub = m.subscribe("/mavros/global_position/compass_hdg", 1, compass_Callback);

  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 1);

  position_LO.data = 0;
  position_LA.data = 0;
  theta.data = 0;
  
  cout<<"Target-Latitutde: ";
  cin>>target_LA.data;
  cout<<"Target-Longitude: ";
  cin>>target_LO.data;

  ros::Rate r(100);
  while (ros::ok())
  {
    ros::spinOnce();
    P_Control();
    pub.publish(velocities);
    r.sleep();
  }

  return 0;
}
