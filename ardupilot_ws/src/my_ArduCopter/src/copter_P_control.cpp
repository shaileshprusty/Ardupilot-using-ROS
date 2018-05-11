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

#define Vk 1

#define kp 0.2

std_msgs::Float64 a;
std_msgs::Float64 b;
std_msgs::Float64 x;
std_msgs::Float64 y;
std_msgs::Float64 theta;
std_msgs::Float64 alpha;
std_msgs::Float64 beta;
std_msgs::Float64 LOS_dist;

geometry_msgs::Twist velocities;

void P_Control()
{
  LOS_dist.data = sqrt((a.data-x.data)*(a.data-x.data)+(b.data-y.data)*(b.data-y.data));

  alpha.data = atan2((a.data-x.data),(b.data-y.data));

  beta.data = 3.14*theta.data/180.0 - alpha.data;
  if (beta.data >= 3.14)
    beta.data -= 2*3.14;
  if (beta.data <= -3.14)
    beta.data += 2*3.14;

  cout<<"\nAngle b/w LOS & orientation: "<<beta.data<<"\nLOS angle: "<<alpha.data<<"\nOrientation angle: "<<theta.data<<endl;

  velocities.linear.z = 0;
  velocities.angular.x = 0;
  velocities.angular.y = 0;

  if (LOS_dist.data <= 0.0000000002)
  {
    velocities.linear.x = Vk*sin(3.14*theta.data/180.0);
    velocities.linear.y = Vk*cos(3.14*theta.data/180.0);
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
  x.data = cordinate->latitude;
  y.data = cordinate->longitude;
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

  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 1000);

  x.data = 0;
  y.data = 0;
  theta.data = 0;
  
  cout<<"x-coordinate: ";
  cin>>a.data;
  cout<<"y-coordinate: ";
  cin>>b.data;

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
