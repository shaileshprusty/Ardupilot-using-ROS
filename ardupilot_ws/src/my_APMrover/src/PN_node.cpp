/*
 * =====================================================================================
 *
 *       Filename:  PN_node.cpp
 *
 *    Description:  Proportional Navigation for Rover
 *
 *        Version:  1.0
 *        Created:  Wednesday 09 May 2018 07:17:34  IST
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

#define Vk 1

#define kp 0

double a, b, x=0, y=0, theta=0, alpha;

geometry_msgs::Twist velocities;

void pn()
{
  ROS_INFO("x = %f \ny = %f \nangle = %f \n\n", x, y, theta);
  velocities.linear.x = Vk*(a-x)/sqrt((a-x)*(a-x) + (b-y)*(b-y));
  velocities.linear.y = Vk*(b-y)/sqrt((a-x)*(a-x) + (b-y)*(b-y));
  velocities.linear.z=0;
  alpha = theta - atan((b-y)/(a-x));
  velocities.angular.x=0;
  velocities.angular.y=0;
  velocities.angular.z = kp*alpha;
  ROS_INFO("Vx = %f \nVy = %f \nVz= %f \n<x = %f \n<y = %f \n<z = %f \n\n", velocities.linear.x, velocities.linear.y, velocities.linear.z, velocities.angular.x, velocities.angular.y, velocities.angular.z);
}

void global_Callback(const sensor_msgs::NavSatFix::ConstPtr& cordinate)
{
  x = cordinate->latitude;
  y = cordinate->longitude;
}

void compass_Callback(const std_msgs::Float64::ConstPtr& angle)
{
  theta = angle->data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "PN_node");

  ros::NodeHandle l, m, n;

  ros::Subscriber global_sub = l.subscribe("/mavros/global_position/global", 1000, global_Callback);
  ros::Subscriber compass_sub = m.subscribe("/mavros/global_position/compass_hdg", 1000, compass_Callback);

  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel", 1000);

  std::cout<<"x-coordinate: ";
  std::cin>>a;
  std::cout<<"y-coordinate: ";
  std::cin>>b;
  
  ros::Rate r(100);
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
    pn();
    pub.publish(velocities);
  }

  return 0;
}
