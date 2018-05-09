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

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/NavSatFix.h"

double x=0, y=0, theta=0;

void pn()
{
  ROS_INFO("x = %f \ny = %f \nangle = %f \n\n", x, y, theta);
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

  ros::NodeHandle m, n;

  ros::Subscriber global_sub = m.subscribe("", 1000, global_Callback);
  ros::Subscriber compass_sub = n.subscribe("", 1000, compass_Callback);

  ros::Rate r(100);
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
    pn();
  }

  return 0;
}
