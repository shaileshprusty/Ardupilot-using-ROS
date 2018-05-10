/*
 * =====================================================================================
 *
 *       Filename:  PN_node_turtlesim.cpp
 *
 *    Description:  Proportional Navigation for Turtlesim (Test Program)
 *
 *        Version:  1.0
 *        Created:  Wednesday 09 May 2018 09:59:29  IST
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Shailesh Chandra Prusty, shailesh.prusty@gmail.com
 *   Organization:  
 *
 * =====================================================================================
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

double X=0, Y=0, Theta=0;
geometry_msgs::Twist velocities;

void pn()
{
  ROS_INFO("x = %f \ny = %f \nangle = %f \n\n", X, Y, Theta);
  velocities.linear.x=0;
  velocities.linear.y=0;
  velocities.linear.z=0;
  velocities.angular.x=0;
  velocities.angular.y=0;
  velocities.angular.z=0;
  ROS_INFO("Vx = %f \nVy = %f \nVz= %f \n<x = %f \n<y = %f \n<z = %f \n\n", velocities.linear.x, velocities.linear.y, velocities.linear.z, velocities.angular.x, velocities.angular.y, velocities.angular.z);
}

void Callback(const turtlesim::Pose::ConstPtr& Data)
{
  X = Data->x;
  Y = Data->y;
  Theta = Data->theta;
}

int main(int argv, char **argc)
{
  ros::init(argv, argc, "PN_node_turtlesim");

  ros::NodeHandle m;
  //ros::NodeHandle n;

  ros::Subscriber sub = m.subscribe("/turtle1/pose", 1000, Callback);
  //ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);

  ros::Rate r(100);
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
    pn();
    //pub.publish(velocities);
  }

  return 0;
}
