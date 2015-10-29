/*
 * main_slam_simulation.cpp
 *
 *  Created on: Sep 24, 2015
 *      Author: karrerm
 */

#include "point_cloud_fusion/PointCloudFusion.hpp"

/**
 * ROS-node to simulate pose messages from a SLAM system and Depth images from a RGB-D camera system
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "slam_simulation_node");

  ROS_INFO("\nStarting slam simulation node with name %s\n", ros::this_node::getName().c_str());

  PointCloudFusion fusion;
  fusion.run();
  return 0;
}



