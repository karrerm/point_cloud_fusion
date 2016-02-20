/*
 * main_point_cloud_compare.cpp
 *
 *  Created on: Jan 5, 2016
 *      Author: Marco Karrer
 */

#include "point_cloud_compare/PointCloudCompare.hpp"

/**
 * ROS-node to evaluate reconstruction by comparing with ground truth
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud_compare");

  ROS_INFO("\nStarting point cloud compare node with name %s\n", ros::this_node::getName().c_str());

  PointCloudCompare compare;
  compare.run();
  return 0;
}



