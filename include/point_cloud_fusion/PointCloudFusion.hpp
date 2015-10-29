/*
 * SlamSimulation.hpp
 *
 *  Created on: Sep 25, 2015
 *      Author: karrer
 */

#ifndef INCLUDE_POINT_CLOUD_FUSION_HPP_
#define INCLUDE_POINT_CLOUD_FUSION_HPP_

#include <ros/ros.h>
#include <ros/node_handle.h>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/common/geometry.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>


#include <iostream>
#include <sstream>
#include <fstream>

#include <opencv2/opencv.hpp>

class PointCloudFusion {
public:
	//-- Constructor and Destructor
	PointCloudFusion();
	~PointCloudFusion(){};
	//-- Start the alignment
	void run();
protected:
	void registration();
	int numberOfClouds_;
	//-- Strings for the different files
	std::string baseName_, fileLocation_, saveName_;
	//-- Input and merged output cloud
	std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pointCloudVect_;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergedPointCloud_;
	//-- ROS Handles
	ros::NodeHandle node_, nodeLocal_;
	double maxDistance_;
	int maxIterations_;
};


#endif /* INCLUDE_SLAM_SIMULATION_NODE_SLAMSIMULATION_HPP_ */
