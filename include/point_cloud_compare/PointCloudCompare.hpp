/*
 * PointCloudCompare.hpp
 *
 *  Created on: Jan. 5, 2016
 *      Author: Marco Karrer
 */

#ifndef INCLUDE_POINT_CLOUD_COMPARE_HPP_
#define INCLUDE_POINT_CLOUD_COMPARE_HPP_

#include <ros/ros.h>
#include <ros/node_handle.h>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>


#include <iostream>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <vector>
#include <opencv2/opencv.hpp>

struct encodedColor {
	unsigned char r, g, b;
};

class PointCloudCompare {
public:
	//-- Constructor and Destructor
	PointCloudCompare();
	~PointCloudCompare(){};
	//-- Start the alignment
	void run();
protected:
	void nnSearch2colorEncoding(pcl::PointCloud<pcl::PointXYZRGB>::Ptr unfilteredPointCloud,
								pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredPointCloud);
	void filtering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr unfilteredPointCloud,
								pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredPointCloud);
	void distance2color(double distance, uint8_t *r, uint8_t *g, uint8_t *b);
	void reorientate();

	//-- ColorMapping
	double interpolate(double val, double y0, double x0, double y1, double x1);
	double colorMapBase(double val);
	double colorMapRed(double value);
	double colorMapGreen(double value);
	double colorMapBlue(double value);

	int numberOfClouds_;
	//-- Strings for the different files
	std::string baseName_, groundTruthLocation_, groundTruthName_, reconstructionLocation_,  reconstructionName_ ,saveName_;
	//-- Input and merged output cloud
	std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pointCloudVect_;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergedPointCloud_;
	//-- ROS Handles
	ros::NodeHandle node_, nodeLocal_;
	double maxDistance_;
	int maxIterations_;
	//-- Filter Properties
	double filterRadius_;
	int filterNeighbours_;
	//-- Reorientation of output cloud
	bool reorientatePC_;
	Eigen::Matrix4d T_w_rec_;
	//-- Subsampling parameters
	double subsampleGroundTruth_;
	double subsampleReconstruction_;

	//-- Visualization
	double largestError_;			// Corresponds to deep red
	double colorScale_;
};


#endif /* INCLUDE_SLAM_SIMULATION_NODE_SLAMSIMULATION_HPP_ */
