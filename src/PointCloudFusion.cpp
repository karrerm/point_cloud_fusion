/*
 * SlamSimulation.cpp
 *
 *  Created on: Sep 25, 2015
 *      Author: karrer
 */

#include "point_cloud_fusion/PointCloudFusion.hpp"


PointCloudFusion::PointCloudFusion() : nodeLocal_("~") {
	bool loadSuccess = true;
	//-- Load Parameters
	loadSuccess &= nodeLocal_.getParam("numberOfClouds", numberOfClouds_);
	loadSuccess &= nodeLocal_.getParam("baseName", baseName_);
	loadSuccess &= nodeLocal_.getParam("fileLocation", fileLocation_);
	loadSuccess &= nodeLocal_.getParam("saveName",saveName_);
	loadSuccess &= nodeLocal_.getParam("maxIterations", maxIterations_);
	loadSuccess &= nodeLocal_.getParam("maxDistance", maxDistance_);

}

void PointCloudFusion::run() {
	//-- Start reading the point clouds
	float x,y,z;
	int dummy,r,g,b, lines;
	pcl::PointXYZRGB tempPoint;
	for (int i = 0; i < numberOfClouds_; i++) {
		//-- Read initial Guess
		std::vector<float> t_temp,q_temp;
		std::string currentPosition = baseName_ + std::to_string(i+1) + "/t";
		std::string currentPose = baseName_ + std::to_string(i+1) + "/q";
		nodeLocal_.getParam(currentPosition,t_temp);
		Eigen::Vector3f t;
		t(0) = t_temp[0]; t(1) = t_temp[1]; t(2) = t_temp[2];
		nodeLocal_.getParam(currentPose,q_temp);
		Eigen::Quaternionf q(q_temp[3], q_temp[0], q_temp[1],q_temp[2]);
		std::cout << t << std::endl;

		//-- Recreate File Name
		std::string currentFile = fileLocation_ + baseName_ + std::to_string(i+1) + ".PTS";
		//-- Read File
		pointCloudVect_.push_back( pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>()));
		std::ifstream pointCloudStream;
		pointCloudStream.open(currentFile,std::ifstream::in);
		pointCloudStream >> lines;
		Eigen::Vector3f point;
		Eigen::Matrix3f R;
		R = (q.toRotationMatrix());
		for (int j = 0; j < lines; j++) {
			pointCloudStream >> point(0) >> point(1) >> point(2) >> dummy >> r >> g >> b;
			point = R*point - t;
			tempPoint.x = point(0); tempPoint.y = point(1); tempPoint.z = point(2);
			tempPoint.r = r; tempPoint.g = g; tempPoint.b = b;
			pointCloudVect_[i]->push_back(tempPoint);
		}
		std::cout << "Read " << i << " of " << numberOfClouds_ << " point clouds" << std::endl;
	}
	//-- Do the actual alignment
	registration();
	//-- Visualize and store the resulting merged point cloud
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("visualization pc"));
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15);
	viewer->initCameraParameters ();
	viewer->addCoordinateSystem (1);
	std::cout << pointCloudVect_[0]->size() << std::endl;
	viewer->addPointCloud(mergedPointCloud_,"1");
	std::string completeSaveNamePLY = fileLocation_ + saveName_ + ".ply";
	std::string completeSaveNamePCD = fileLocation_ + saveName_ + ".pcd";
	pcl::io::savePLYFile(completeSaveNamePLY,*mergedPointCloud_);
	pcl::io::savePCDFile(completeSaveNamePCD,*mergedPointCloud_,0);
	while(!viewer->wasStopped()){
		viewer->spinOnce (10);
	}
}

void PointCloudFusion::registration() {
	//-- Set first Point-Cloud as default
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergedPointCloud (new pcl::PointCloud<pcl::PointXYZRGB>());
	for (int i = 0; i < pointCloudVect_[0]->size(); i++) {
		mergedPointCloud->push_back(pointCloudVect_[0]->points[i]);
	}
	//-- Subsequently align the other point clouds
	for (int i = 1; i < pointCloudVect_.size(); i++) {
		std::cout << "Align Point cloud #" << i+1 << " of " << pointCloudVect_.size() << std::endl;
		pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
		icp.setMaximumIterations (maxIterations_);
		icp.setMaxCorrespondenceDistance (maxDistance_);
		icp.setInputCloud(pointCloudVect_[i]);
		icp.setInputTarget(mergedPointCloud);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final (new pcl::PointCloud<pcl::PointXYZRGB>());
		icp.align(*Final);
		//-- Add Point Cloud to merged
		for (int j = 0; j < Final->size(); j++) {
			mergedPointCloud->push_back(Final->points[j]);
		}
	}
	mergedPointCloud_ = mergedPointCloud;

}
