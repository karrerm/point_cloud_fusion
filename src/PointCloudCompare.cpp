/*
 * PointCloudCompare.cpp
 *
 *  Created on: Jan. 5, 2016
 *      Author: Marco Karrer
 */

#include "point_cloud_compare/PointCloudCompare.hpp"


PointCloudCompare::PointCloudCompare() : nodeLocal_("~") {
	bool loadSuccess = true;
	//-- Load Parameters
	loadSuccess &= nodeLocal_.getParam("groundTruthLocation", groundTruthLocation_);
	loadSuccess &= nodeLocal_.getParam("groundTruthName", groundTruthName_);
	loadSuccess &= nodeLocal_.getParam("reconstructionLocation", reconstructionLocation_);
	loadSuccess &= nodeLocal_.getParam("reconstructionName", reconstructionName_);
	loadSuccess &= nodeLocal_.getParam("subsampleGroundTruth", subsampleGroundTruth_);
	loadSuccess &= nodeLocal_.getParam("subsampleReconstruction",subsampleReconstruction_);
	loadSuccess &= nodeLocal_.getParam("largestError", largestError_);
	loadSuccess &= nodeLocal_.getParam("saveName",saveName_);
	loadSuccess &= nodeLocal_.getParam("maxIterations", maxIterations_);
	loadSuccess &= nodeLocal_.getParam("maxDistance", maxDistance_);
	loadSuccess &= nodeLocal_.getParam("filterRadius", filterRadius_);
	loadSuccess &= nodeLocal_.getParam("filterNeighbours",filterNeighbours_);
	loadSuccess &= nodeLocal_.getParam("sequentialMeshes",sequentialMeshes_);
	XmlRpc::XmlRpcValue T_w_rec;
	loadSuccess &= nodeLocal_.getParam("InitialGuess/T_w_rec", T_w_rec);
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			T_w_rec_(i,j) = (double)T_w_rec[i][j];
		}
	}
	T_w_rec_ = T_w_rec_.inverse();
	colorScale_ = 2.0/largestError_;
	//-- Display if Parameters were read successfully
	if (loadSuccess) {
		std::cout << "Could successfully read the parameters" << std::endl;
	} else {
 		std::cout << "Could NOT read the parameters" << std::endl;
		return;
	}

}

void PointCloudCompare::run() {
	//-- Start reading the point clouds
	ros::Time startTime = ros::Time::now();
	float x,y,z;
	int dummy,r,g,b, lines;
	pcl::PointXYZRGB tempPoint;
	//-- Read the input clouds
	std::cout << "Read Ground-Truth" << std::endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr groundTruthPC (new pcl::PointCloud<pcl::PointXYZRGB>);
	std::cout << "Read Reconstruction" << std::endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputReconstrPC (new pcl::PointCloud<pcl::PointXYZRGB>);
	std::string groundTruthFullPathName = groundTruthLocation_ + "/" + groundTruthName_ + ".ply";
	std::string reconstructionFullPathName = reconstructionLocation_ + "/" + reconstructionName_ + ".ply";
	pcl::io::loadPLYFile (groundTruthFullPathName, *groundTruthPC);
	pcl::io::loadPLYFile (reconstructionFullPathName, *inputReconstrPC);

	//-- Subsampling
	if (subsampleGroundTruth_ != 0.0) {
		std::cout << "Downsample Ground-Truth" << std::endl;
		pcl::VoxelGrid<pcl::PointXYZRGB> sor;
		sor.setInputCloud (groundTruthPC);
		sor.setLeafSize (subsampleGroundTruth_,subsampleGroundTruth_,subsampleGroundTruth_);
		sor.filter (*groundTruthPC);
	}
	if (subsampleReconstruction_ != 0.0) {
		std::cout << "Downsample Reconstruction" << std::endl;
		pcl::VoxelGrid<pcl::PointXYZRGB> sor;
		sor.setInputCloud (inputReconstrPC);
		sor.setLeafSize(subsampleReconstruction_, subsampleReconstruction_, subsampleReconstruction_);
		sor.filter (*inputReconstrPC);
	}

	//-- Preorientate PointCloud
	std::cout << "Pre-align reconstruction using initial guess" << std::endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr preOrientReconstrPC(new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::transformPointCloud(*inputReconstrPC, *preOrientReconstrPC, T_w_rec_);
	pcl::io::savePLYFile("/home/karrer/DebugCloud.ply",*preOrientReconstrPC);
	//-- Perform ICP alignment
	std::cout << "Perform ICP-alignment" << std::endl;
	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	icp.setMaximumIterations (maxIterations_);
	icp.setMaxCorrespondenceDistance (maxDistance_);
	icp.setInputSource(preOrientReconstrPC);
	icp.setInputTarget(groundTruthPC);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr alignedReconstrPC (new pcl::PointCloud<pcl::PointXYZRGB>());
	icp.align(*alignedReconstrPC);
	std::cout << icp.getFinalTransformation() << std::endl;

	//-- Compute Differenced
	std::cout << "Compute Difference" << std::endl;
	nnSearch2colorEncoding(groundTruthPC, alignedReconstrPC);
	ros::Time endTime = ros::Time::now();
	std::cout << "Evaluation took " << (endTime - startTime).toSec() << " seconds" << std::endl;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("visualization pc"));
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15);
	viewer->initCameraParameters ();
	viewer->addCoordinateSystem (1);
	viewer->addPointCloud(alignedReconstrPC,"1");
	std::string completeSaveNamePLY = reconstructionLocation_ + "/" + saveName_ + ".ply";
	//std::string completeSaveNamePCD = fileLocation_ + saveName_ + ".pcd";
	pcl::io::savePLYFile(completeSaveNamePLY,*alignedReconstrPC);
	//pcl::io::savePCDFile(completeSaveNamePCD,*mergedPointCloud_,0);
	while(!viewer->wasStopped()){
		viewer->spinOnce (10);
	}
}

void PointCloudCompare::nnSearch2colorEncoding(pcl::PointCloud<pcl::PointXYZRGB>::Ptr groundTruthPC,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr reconstructionPC) {
//-- Find the nearest neighbour in the ground truth for every point in the reconstruction and encode the distance
//-- as a color
	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
	kdtree.setInputCloud(groundTruthPC);
	int K = 1;
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);
	uint8_t  r, g, b;
	double distance;
	std::vector<double> distanceError;
	double meanErr;
	for (int i = 0; i < reconstructionPC->points.size(); i++) {
		kdtree.nearestKSearch (reconstructionPC->points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance);
		distance = std::sqrt((double)pointNKNSquaredDistance[0]);
		distance2color(distance, &r, &g, &b);
		reconstructionPC->points[i].r = r;
		reconstructionPC->points[i].g = g;
		reconstructionPC->points[i].b = b;
		if (distance < 2*largestError_) {
			distanceError.push_back(distance);
			meanErr += distance;
		}
	}
	meanErr = meanErr / distanceError.size();
	//-- Sort the distances (e.g. to compute median value)
	std::sort(distanceError.begin(),distanceError.end());
	//-- Compute the Standard Deviation and Median value
	double varErr = 0;
	for (int i = 0; i < distanceError.size(); i++) {
		varErr += (distanceError[i] - meanErr) * (distanceError[i] - meanErr);
	}
	varErr = varErr / distanceError.size();
	int medianInd = (int) std::round(((double)distanceError.size())/2.0);
	double medianErr = distanceError[medianInd];
	std::string fileNameErrStatistics = reconstructionLocation_ + "/" + saveName_ + "_results.txt";
	std::ofstream file(fileNameErrStatistics);
	file << "Error Statistic for " << reconstructionName_ << std::endl;
	file << "----------------------------------------------------------\n" << std::endl;
	file << "Ground Truth Point Cloud" << std::endl;
	file << " Name:                   " << 	groundTruthName_ << std::endl;
	file << " Subsampling Voxel-Size: " << subsampleGroundTruth_ << std::endl;
	file << " Nr. of Points:          " << groundTruthPC->points.size() << std::endl << std::endl;
	file << "Evaluation Point Cloud   " << std::endl;
	file << " Name:                   " << reconstructionName_ << std::endl;
	file << " Subsampling Voxel-Size: " << subsampleReconstruction_ << std::endl;
	file << " Nr. of Points:          " << reconstructionPC->points.size() << std::endl;
	file << " Defined max. Error [m]: " << largestError_ << " (corresponds to deep red)" << std::endl;
	file << " Mean Error [m]          " << meanErr << std::endl;
	file << " Median Error [m]        " << medianErr << std::endl;
	file << " Error Variance [m^2]    " << varErr << std::endl;
	file.close();
}

void PointCloudCompare::distance2color(double distance, uint8_t *r, uint8_t *g, uint8_t *b) {
//-- Convert Distance measurement to color encoding. The parameter largestError_ corresponds to red
//-- (also all values that are larger), the value 0 corresponds to blue.
	if (distance > largestError_){
		distance = largestError_;
	}
	//-- Convert Distance to value between 0 and 1
	double value = distance * colorScale_-1.0;

	//-- Convert Value to RGB (with values between 0 and 1)
	double r_d = colorMapRed(value);
	double g_d = colorMapGreen(value);
	double b_d = colorMapBlue(value);
	//-- Convert RGB values to unsigned 8bit integers
	*r = (uint8_t) std::round(r_d*255);
	*g = (uint8_t) std::round(g_d*255);
	*b = (uint8_t) std::round(b_d*255);
}


double PointCloudCompare::interpolate( double val, double y0, double x0, double y1, double x1 ) {
    return (val-x0)*(y1-y0)/(x1-x0) + y0;
}

double PointCloudCompare::colorMapBase( double val ) {
    if ( val <= -0.75 ) return 0;
    else if ( val <= -0.25 ) return interpolate( val, 0.0, -0.75, 1.0, -0.25 );
    else if ( val <= 0.25 ) return 1.0;
    else if ( val <= 0.75 ) return interpolate( val, 1.0, 0.25, 0.0, 0.75 );
    else return 0.0;
}

double PointCloudCompare::colorMapRed(double value) {
    return colorMapBase(value - 0.5 );
}
double PointCloudCompare::colorMapGreen(double value) {
    return colorMapBase(value);
}
double PointCloudCompare::colorMapBlue(double value) {
    return colorMapBase(value + 0.5 );
}


void PointCloudCompare::filtering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr unfilteredPointCloud,
								pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredPointCloud) {
	//mergedPointCloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
	//-- build the filter
    outrem.setInputCloud(unfilteredPointCloud);
    outrem.setRadiusSearch(filterRadius_);
    outrem.setMinNeighborsInRadius (filterNeighbours_);
    //-- apply filter
    outrem.filter (*filteredPointCloud);
}

