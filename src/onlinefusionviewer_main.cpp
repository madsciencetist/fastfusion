/*
 * viewerfusion_main.cpp
 *
 *  Created on: Jun 23, 2013
 *      Author: steinbrf
 */

#include <auxiliary/multivector.h>

#include <opencv2/opencv.hpp>
#include <omp.h>
#include <stdio.h>
#include <math.h>
#include <string>
#include <fstream>
#include <sstream>
#include <Eigen/Geometry>
#include <auxiliary/debug.hpp>

//#include <fusionGPU/texturefusion.hpp>
//#include <fusionGPU/geometryfusion_single_soa.hpp>
//#include <fusionGPU/geometryfusion_single_aos.hpp>
//#include <fusionGPU/geometryfusion_dynamic_aos.hpp>
//#include <fusionGPU/geometryfusion_dynamic_multiscale.hpp>
//#include <fusionGPU/geometryfusion_mipmap.hpp>
#include <fusion/geometryfusion_mipmap_cpu.hpp>

#include "onlinefusionviewer.hpp"
#include <QtWidgets/qapplication.h>

#include <tclap/CmdLine.h>
#include <fusion/mesh.hpp>

#define BOXDELTA 0.001
//#define VOLUMERADIUS 1.5
#define VOLUMERADIUS 1.4
#define USE_ORIGINAL_VOLUME 1

#include <fusion/definitions.h>

#include <deque>
#include <list>

//CameraInfo kinectPoseFromSophus(Sophus::SE3 pos){
//	CameraInfo result;
//	cv::Mat intrinsic = cv::Mat::eye(3,3,cv::DataType<double>::type);
//	//Kinect Intrinsic Parameters
//	//Kinect
//	intrinsic.at<double>(0,0) = intrinsic.at<double>(1,1) = 525.0;
//	intrinsic.at<double>(0,2) = 319.5;
//	intrinsic.at<double>(1,2) = 239.5;
//	//DLR
////	intrinsic.at<double>(0,0) = intrinsic.at<double>(1,1) = 551.85425;
////	intrinsic.at<double>(0,2) = 376.0;
////	intrinsic.at<double>(1,2) = 240.0;
//
//	result.setIntrinsic(intrinsic);
//	Eigen::Matrix3d rotation = pos.rotation_matrix();
//	cv::Mat rotation2 = cv::Mat::eye(3,3,cv::DataType<double>::type);
//	for(int i=0;i<3;i++) for(int j=0;j<3;j++) rotation2.at<double>(i,j) = rotation(i,j);
//	result.setRotation(rotation2);
//	Eigen::Vector3d translation = pos.translation();
//	cv::Mat translation2 = cv::Mat::zeros(3,1,cv::DataType<double>::type);
//	for(int i=0;i<3;i++) translation2.at<double>(i,0) = translation(i);
//	result.setTranslation(translation2);
//	return result;
//}

//CameraInfo kinectPoseFromSophus(Sophus::SE3 pos,float fx, float fy, float cx, float cy){
//	CameraInfo result;
//	cv::Mat intrinsic = cv::Mat::eye(3,3,cv::DataType<double>::type);
//	//Kinect Intrinsic Parameters
//	intrinsic.at<double>(0,0) = fx;
//	intrinsic.at<double>(1,1) = fy;
//	intrinsic.at<double>(0,2) = cx;
//	intrinsic.at<double>(1,2) = cy;
//
//	result.setIntrinsic(intrinsic);
//	Eigen::Matrix3d rotation = pos.rotation_matrix();
//	cv::Mat rotation2 = cv::Mat::eye(3,3,cv::DataType<double>::type);
//	for(int i=0;i<3;i++) for(int j=0;j<3;j++) rotation2.at<double>(i,j) = rotation(i,j);
//	result.setRotation(rotation2);
//	Eigen::Vector3d translation = pos.translation();
//	cv::Mat translation2 = cv::Mat::zeros(3,1,cv::DataType<double>::type);
//	for(int i=0;i<3;i++) translation2.at<double>(i,0) = translation(i);
//	result.setTranslation(translation2);
//	return result;
//}

CameraInfo kinectPoseFromEigen(std::pair<Eigen::Matrix3d, Eigen::Vector3d> pos, float fx, float fy, float cx, float cy)
{
	CameraInfo result;
	cv::Mat intrinsic = cv::Mat::eye(3, 3, cv::DataType<double>::type);
	//Kinect Intrinsic Parameters
	intrinsic.at<double>(0, 0) = fx;
	intrinsic.at<double>(1, 1) = fy;
	intrinsic.at<double>(0, 2) = cx;
	intrinsic.at<double>(1, 2) = cy;

	result.setIntrinsic(intrinsic);
	Eigen::Matrix3d rotation = pos.first;
	cv::Mat rotation2 = cv::Mat::eye(3, 3, cv::DataType<double>::type);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			rotation2.at<double>(i, j) = rotation(i, j);
	result.setRotation(rotation2);
	Eigen::Vector3d translation = pos.second;
	cv::Mat translation2 = cv::Mat::zeros(3, 1, cv::DataType<double>::type);
	for (int i = 0; i < 3; i++)
		translation2.at<double>(i, 0) = translation(i);
	result.setTranslation(translation2);
	return result;
}

void filterimage(cv::Mat &image)
{
	cv::Mat input = image.clone();
	for (int x = 1; x < image.cols - 1; x++)
	{
		for (int y = 1; y < image.rows - 1; y++)
		{
			if (std::isfinite(input.at<float>(y, x)))
			{
				float sum = 0.0f;
				float count = 0.0f;
				for (int dx = -1; dx <= 1; dx++)
				{
					for (int dy = -1; dy <= 1; dy++)
					{
						if (std::isfinite(input.at<float>(y + dy, x + dx)) && fabs(input.at<float>(y, x) - input.at<float>(y + dy, x + dx)) < 0.1f)
						{
							sum += input.at<float>(y + dy, x + dx);
							count += 1.0f;
						}
					}
				}
				image.at<float>(y, x) = sum / count;
			}
		}
	}
}

#include <list>

int main(int argc, char *argv[])
{

	std::string meshname = "";
	bool noViewer = true;
	bool bufferImages = false;
	bool volumeColor = true;

	bool useLoopClosures = false;

	unsigned int startimage = 0;
	unsigned int endimage = 1000000;
	unsigned int imageStep = 1;

	float maxCamDistance = MAXCAMDISTANCE;
	float minDepth = 0.3f;
	float scale = DEFAULT_SCALE;
	float threshold = DEFAULT_SCALE;

	float imageDepthScale = 1000; // DEPTHSCALE;

	bool threadMeshing = true;
	bool threadFusion = true;
	bool threadImageReading = false;
	bool performIncrementalMeshing = true;

	int depthConstistencyChecks = 0;

	TCLAP::CmdLine cmdLine("onlinefusion");

	TCLAP::ValueArg<std::string> loadMeshArg("l", "loadmesh", "Loads this mesh", false, meshname, "string");
	TCLAP::SwitchArg threadMeshingArg("", "thread-meshing", "Thread the Meshing inside the Fusion", threadMeshing);
	TCLAP::SwitchArg threadFusionArg("", "thread-fusion", "Thread the Fusion inside the Viewer", threadFusion);
	TCLAP::SwitchArg threadImageReadingArg("", "thread-image", "Thread reading the Images from Hard Disk", threadImageReading);
	TCLAP::SwitchArg viewerArg("v", "viewer", "Show a Viewer after Fusion", !noViewer);
	TCLAP::SwitchArg bufferArg("b", "buffer", "Buffer all Images", bufferImages);
	TCLAP::SwitchArg useLoopClosuresArg("c", "loopclosures", "Read Multiple Trajectories and perform Loop Closures", useLoopClosures);
	TCLAP::SwitchArg performIncrementalMeshingArg("", "incremental-meshing", "Perform incremental Meshing", performIncrementalMeshing);
	TCLAP::ValueArg<int> startimageArg("s", "startimage", "Number of the Start Image", false, startimage, "int");
	TCLAP::ValueArg<int> endimageArg("e", "endimage", "Number of the End Image", false, endimage, "int");
	TCLAP::ValueArg<int> imageStepArg("k", "imagestep", "Use every kth step", false, imageStep, "int");
	TCLAP::ValueArg<int> depthConsistencyChecksArg("", "consistency-checks", "Number of Depth Consistency Checks", false, depthConstistencyChecks, "int");
	TCLAP::ValueArg<float> maxCamDistanceArg("", "max-camera-distance", "Maximum Camera Distance to Surface", false, maxCamDistance, "float");
	TCLAP::ValueArg<float> minDepthArg("", "min-depth", "Minimum depth measurement considered valid", false, minDepth, "float");
	TCLAP::ValueArg<float> scaleArg("", "scale", "Size of the Voxel", false, scale, "float");
	TCLAP::ValueArg<float> thresholdArg("", "threshold", "Threshold", false, threshold, "float");
	TCLAP::ValueArg<float> imageDepthScaleArg("", "imagescale", "Image Depth Scale", false, imageDepthScale, "float");
	TCLAP::UnlabeledMultiArg<std::string> bagsArg("filenames", "The rosbag filenames", false, "string");

	cmdLine.add(loadMeshArg);
	cmdLine.add(threadMeshingArg);
	cmdLine.add(threadFusionArg);
	cmdLine.add(threadImageReadingArg);
	cmdLine.add(viewerArg);
	cmdLine.add(bufferArg);
	cmdLine.add(useLoopClosuresArg);
	cmdLine.add(performIncrementalMeshingArg);
	cmdLine.add(startimageArg);
	cmdLine.add(endimageArg);
	cmdLine.add(imageStepArg);
	cmdLine.add(depthConsistencyChecksArg);
	cmdLine.add(maxCamDistanceArg);
	cmdLine.add(scaleArg);
	cmdLine.add(thresholdArg);
	cmdLine.add(imageDepthScaleArg);

	cmdLine.add(bagsArg);
	cmdLine.parse(argc, argv);

	meshname = loadMeshArg.getValue();
	threadMeshing = threadMeshingArg.getValue();
	threadFusion = threadFusionArg.getValue();
	threadImageReading = threadImageReadingArg.getValue();
	noViewer = !viewerArg.getValue();
	bufferImages = bufferArg.getValue();
	useLoopClosures = useLoopClosuresArg.getValue();
	performIncrementalMeshing = performIncrementalMeshingArg.getValue();
	startimage = startimageArg.getValue();
	endimage = endimageArg.getValue();
	imageStep = imageStepArg.getValue();
	if (imageStep < 1)
		imageStep = 1;
	depthConstistencyChecks = depthConsistencyChecksArg.getValue();
	maxCamDistance = maxCamDistanceArg.getValue();
	scale = scaleArg.getValue();
	threshold = thresholdArg.getValue();
	imageDepthScale = imageDepthScaleArg.getValue();

	if (threadMeshing)
		fprintf(stderr, "\nMeshing will run in a separate Thread\n");
	else
		fprintf(stderr, "\nMeshing will NOT run in a separate Thread\n");
	if (threadFusion)
		fprintf(stderr, "\nFusion will run in a separate Thread\n");
	if (threadImageReading)
		fprintf(stderr, "\nImage Reading will run in a separate Thread\n");

	CameraInfo startpos;

	std::string defaultname = "../test/rgbd_dataset_freiburg3_long_office_household/associationVICOM.txt";

	std::string tempfileprefix = "debug/";

	std::vector<std::string> bags = bagsArg.getValue();

	if (!bags.size())
	{
		throw std::invalid_argument("Error: no rosbag provided");
	}

//	fprintf(stderr,"\nCreating Mipmapping GPU Octree");
//	Fusion_AoS *fusion = new FusionMipMap(0,0,0,DEFAULT_SCALE,DISTANCETHRESHOLD,0,volumeColor);

	fprintf(stderr, "\nCreating Mipmapping CPU Octree");
	FusionMipMapCPU *fusion = new FusionMipMapCPU(0, 0, 0, scale, threshold, 0, volumeColor);

	fusion->setThreadMeshing(threadMeshing);
	fusion->setDepthChecks(depthConstistencyChecks);
	fusion->setIncrementalMeshing(performIncrementalMeshing);

	fprintf(stderr, "\nCreating Viewer");
	QApplication application(argc, argv);

	//	OnlineFusionViewerManipulated viewer(false);
	OnlineFusionViewerManipulated *viewerpointer = new OnlineFusionViewerManipulated(false);
	OnlineFusionViewerManipulated &viewer = *viewerpointer;

	fprintf(stderr, "\nSetting Viewer Parameters");
	viewer._fusion = fusion;
	viewer.setWindowTitle("Fusion Volume");
	viewer._bags = bags;
	viewer._threadFusion = threadFusion;
	viewer._threadImageReading = threadImageReading;
	viewer.show();
	viewer._imageDepthScale = imageDepthScale;
	viewer._maxCamDistance = maxCamDistance;
	viewer._minDepth = minDepth;
	viewer._firstFrame = (long int)startimage;
	viewer._currentFrame = (long int)startimage - 1;
	fprintf(stderr, "\nSet Viewer Frame to %li", (long int)viewer._currentFrame);
	viewer._nextStopFrame = endimage;
	fprintf(stderr, "\nStarting Qt-Application");
	application.exec();

	fprintf(stderr, "\nDeleting Viewer");
	delete viewerpointer;

	fprintf(stderr, "\nProgram %s exiting.\n\n", argv[0]);
	fprintf(stderr, "\nPress Enter exit the Program");
	char input[256];
	fprintf(stderr, "%s", fgets(input, 256, stdin));
	return 0;
}
