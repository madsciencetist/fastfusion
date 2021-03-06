/*
 *  main.cpp
 *
 *  Created on: Feb 20, 2017
 *      Author: Mieos
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

#include <fusion/geometryfusion_mipmap_cpu.hpp>

#include "onlinefusionviewer.hpp"
#include <qapplication.h>

#include <tclap/CmdLine.h>
#include <fusion/mesh.hpp>



#define BOXDELTA 0.001
#define VOLUMERADIUS 1.4
#define USE_ORIGINAL_VOLUME 1

#include <fusion/definitions.h>


#include <deque>
#include <list>

CameraInfo kinectPoseFromEigen(std::pair<Eigen::Matrix3d,Eigen::Vector3d> pos,float fx, float fy, float cx, float cy){
   CameraInfo result;
   cv::Mat intrinsic = cv::Mat::eye(3,3,cv::DataType<double>::type);
   //Kinect Intrinsic Parameters
   intrinsic.at<double>(0,0) = fx;
   intrinsic.at<double>(1,1) = fy;
   intrinsic.at<double>(0,2) = cx;
   intrinsic.at<double>(1,2) = cy;

   result.setIntrinsic(intrinsic);
   Eigen::Matrix3d rotation = pos.first;
   cv::Mat rotation2 = cv::Mat::eye(3,3,cv::DataType<double>::type);
   for(int i=0;i<3;i++) for(int j=0;j<3;j++) rotation2.at<double>(i,j) = rotation(i,j);
   result.setRotation(rotation2);
   Eigen::Vector3d translation = pos.second;
   cv::Mat translation2 = cv::Mat::zeros(3,1,cv::DataType<double>::type);
   for(int i=0;i<3;i++) translation2.at<double>(i,0) = translation(i);
   result.setTranslation(translation2);
   return result;
}


int main(int argc, char *argv[])
{

   std::string meshname = "";
   bool noViewer = true;
   bool bufferImages = false;
   bool useColor = true;
   bool volumeColor = true;


   bool useLoopClosures = false;

   unsigned int startimage = 0;
   unsigned int endimage = 1000000;
   unsigned int imageStep = 1;

   float maxCamDistance = MAXCAMDISTANCE;
   float scale = DEFAULT_SCALE;
   float threshold = DEFAULT_SCALE;

   float imageDepthScale = DEPTHSCALE;

   std::string intrinsics = "";

   bool threadMeshing = true;
   bool threadFusion = false;
   bool threadImageReading = false;
   bool performIncrementalMeshing = true;

   int depthConstistencyChecks = 0;

   TCLAP::CmdLine cmdLine("onlinefusion");

   TCLAP::ValueArg<std::string> loadMeshArg("l","loadmesh","Loads this mesh",false,meshname,"string");
   TCLAP::SwitchArg threadMeshingArg("","thread-meshing","Thread the Meshing inside the Fusion",threadMeshing);
   TCLAP::SwitchArg threadFusionArg("","thread-fusion","Thread the Fusion inside the Viewer",threadFusion);
   TCLAP::SwitchArg threadImageReadingArg("","thread-image","Thread reading the Images from Hard Disk",threadImageReading);
   TCLAP::SwitchArg viewerArg("v","viewer","Show a Viewer after Fusion",!noViewer);
   TCLAP::SwitchArg bufferArg("b","buffer","Buffer all Images",bufferImages);
   TCLAP::SwitchArg useLoopClosuresArg("c","loopclosures","Read Multiple Trajectories and perform Loop Closures",useLoopClosures);
   TCLAP::SwitchArg performIncrementalMeshingArg("","incremental-meshing","Perform incremental Meshing",performIncrementalMeshing);
   TCLAP::ValueArg<int> startimageArg("s","startimage","Number of the Start Image",false,startimage,"int");
   TCLAP::ValueArg<int> endimageArg("e","endimage","Number of the End Image",false,endimage,"int");
   TCLAP::ValueArg<int> imageStepArg("k","imagestep","Use every kth step",false,imageStep,"int");
   TCLAP::ValueArg<int> depthConsistencyChecksArg("","consistency-checks","Number of Depth Consistency Checks",false,depthConstistencyChecks,"int");
   TCLAP::ValueArg<float> maxCamDistanceArg("","max-camera-distance","Maximum Camera Distance to Surface",false,maxCamDistance,"float");
   TCLAP::ValueArg<float> scaleArg("","scale","Size of the Voxel",false,scale,"float");
   TCLAP::ValueArg<float> thresholdArg("","threshold","Threshold",false,threshold,"float");
   TCLAP::ValueArg<float> imageDepthScaleArg("","imagescale","Image Depth Scale",false,imageDepthScale,"float");
   TCLAP::ValueArg<std::string> intrinsicsArg("","intrinsics","File with Camera Matrix",false,intrinsics,"string");

   TCLAP::UnlabeledMultiArg<std::string> associationfilenamesArg("filenames", "The File Names",false,"string");


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
   cmdLine.add(intrinsicsArg);

   cmdLine.add(associationfilenamesArg);
   cmdLine.parse(argc,argv);

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
   if(imageStep < 1) imageStep = 1;
   depthConstistencyChecks = depthConsistencyChecksArg.getValue();
   maxCamDistance = maxCamDistanceArg.getValue();
   scale = scaleArg.getValue();
   threshold = thresholdArg.getValue();
   imageDepthScale = imageDepthScaleArg.getValue();
   intrinsics = intrinsicsArg.getValue();

   float fx = 525.0f;
   float fy = 525.0f;
   float cx = 319.5f;
   float cy = 239.5f;

   //Intrinsics
   if (intrinsics != "") {
      fprintf(stderr,"\nReading intrinsic camera parameters from %s\n",intrinsics.c_str());
      std::fstream intrinsicsfile;
      intrinsicsfile.open(intrinsics.c_str(),std::ios::in);
      if(!intrinsicsfile.is_open()){
         fprintf(stderr,"\nERROR: Could not open File %s for reading!",intrinsics.c_str());
      } else {
         float temp;
         fx = fy = cx = cy = -1.0f;
         intrinsicsfile >> fx;
         intrinsicsfile >> temp;
         intrinsicsfile >> cx;
         intrinsicsfile >> temp;
         intrinsicsfile >> fy;
         intrinsicsfile >> cy;
         intrinsicsfile.close();
         fprintf(stderr,"\nCamera Intrinsics from File: %f %f %f %f",fx,fy,cx,cy);
      }
   } else {
      fprintf(stderr, "\nNo Camera Intrinsics found\n");
   }

   if(threadMeshing) fprintf(stderr,"\nMeshing will run in a separate Thread");
   else              fprintf(stderr,"\nMeshing will NOT run in a separate Thread");
   if(threadFusion)  fprintf(stderr,"\nFusion will run in a separate Thread");
   if(threadImageReading) fprintf(stderr,"\nImage Reading will run in a separate Thread");

   CameraInfo startpos;

   std::string defaultname = "../test/rgbd_dataset_freiburg3_long_office_household/associationVICOM.txt";

   std::string tempfileprefix = "debug/";

   std::vector<std::string> associationfilenames = associationfilenamesArg.getValue();
   std::vector<std::string> prefices;

   if(!associationfilenames.size()){
      associationfilenames.push_back(defaultname);
   }


   for(unsigned int i=0;i<associationfilenames.size();i++){
      prefices.push_back(associationfilenames[i].substr(0,associationfilenames[i].find_last_of('/')+1));
   }



   std::vector<std::pair<Eigen::Matrix3d,Eigen::Vector3d> > poses_from_assfile;
   std::vector<std::vector<std::string> > depthNames;
   std::vector<std::vector<std::string> > rgbNames;
   std::vector<std::vector<CameraInfo> > poses;

   if(!useLoopClosures){
      fprintf(stderr,"\nBuilding a single Trajectory...");
      poses.push_back(std::vector<CameraInfo>());
      depthNames.push_back(std::vector<std::string>());
      rgbNames.push_back(std::vector<std::string>());
      std::vector<CameraInfo> &trajectory = poses.back();
      std::vector<std::string> &depthNamesLast = depthNames.back();
      std::vector<std::string> &rgbNamesLast = rgbNames.back();
      unsigned int assfilestartindex = 0;
      for(unsigned int f=0;f<associationfilenames.size();f++){
         std::string prefix = prefices[f];
         std::fstream associationfile; float junkstamp;
         std::string depthname; std::string rgbname;
         float q1, q2, q3, q4, translation1, translation2, translation3;
         associationfile.open(associationfilenames[f].c_str(),std::ios::in);
         if(!associationfile.is_open()){
            fprintf(stderr,"\nERROR: Could not open File %s",associationfilenames[f].c_str());
         }else{
            fprintf(stderr,"\nReading Association File");
            while(!associationfile.eof()){
               std::string temp("");
               getline(associationfile,temp);
               std::stringstream stream(temp);
               stream >> junkstamp;
               stream >> translation1; stream >> translation2; stream >> translation3;
               stream >> q1; stream >> q2; stream >> q3; stream >> q4;
               stream >> junkstamp;
               stream >> depthname;
               if(temp!=""){
                  Eigen::Matrix<double, 3, 3> rotEigen;
                  Eigen::Quaterniond quatEigen(q4,q1,q2,q3);
                  Eigen::Vector3d transEigen(translation1,translation2,translation3);
                  rotEigen = quatEigen.toRotationMatrix();
                  //poses_from_assfile.push_back(std::pair<Eigen::Matrix3d,Eigen::Vector3d>(Eigen::Quaterniond(q4,q1,q2,q3).toRotationMatrix(),Eigen::Vector3d(translation1,translation2,translation3)));
                  poses_from_assfile.push_back(std::pair<Eigen::Matrix3d,Eigen::Vector3d>(rotEigen,transEigen));
                  depthNamesLast.push_back(depthname);
                  if(useColor){
                     stream >> junkstamp; stream >> rgbname; rgbNamesLast.push_back(rgbname);
                  }
               }
            }
            fprintf(stderr,"\nRead %i Poses from Association File Nr %i : %s .",
                  (int)depthNamesLast.size()-assfilestartindex,f,associationfilenames[f].c_str());
         }

         for(unsigned int i=assfilestartindex;i<poses_from_assfile.size();i++){
            trajectory.push_back(kinectPoseFromEigen(poses_from_assfile[i],fx,fy,cx,cy));
            //trajectory.back().setExtrinsic(startpos.getExtrinsic()*trajectory.back().getExtrinsic());
            depthNamesLast[i] = prefix + depthNamesLast[i];
            if(useColor) rgbNamesLast[i] = prefix + rgbNamesLast[i];
         }
         startpos = trajectory.back();
         assfilestartindex = depthNamesLast.size();
         poses_from_assfile.clear();
      }
   }
   else{
      fprintf(stderr,"\nUnsuported Trajectories");
      return -1;
   }

   if(!useLoopClosures){
      fprintf(stderr,"\nRead %i Poses und Depth Images and %i RGB Images from %i Association Files",
            (int)depthNames.size(), (int)rgbNames.size(), (int)associationfilenames.size());
   }
   else{
      fprintf(stderr,"\nUnsuported Trajectories");
      return -1;
   }


   if(startimage >= depthNames.front().size()) startimage = depthNames.front().size()-1;
   if(endimage >= depthNames.back().size()) endimage = depthNames.back().size()-1;

   fprintf(stderr,"\nCreating Mipmapping CPU Octree");
   FusionMipMapCPU *fusion = new FusionMipMapCPU(0,0,0,scale,threshold,0,volumeColor);

   fusion->setThreadMeshing(threadMeshing);
   fusion->setDepthChecks(depthConstistencyChecks);
   fusion->setIncrementalMeshing(performIncrementalMeshing);

   fprintf(stderr,"\nCreating Viewer");
   QApplication application(argc,argv);

   OnlineFusionViewerManipulated *viewerpointer = new OnlineFusionViewerManipulated(false);
   OnlineFusionViewerManipulated &viewer = *viewerpointer;

   fprintf(stderr,"\nSetting Viewer Parameters");
   viewer._fusion = fusion;
   viewer.setWindowTitle("Fusion Volume");
   viewer._poses = poses;
   viewer._depthNames = depthNames;
   viewer._rgbNames = rgbNames;
   viewer._threadFusion = threadFusion;
   viewer._threadImageReading = threadImageReading;
   viewer.show();
   viewer._imageDepthScale = imageDepthScale;
   viewer._maxCamDistance = maxCamDistance;
   viewer._firstFrame = (long int)startimage;
   viewer._currentFrame = (long int)startimage-1;
   fprintf(stderr,"\nSet Viewer Frame to %li",(long int)viewer._currentFrame);
   viewer._nextStopFrame = endimage;
   fprintf(stderr,"\nStarting Qt-Application");
   application.exec();

   fprintf(stderr,"\nDeleting Viewer");
   delete viewerpointer;

   fprintf(stderr,"\nProgram %s exiting.\n\n",argv[0]);
   fprintf(stderr,"\nPress Enter exit the Program");
   char input[256];
   fprintf(stderr,"%s",fgets(input,256,stdin));
   return 0;
}

