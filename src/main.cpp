/**
* Modified version of original DSO ROS wrapper to make it work for LDSO
* 
* This file is part of DSO.
* 
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/





#include <locale.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

//#include "util/settings.h"
#include "frontend/FullSystem.h"
#include "frontend/Undistort.h"
#include "frontend/DSOViewer.h"
//#include "IOWrapper/OutputWrapper/SampleOutputWrapper.h"


#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include "cv_bridge/cv_bridge.h"
#include <glog/logging.h>
#include "IOWrapper/Pangolin/PangolinDSOViewer.h"
//#include "IOWrapper/OutputWrapper/SampleOutputWrapper.h"



std::string calib = "";
std::string vignetteFile = "";
std::string gammaFile = "";
std::string vocPath = "../../LDSO/vocab/orbvoc.dbow3";

bool useSampleOutput=false;

using namespace ldso;

void parseArgument(char* arg)
{
	
	char buf[1000];
	if(1==sscanf(arg,"calib=%s",buf))
	{
		calib = buf;
		printf("loading calibration from %s!\n", calib.c_str());
		return;
	}
	if(1==sscanf(arg,"vignette=%s",buf))
	{
		vignetteFile = buf;
		printf("loading vignette from %s!\n", vignetteFile.c_str());
		return;
	}

	  if (1 == sscanf(arg, "vocab=%s", buf)) {
        vocPath = buf;
        printf("loading vocabulary from %s!\n", vocPath.c_str());
        return;
    }

	if(1==sscanf(arg,"gamma=%s",buf))
	{
		gammaFile = buf;
		printf("loading gammaCalib from %s!\n", gammaFile.c_str());
		return;
	}

	printf("could not parse argument \"%s\"!!\n", arg);
}

ros::Publisher posePub;
FullSystem* fullSystem = 0;
Undistort* undistorter = 0;
int frameID = 0;

void vidCb(const sensor_msgs::ImageConstPtr img)
{
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
	assert(cv_ptr->image.type() == CV_8U);
	assert(cv_ptr->image.channels() == 1);
	
	MinimalImageB minImg((int)cv_ptr->image.cols, (int)cv_ptr->image.rows,(unsigned char*)cv_ptr->image.data);
	ImageAndExposure* undistImg = undistorter->undistort<unsigned char>(&minImg, 1,0, 1.0f);
	undistImg->timestamp = cv_ptr->header.stamp.toSec();
	fullSystem->addActiveFrame(undistImg, frameID);
	frameID++;
	delete undistImg;

	// get pose
	std::vector<double> pose;
	pose = fullSystem->getResult();

	// publish odom
	nav_msgs::Odometry odom;
	odom.header.stamp =  ros::Time::now();
	odom.pose.pose.position.x = pose[1];
	odom.pose.pose.position.y = pose[2];
	odom.pose.pose.position.z = pose[3];
	odom.pose.pose.orientation.x = pose[4];
	odom.pose.pose.orientation.y = pose[5];
	odom.pose.pose.orientation.z = pose[6];
	odom.pose.pose.orientation.z = pose[7];
	posePub.publish(odom);

}


int main( int argc, char** argv )
{
	ros::init(argc, argv, "ldso_live");

	for(int i=1; i<argc;i++) parseArgument(argv[i]);


	setting_desiredImmatureDensity = 1000;
	setting_desiredPointDensity = 1200;
	setting_minFrames = 5;
	setting_maxFrames = 7;
	setting_maxOptIterations=4;
	setting_minOptIterations=1;
	setting_logStuff = false;
	setting_kfGlobalWeight = 1.3;


	printf("MODE WITH CALIBRATION, but without exposure times!\n");
	setting_photometricCalibration = 2;
	setting_affineOptModeA = 0;
	setting_affineOptModeB = 0;



    undistorter = Undistort::getUndistorterForFile(calib, gammaFile, vignetteFile);

    setGlobalCalib(
            (int)undistorter->getSize()[0],
            (int)undistorter->getSize()[1],
            undistorter->getK().cast<float>());

	shared_ptr<ORBVocabulary> voc(new ORBVocabulary());
    voc->load(vocPath);
    fullSystem = new FullSystem(voc);
    fullSystem->linearizeOperation=false;


    if(undistorter->photometricUndist != 0)
    	fullSystem->setGammaFunction(undistorter->photometricUndist->getG());

    ros::NodeHandle nh;
    ros::Subscriber imgSub = nh.subscribe("image", 1, &vidCb);

	ros::NodeHandle r;
	posePub = r.advertise<nav_msgs::Odometry>("dso_odom", 10);

    ros::spin();

    delete undistorter;
    delete fullSystem;

	return 0;
}

