/**
* This file is part of the Collaborative Visual SLAM Extension for ORB-SLAM2:
* "Collaborative Visual SLAM using Compressed Feature Exchange"
* Copyright (C) 2017-2018 Dominik Van Opdenbosch <dominik dot van-opdenbosch at tum dot de>
* Chair of Media Technology, Technical University of Munich
* For more information see <https://d-vo.github.io/>
*
* ORB-SLAM2 and the collaborative extension is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 and the collaborative extension is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with the ORB-SLAM2 collaborative extension. If not, see <http://www.gnu.org/licenses/>.
*
* The extension is built upon ORB-SLAM2:
* ORB-SLAM2 Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*/

#include "boost/program_options.hpp"

#include "ros/ros.h"
#include "compression/msg_features.h"

#include "System.h"
#include "feature_coder.h"



namespace po = boost::program_options;

std::shared_ptr<CORB_SLAM2::ORBextractor> mpORBextractorLeft;
std::shared_ptr<CORB_SLAM2::ORBextractor> mpORBextractorRight;

int nFeatures;
float fScaleFactor;
int nLevels;
int fIniThFAST;
int fMinThFAST;
cv::Mat K0;
cv::Mat DistCoef0;
cv::Mat M1l,M2l;
cv::Mat M1r,M2r;
float mBaseline;
float mFocalLength;


void ExtractORB(int flag, const cv::Mat &im, std::vector<cv::KeyPoint> &vKeys, cv::Mat &descriptors)
{
    if(flag==0){
        (*mpORBextractorLeft)(im,cv::Mat(),vKeys,descriptors);
    } else {
        (*mpORBextractorRight)(im,cv::Mat(),vKeys,descriptors);
    }
}

void loadSettings(const std::string &settingsFile)
{
	// Load camera parameters from settings file
	cv::FileStorage fsSettings(settingsFile, cv::FileStorage::READ);
	float fx = fsSettings["Camera.fx"];
	float fy = fsSettings["Camera.fy"];
	float cx = fsSettings["Camera.cx"];
	float cy = fsSettings["Camera.cy"];

	K0 = cv::Mat::eye(3,3,CV_32F);
	K0.at<float>(0,0) = fx;
	K0.at<float>(1,1) = fy;
	K0.at<float>(0,2) = cx;
	K0.at<float>(1,2) = cy;

	DistCoef0 = cv::Mat(4,1,CV_32F);
	DistCoef0.at<float>(0) = fsSettings["Camera.k1"];
	DistCoef0.at<float>(1) = fsSettings["Camera.k2"];
	DistCoef0.at<float>(2) = fsSettings["Camera.p1"];
	DistCoef0.at<float>(3) = fsSettings["Camera.p2"];
	const float k3 = fsSettings["Camera.k3"];
	if(k3!=0)
	{
		DistCoef0.resize(5);
		DistCoef0.at<float>(4) = k3;
	}

	float bf = fsSettings["Camera.bf"];
	mBaseline = bf / fx;
	mFocalLength = fx;

	cout << endl << "Camera Parameters: " << endl;
	cout << "- fx: " << fx << endl;
	cout << "- fy: " << fy << endl;
	cout << "- cx: " << cx << endl;
	cout << "- cy: " << cy << endl;
	cout << "- k1: " << DistCoef0.at<float>(0) << endl;
	cout << "- k2: " << DistCoef0.at<float>(1) << endl;
	if(DistCoef0.rows==5)
		cout << "- k3: " << DistCoef0.at<float>(4) << endl;
	cout << "- p1: " << DistCoef0.at<float>(2) << endl;
	cout << "- p2: " << DistCoef0.at<float>(3) << endl;


	// Load ORB parameters
	nFeatures = fsSettings["ORBextractor.nFeatures"];
	fScaleFactor = fsSettings["ORBextractor.scaleFactor"];
	nLevels = fsSettings["ORBextractor.nLevels"];
	fIniThFAST = fsSettings["ORBextractor.iniThFAST"];
	fMinThFAST = fsSettings["ORBextractor.minThFAST"];


	cout << endl  << "ORB Extractor Parameters: " << endl;
	cout << "- Number of Features: " << nFeatures << endl;
	cout << "- Scale Levels: " << nLevels << endl;
	cout << "- Scale Factor: " << fScaleFactor << endl;
	cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
	cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;
}


int main(int argc, char **argv)
{
	std::cout << "Loading Things" << std::endl;
	po::options_description desc("Allowed options");
	desc.add_options()
						("help", "produce help message")
						("voc,v", po::value<std::string>(), "Vocabulary path")
						("input,i", po::value<std::string>(), "Image path")
						("coding,c", po::value<std::string>(), "settings path")
						("settings,s", po::value<std::string>(), "ORB SLAM settings path")
						("robotid,r", po::value<int>(), "agent id");


	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);
	std::cout << "Loading Settings" << std::endl;

	// Load settings
	std::string strSettingPath = vm["settings"].as<std::string>();
	std::cout << strSettingPath << std::endl;
	loadSettings(strSettingPath);


	// Load vocabulary
	std::string voc_path = vm["voc"].as<std::string>();
	ORBVocabulary voc;
	std::cout << "Loading vocabulary from " << voc_path << std::endl;
	voc.loadFromTextFile(voc_path);


	// Load coding statistics
	std::string settings_path = vm["coding"].as<std::string>();
	std::cout << "Loading statistics from " << settings_path << std::endl;
	LBFC2::CodingStats codingModel;
	codingModel.load(settings_path );


	// Load images
	std::string image_path = vm["input"].as<std::string>();
	std::cout << "Loading Video from " << image_path << std::endl;

	bool success = false;
	int frameskips = 5;
	size_t nImages = 0;
	cv::VideoCapture cap(image_path);

	// get dimensions of image from first image
	int imgWidth;
	int imgHeight;
	while(!success){
		cv::Mat frame;
		success = cap.read(frame);
		if(success){
			imgWidth = vImgLeft[0].size().width;
			imgHeight = vImgLeft[0].size().height;
		}
	}

	int bufferSize = 1;
	bool inter = true;
	bool stereo = false;
	bool depth = false;
	LBFC2::FeatureCoder encoder(voc, codingModel,imgWidth, imgHeight, nLevels, 32, bufferSize, inter, stereo, depth, mFocalLength, mBaseline);

	// Setup features
    mpORBextractorLeft = std::shared_ptr<CORB_SLAM2::ORBextractor>(new CORB_SLAM2::ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST));
    mpORBextractorRight = std::shared_ptr<CORB_SLAM2::ORBextractor>(new CORB_SLAM2::ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST));

    // Setup ROS
    int nRobotId = vm["robotid"].as<int>();
  	std::string bitstreamTopic = "/featComp/bitstream" + std::to_string(nRobotId);
  	std::string name = "agent" + std::to_string(nRobotId);
  	ros::init(argc, argv, name.c_str());
  	ros::NodeHandle n;
  	ros::Publisher bitstream_pub = n.advertise<compression::msg_features>(bitstreamTopic, 1000, true);
    ros::Rate poll_rate(500);
	
	while(bitstream_pub.getNumSubscribers() == 0){
		poll_rate.sleep();
		std::cout << "loop" << std::endl;
	}

	std::cout << "Start" << std::endl;

	//process each frame
	while(success){
		if(nImages % 64 == 0){
			std::cout << "Loading Image " << nImages << std::endl;
		}
		cv::Mat frame;
		success = cap.read(frame);
		if(success){
			nImages++;
			if(nImages % frameskips == 0){
				std::vector<cv::KeyPoint> keypointsLeft;
				cv::Mat descriptorsLeft;
				std::thread threadLeft(ExtractORB,0,frame, std::ref(keypointsLeft), std::ref(descriptorsLeft));
				threadLeft.join();

				std::vector<uchar> bitstream;
				encoder.encodeImage(keypointsLeft, descriptorsLeft, bitstream);
				double tframe = nImages/frameskips;
				compression::msg_features msg;

				msg.header.stamp = ros::Time::now();
				msg.tframe = tframe;
				msg.nrobotid = nRobotId;
				msg.data.assign(bitstream.begin(),bitstream.end());
				bitstream_pub.publish(msg);

				ros::spinOnce();
				usleep((1)*1e6);
			}
		} 
	}
	std::cout << "Done Processing Video" << std::endl;
}
