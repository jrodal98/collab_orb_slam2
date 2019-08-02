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

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

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
cv::Mat M1l, M2l;
cv::Mat M1r, M2r;
float mBaseline;
float mFocalLength;

void ExtractORB(int flag, const cv::Mat &im, std::vector<cv::KeyPoint> &vKeys, cv::Mat &descriptors)
{
	if (flag == 0)
	{
		(*mpORBextractorLeft)(im, cv::Mat(), vKeys, descriptors);
	}
	else
	{
		(*mpORBextractorRight)(im, cv::Mat(), vKeys, descriptors);
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

	K0 = cv::Mat::eye(3, 3, CV_32F);
	K0.at<float>(0, 0) = fx;
	K0.at<float>(1, 1) = fy;
	K0.at<float>(0, 2) = cx;
	K0.at<float>(1, 2) = cy;

	DistCoef0 = cv::Mat(4, 1, CV_32F);
	DistCoef0.at<float>(0) = fsSettings["Camera.k1"];
	DistCoef0.at<float>(1) = fsSettings["Camera.k2"];
	DistCoef0.at<float>(2) = fsSettings["Camera.p1"];
	DistCoef0.at<float>(3) = fsSettings["Camera.p2"];
	const float k3 = fsSettings["Camera.k3"];
	if (k3 != 0)
	{
		DistCoef0.resize(5);
		DistCoef0.at<float>(4) = k3;
	}

	float bf = fsSettings["Camera.bf"];
	mBaseline = bf / fx;
	mFocalLength = fx;

	cerr << endl
		 << "Camera Parameters: " << endl;
	cerr << "- fx: " << fx << endl;
	cerr << "- fy: " << fy << endl;
	cerr << "- cx: " << cx << endl;
	cerr << "- cy: " << cy << endl;
	cerr << "- k1: " << DistCoef0.at<float>(0) << endl;
	cerr << "- k2: " << DistCoef0.at<float>(1) << endl;
	if (DistCoef0.rows == 5)
		cerr << "- k3: " << DistCoef0.at<float>(4) << endl;
	cerr << "- p1: " << DistCoef0.at<float>(2) << endl;
	cerr << "- p2: " << DistCoef0.at<float>(3) << endl;

	// Load ORB parameters
	nFeatures = fsSettings["ORBextractor.nFeatures"];
	fScaleFactor = fsSettings["ORBextractor.scaleFactor"];
	nLevels = fsSettings["ORBextractor.nLevels"];
	fIniThFAST = fsSettings["ORBextractor.iniThFAST"];
	fMinThFAST = fsSettings["ORBextractor.minThFAST"];

	cerr << endl
		 << "ORB Extractor Parameters: " << endl;
	cerr << "- Number of Features: " << nFeatures << endl;
	cerr << "- Scale Levels: " << nLevels << endl;
	cerr << "- Scale Factor: " << fScaleFactor << endl;
	cerr << "- Initial Fast Threshold: " << fIniThFAST << endl;
	cerr << "- Minimum Fast Threshold: " << fMinThFAST << endl;
}

int main(int argc, char **argv)
{
	std::cerr << "Loading Things" << std::endl;
	po::options_description desc("Allowed options");
	desc.add_options()("help", "produce help message")("voc,v", po::value<std::string>(), "Vocabulary path")("input,i", po::value<std::string>(), "Image path")("coding,c", po::value<std::string>(), "settings path")("settings,s", po::value<std::string>(), "ORB SLAM settings path")("robotid,r", po::value<int>(), "agent id");

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);
	std::cerr << "Loading Settings" << std::endl;

	// Load settings
	std::string strSettingPath = vm["settings"].as<std::string>();
	std::cerr << strSettingPath << std::endl;
	loadSettings(strSettingPath);

	// Load vocabulary
	std::string voc_path = vm["voc"].as<std::string>();
	ORBVocabulary voc;
	std::cerr << "Loading vocabulary from " << voc_path << std::endl;
	voc.loadFromTextFile(voc_path);

	// Load coding statistics
	std::string settings_path = vm["coding"].as<std::string>();
	std::cerr << "Loading statistics from " << settings_path << std::endl;
	LBFC2::CodingStats codingModel;
	codingModel.load(settings_path);

	// Load images
	std::string image_path = vm["input"].as<std::string>();
	std::cerr << "Loading Video from " << image_path << std::endl;

	bool success = false;
	int frameskips = 1;
	size_t nImages = 0;
	cv::VideoCapture cap(image_path);

	// get dimensions of image from first image
	int imgWidth;
	int imgHeight;
	while (!success)
	{
		cv::Mat frame;
		success = cap.read(frame);
		if (success)
		{
			imgWidth = frame.size().width;
			imgHeight = frame.size().height;
		}
	}

	int bufferSize = 1;
	bool inter = true;
	bool stereo = false;
	bool depth = false;
	LBFC2::FeatureCoder encoder(voc, codingModel, imgWidth, imgHeight, nLevels, 32, bufferSize, inter, stereo, depth, mFocalLength, mBaseline);

	// Setup features
	mpORBextractorLeft = std::shared_ptr<CORB_SLAM2::ORBextractor>(new CORB_SLAM2::ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST));
	mpORBextractorRight = std::shared_ptr<CORB_SLAM2::ORBextractor>(new CORB_SLAM2::ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST));

	// Setup ROS
	int nRobotId = vm["robotid"].as<int>();
	std::cerr << "Start" << std::endl;
	//process each frame
	while (success)
	{
		if (nImages % 64 == 0)
		{
			std::cerr << "Loading Image " << nImages << std::endl;
		}
		cv::Mat frame;
		success = cap.read(frame);
		if (success)
		{
			nImages++;
			if (nImages % frameskips == 0)
			{
				std::vector<cv::KeyPoint> keypointsLeft;
				cv::Mat descriptorsLeft;
				cv::Mat gray_img;
				cv::cvtColor(frame, gray_img, cv::COLOR_RGB2GRAY);
				std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
				ExtractORB(0, gray_img, std::ref(keypointsLeft), std::ref(descriptorsLeft));

				std::vector<uchar> bitstream;
				encoder.encodeImage(keypointsLeft, descriptorsLeft, bitstream);

				//double tframe = vTimestamps[imgId];

				compression::msg_features msg;
				msg.nrobotid = nRobotId;
				cv::imencode(".png", frame, msg.img);
				msg.data.assign(bitstream.begin(), bitstream.end());
				uint64_t n = bitstream.size();
				std::cout.write(static_cast<char*>(static_cast<void*>(&n)), sizeof(n));
				std::cout.write(static_cast<char*>(static_cast<void*>(bitstream.data())), n);
				n = msg.img.size();
				std::cout.write(static_cast<char*>(static_cast<void*>(&n)), sizeof(n));
				std::cout.write(static_cast<char*>(static_cast<void*>(msg.img.data())), n);

				std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();

				double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
				usleep((1) * 1e6);
			}
		}
	}
	std::cerr << "Done Processing Video" << std::endl;
}
