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

#include <signal.h>

#include "boost/program_options.hpp"

#include "compression/msg_features.h"

#include "System.h"
#include "feature_coder.h"

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "agent_brain.pb.h"

namespace po = boost::program_options;

// Setup decoder
ORBVocabulary voc;
LBFC2::CodingStats codingModel;
CORB_SLAM2::System *SLAM;

// Adapt to agent
int bufferSize = 1;
bool inter = true;
bool stereo = false;
bool depth = false;

int nlevels = 8;
//int imgWidth = 1241;
//int imgHeight = 376;

//int imgWidth = 1280;
//int imgHeight = 720;

int imgWidth;
int imgHeight;

std::map<int, std::mutex> mutexPool;
std::map<int, std::thread> mThreadMap;

bool bUseViewer = false;
string strSettingsFile1;
string strSettingsFile2;

void signal_handler(int signal)
{
	//Save trajectory / stats etc.
	std::cerr << "Shutting down" << std::endl;
	SLAM->Shutdown();

	std::cerr << "Exit." << std::endl;
	exit(signal);
}

void trackMono(CORB_SLAM2::System *SLAM, const CORB_SLAM2::FrameInfo &info, const std::vector<cv::KeyPoint> &keyPointsLeft,
			   const cv::Mat &descriptorLeft, const std::vector<unsigned int> &visualWords,
			   const std::vector<cv::KeyPoint> &keyPointsRight, const cv::Mat &descriptorRight,
			   const double &timestamp, int nAgentId, const cv::Mat &img)
{
	//SLAM->TrackMonoCompressed(info, keyPointsLeft, descriptorLeft, visualWords, keyPointsRight, descriptorRight, timestamp, nAgentId);
	SLAM->TrackStereoCompressed(info, keyPointsLeft, descriptorLeft, visualWords, keyPointsRight, descriptorRight, timestamp, nAgentId, img);
}

bool read_data(int fd, std::vector<uchar> &data, std::vector<uchar> &img)
{
	uint64_t size;
	if (read(fd, &size, sizeof(uint64_t)) < 0)
	{
		perror("Error reading encoded features buffer size from fifo pipe");
		exit(0);
	}
	if (!size)
		return false;
	data.resize(size);
	if (read(fd, &data[0], size) < 0)
	{
		perror("Error reading encoded features buffer size from fifo pipe");
		exit(0);
	}
	if (read(fd, &size, sizeof(uint64_t)) < 0)
	{
		perror("Error reading encoded features buffer size from fifo pipe");
		exit(0);
	}
	img.resize(size);
	if (read(fd, &img[0], size) < 0)
	{
		perror("Error reading encoded features buffer size from fifo pipe");
		exit(0);
	}

	return true;
}

void track(int robot_id, std::vector<uchar> &data, std::vector<uchar> &img_vec, CORB_SLAM2::System *SLAM, LBFC2::FeatureCoder *coder)
{
	cv::Mat img = cv::imdecode(img_vec, 1);
	std::vector<unsigned int> vDecVisualWords;
	std::vector<cv::KeyPoint> vDecKeypointsLeft, vDecKeypointsRight;
	cv::Mat decDescriptorsLeft, decDescriptorsRight;

	// Get frame info
	CORB_SLAM2::FrameInfo info;
	info.mnHeight = imgHeight;
	info.mnWidth = imgWidth;

	int nAgentId = robot_id;
	coder->decodeImageStereo(data, vDecKeypointsLeft, decDescriptorsLeft, vDecKeypointsRight, decDescriptorsRight, vDecVisualWords);
	const double tframe = 0.0; // the timeframe seems unnecessary right now, will have to actually do something about this if I'm incorrect
	trackMono(SLAM, info, vDecKeypointsLeft, decDescriptorsLeft, vDecVisualWords,
			  vDecKeypointsRight, decDescriptorsRight, tframe, nAgentId, img);
}

void handle_agent(int robot_id)
{
	std::cerr << "Handling robot " << robot_id << std::endl;
	SLAM->InitAgent(robot_id, (robot_id == 0) ? strSettingsFile1 : strSettingsFile2, CORB_SLAM2::Sensor::STEREO, bUseViewer);
	std::string myfifo = "/tmp/outpipe" + std::to_string(robot_id);
	int fd = open(myfifo.c_str(), O_RDONLY);
	LBFC2::FeatureCoder *coder = new LBFC2::FeatureCoder(voc, codingModel, imgWidth, imgHeight, nlevels, 32, bufferSize, inter, stereo, depth);
	std::vector<uchar> data;
	std::vector<uchar> img;
	while (1)
	{
		if (!read_data(fd, data, img))
			break;
		track(robot_id, data, img, SLAM, coder);
		data.clear();
		img.clear();
	}
	close(fd);
}

int main(int argc, char **argv)
{
	po::options_description desc("Allowed options");
	desc.add_options()("help", "produce help message")("voc,v", po::value<std::string>(), "Vocabulary path")("coding,c", po::value<std::string>(), "coding model")("settings,s", po::value<std::string>(), "settings base path");

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);

	// Load vocabulary
	std::string voc_path = vm["voc"].as<std::string>();
	std::cerr << "Loading vocabulary from " << voc_path << std::endl;
	voc.loadFromTextFile(voc_path);

	// Load coding model
	std::string stats_path = vm["coding"].as<std::string>();
	std::cerr << "Loading statistics from " << stats_path << std::endl;
	codingModel.load(stats_path);

	// Setup ORB SLAM - make sure to use the correct settings file
	std::string settings_path = vm["settings"].as<std::string>();
	//const string &strSettingsFile1 = settings_path + "/KITTI00-02.yaml";
	//const string &strSettingsFile2 = settings_path + "/KITTI04-12.yaml";
	strSettingsFile1 = settings_path + "/statue.yaml";
	strSettingsFile2 = settings_path + "/statue.yaml";
	cv::FileStorage fsSettings(strSettingsFile1, cv::FileStorage::READ);
	imgHeight = fsSettings["Camera.height"];
	imgWidth = fsSettings["Camera.width"];

	SLAM = new CORB_SLAM2::System(voc_path);

	signal(SIGINT, signal_handler);

	std::map<int, LBFC2::FeatureCoder *> coderMap;
	std::map<int, std::thread> threadPool;

	int robot_id = 0;
	std::vector<std::thread> agents;
	std::string input;
	std::cin >> input;
	while (input != "QUIT")
	{
		agents.push_back(std::thread(handle_agent, robot_id++));
		std::cin >> input;
	}

	for (int i = 0; i < robot_id; i++)
	{
		agents[i].join();
	}
	// Lets spin
	std::cerr << "Finished" << std::endl;

	signal_handler(0);

	return 0;
}
