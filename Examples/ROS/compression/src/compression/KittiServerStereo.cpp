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

// Setup decoder
ORBVocabulary voc;
LBFC2::CodingStats codingModel;
CORB_SLAM2::System *SLAM;
bool bUseViewer = false;
string strSettingsFile1;
string strSettingsFile2;
// ros::AsyncSpinner *spinner;

// Adapt to agent
int bufferSize = 1;
bool inter = true;
bool stereo = false;
bool depth = false;

int nlevels = 8;
int imgWidth = 1241;
int imgHeight = 376;

std::map<int, std::mutex> mutexPool;
std::map<int, std::thread> mThreadMap;

void signal_handler(int signal)
{
	//Save trajectory / stats etc.
	std::cerr << "Shutting down" << std::endl;
	SLAM->Shutdown();

	std::cerr << "Exit." << std::endl;
	exit(signal);
}

void trackStereo(CORB_SLAM2::System *SLAM, const CORB_SLAM2::FrameInfo &info, const std::vector<cv::KeyPoint> &keyPointsLeft,
				 const cv::Mat &descriptorLeft, const std::vector<unsigned int> &visualWords,
				 const std::vector<cv::KeyPoint> &keyPointsRight, const cv::Mat &descriptorRight,
				 const double &timestamp, int nAgentId, const cv::Mat &img)
{
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
	trackStereo(SLAM, info, vDecKeypointsLeft, decDescriptorsLeft, vDecVisualWords,
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

void callback(const compression::msg_features::ConstPtr msg, CORB_SLAM2::System *SLAM, std::map<int, LBFC2::FeatureCoder *> *coderMap)
{
	// Convert bitstream
	std::vector<uchar> img_bitstream(msg->data.begin(), msg->data.end());
	std::vector<uchar> img_vec(msg->img.begin(), msg->img.end());
	// cv::Mat data_mat(img_vec, true);
	cv::Mat img = cv::imdecode(img_vec, 1);
	// Setup visual variables
	std::vector<unsigned int> vDecVisualWords;
	std::vector<cv::KeyPoint> vDecKeypointsLeft, vDecKeypointsRight;
	cv::Mat decDescriptorsLeft, decDescriptorsRight;

	// Get frame info
	CORB_SLAM2::FrameInfo info;
	info.mnHeight = imgHeight;
	info.mnWidth = imgWidth;

	// Lock decoder for the corresponding agent id
	int nAgentId = msg->nrobotid;
	std::unique_lock<std::mutex> lock(mutexPool[nAgentId]);

	// Check if decoding instance is available
	if (coderMap->find(nAgentId) == coderMap->end())
		(*coderMap)[nAgentId] = new LBFC2::FeatureCoder(voc, codingModel, imgWidth, imgHeight, nlevels, 32, bufferSize, inter, stereo, depth);

	// Decode the features
	(*coderMap)[nAgentId]->decodeImageStereo(img_bitstream, vDecKeypointsLeft, decDescriptorsLeft, vDecKeypointsRight, decDescriptorsRight, vDecVisualWords);

	// Wait previous tracking to finish
	if (mThreadMap[nAgentId].joinable())
		mThreadMap[nAgentId].join();

	// Pass the images to the SLAM system in parallel
	const double tframe = msg->tframe;

	mThreadMap[nAgentId] = std::thread(&trackStereo, SLAM, info, vDecKeypointsLeft, decDescriptorsLeft, vDecVisualWords,
									   vDecKeypointsRight, decDescriptorsRight, tframe, nAgentId, img);
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
	strSettingsFile2 = settings_path + "/KITTI04-12.yaml";
	strSettingsFile1 = settings_path + "/KITTI00-02.yaml";
	SLAM = new CORB_SLAM2::System(voc_path);

	// Call init robot prior to tracking.
	// SLAM->InitAgent(0, strSettingsFile1, CORB_SLAM2::Sensor::STEREO, bUseViewer);
	// SLAM->InitAgent(1, strSettingsFile2, CORB_SLAM2::Sensor::STEREO, bUseViewer);

	// Setup node
	// std::string name = "server";
	// ros::init(argc, argv, name.c_str());
	// ros::NodeHandle nh;

	// Setup signal handler to store trajectory
	signal(SIGINT, signal_handler);

	std::map<int, LBFC2::FeatureCoder *> coderMap;
	// std::map<int, ros::Subscriber> subscriberMap;
	std::map<int, std::thread> threadPool;

	// Setup ros subscriber

	// Wait for incoming robots here...
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

	// std::vector<int> vnRobots = {0, 1};
	// for (size_t n = 0; n < vnRobots.size(); n++)
	// {
	// 	int nAgentId = vnRobots[n];

	// 	std::map<int, ros::Publisher> mFeedbackPub;
	// 	std::map<int, ros::Subscriber> mBitstreamSub;

	// 	std::string sRobotId = std::to_string(nAgentId);
	// 	std::string bitstreamRobot = "/featComp/bitstream" + sRobotId;
	// 	subscriberMap[nAgentId] = nh.subscribe<compression::msg_features>(bitstreamRobot, 10000, boost::bind(callback, _1, SLAM, &coderMap));
	// 	// I can probably emulate this by having one thread per robot that continuously checks for new data on its pipe.  Need to figure out how
	// 	// to read the msgpack data off the pipe properly.
	// }

	// // Lets spin
	// ros::MultiThreadedSpinner spinner(vnRobots.size());
	// spinner.spin();

	// ros::waitForShutdown();

	std::cerr << "Finished" << std::endl;

	signal_handler(0);

	return 0;
}
