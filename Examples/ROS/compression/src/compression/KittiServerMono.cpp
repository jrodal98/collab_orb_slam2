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

#include "System.h"
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include "agent_brain.pb.h"
#include <bitset>
#include "ChosenVocabulary.h"

namespace po = boost::program_options;

// Setup decoder
fbow::Vocabulary voc;
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

void handle_agent(int robot_id)
{
	std::cerr << "Handling robot " << robot_id << std::endl;
	SLAM->InitAgent(robot_id, (robot_id == 0) ? strSettingsFile1 : strSettingsFile2, CORB_SLAM2::Sensor::MONOCULAR, bUseViewer);
	std::string myfifo = "outpipe" + std::to_string(robot_id);
	std::ifstream fifo_file(myfifo);
	std::cerr << "Opened fifo " << myfifo << std::endl;
	std::vector<uchar> data;
	std::vector<uchar> img;
	while (fifo_file && !fifo_file.eof())
	{
		uint64_t size;
		fifo_file.read(static_cast<char*>(static_cast<void*>(&size)), sizeof(size));
		if(!size) break;
		agent_brain::slam_data slam_data;
		data.resize(size);
		fifo_file.read(static_cast<char*>(static_cast<void*>(data.data())), size);
		slam_data.ParseFromArray(data.data(), size); // TODO THIS IS INEFFICIENT, I SHOULD BE ABLE TO PARSE STRAIGHT FROM THE PIPE
		size_t data_size = slam_data.descriptions_size();
        uchar descriptor_data[data_size][32];
        int row = 0;
        for (string description256: slam_data.descriptions()) {
            for (int i = 0; i < description256.size(); i += 8) {
                string description8 = description256.substr(i,8);
                // bitset<8> b(description8);
                // descriptor_data[row][i/8] = ( b.to_ulong() & 0xFF);
                descriptor_data[row][i/8] = std::stoi(description8, nullptr, 2);
            }
            row++;
        }
        cv::Mat descriptors(data_size,32,0,&descriptor_data);
		std::vector<cv::KeyPoint> keypoints;
		for (auto proto_kp: slam_data.keypoints()) {
			// TODO: Extract color information as well
			cv::KeyPoint kp(proto_kp.x(), proto_kp.y(), 7.f);
			keypoints.push_back(kp);
		}
		std::cerr << descriptors.size() << " " << keypoints.size() << std::endl;
		SLAM->TrackMonocular(descriptors, keypoints, robot_id, imgHeight, imgWidth);
	}
}

int main(int argc, char **argv)
{
	po::options_description desc("Allowed options");
	desc.add_options()("help", "produce help message")("voc,v", po::value<std::string>(), "Vocabulary path")("coding,c", po::value<std::string>(), "coding model")("settings,s", po::value<std::string>(), "settings path");

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);

	// Load vocabulary
	std::string voc_path = vm["voc"].as<std::string>();
	std::cerr << "Loading vocabulary from " << voc_path << std::endl;
	voc.readFromFile(voc_path);

	// Load coding model
	std::string stats_path = vm["coding"].as<std::string>();
	std::cerr << "Loading statistics from " << stats_path << std::endl;

	// Setup ORB SLAM - make sure to use the correct settings file
	std::string settings_path = vm["settings"].as<std::string>();
	strSettingsFile1 = settings_path;
	strSettingsFile2 = settings_path;
	cv::FileStorage fsSettings(strSettingsFile1, cv::FileStorage::READ);
	imgHeight = fsSettings["Camera.height"];
	imgWidth = fsSettings["Camera.width"];

	SLAM = new CORB_SLAM2::System(voc_path);

	signal(SIGINT, signal_handler);

	int robot_id = 0;
	std::vector<std::thread> agents;
	for (std::string line; std::getline(std::cin, line);) {
		agents.push_back(std::thread(handle_agent, robot_id++));
		if(line == "QUIT" || line == "QUIT\n") {break;}
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
