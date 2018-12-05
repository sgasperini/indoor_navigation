/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include<opencv2/core/core.hpp>
#include<time.h>
#include<librealsense2/rs.hpp>

#include<System.h>

using namespace std;
using namespace cv;

int main(int argc, char **argv) {

	// RealSense Pipeline encapsulating actual device and sensors
	rs2::pipeline pipe;

	//Create a configuration for configuring the pipeline with a non default profile
	rs2::config cfg;

	// Read rectification parameters
    // cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    // if(!fsSettings.isOpened())
    // {
    //     cerr << "ERROR: Wrong path to settings" << endl;
    //     return -1;
    // }

    // cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    // fsSettings["LEFT.K"] >> K_l;
    // fsSettings["RIGHT.K"] >> K_r;

    // fsSettings["LEFT.P"] >> P_l;
    // fsSettings["RIGHT.P"] >> P_r;

    // fsSettings["LEFT.R"] >> R_l;
    // fsSettings["RIGHT.R"] >> R_r;

    // fsSettings["LEFT.D"] >> D_l;
    // fsSettings["RIGHT.D"] >> D_r;

    // int rows_l = fsSettings["LEFT.height"];
    // int cols_l = fsSettings["LEFT.width"];
    // int rows_r = fsSettings["RIGHT.height"];
    // int cols_r = fsSettings["RIGHT.width"];

    // if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
    //         rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
    // {
    //     cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
    //     return -1;
    // }

    // cv::Mat M1l,M2l,M1r,M2r;
    // cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    // cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);

	//Add desired streams to configuration
	cfg.enable_stream(RS2_STREAM_INFRARED, 1);
	cfg.enable_stream(RS2_STREAM_INFRARED, 2);

	// Start streaming from the pipeline
	rs2::pipeline_profile selection = pipe.start(cfg);
	rs2::device selected_device = selection.get_device();
	auto depth_sensor = selected_device.first<rs2::depth_sensor>();

	// Disbale the projection pattern
	if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
	{
	    depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); // Disable emitter
	}
	if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
	{
	    depth_sensor.set_option(RS2_OPTION_LASER_POWER, 0.f); // Disable laser
	}

	// Check the starting time
	#ifdef COMPILEDWITHC11
	        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
	#else
	        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
	#endif

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    for(;;){

		rs2::frameset data = pipe.wait_for_frames();
		rs2::frame ir1 = data.get_infrared_frame(1);
		rs2::frame ir2 = data.get_infrared_frame(2);

		//Find the size of the frame
		int widthIR1 = ir1.as<rs2::video_frame>().get_width();
		int heightIR1 = ir1.as<rs2::video_frame>().get_height();
		int widthIR2 = ir2.as<rs2::video_frame>().get_width();
		int heightIR2 = ir2.as<rs2::video_frame>().get_height();

		// Create OpenCV matrix of size (w,h) from the data
        Mat irFrame1(Size(widthIR1, heightIR1), CV_8UC1, (void*)ir1.get_data(), Mat::AUTO_STEP);
        Mat irFrame2(Size(widthIR2, heightIR2), CV_8UC1, (void*)ir2.get_data(), Mat::AUTO_STEP);

        // Check the current time
		#ifdef COMPILEDWITHC11
        		std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
		#else
        		std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
		#endif

        // Evaluate the timestamp
        double tFrame = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

		if(irFrame1.empty() || irFrame2.empty()){
			cerr << "Failed to read image from the camera" << endl;
			return 1;
		}

		// cv::Mat irLeftRect, irRightRect;
  //       cv::remap(irFrame1,irLeftRect,M1l,M2l,cv::INTER_LINEAR);
  //       cv::remap(irFrame2,irRightRect,M1r,M2r,cv::INTER_LINEAR);

		// Pass the images to the SLAM system
        SLAM.TrackStereo(irFrame1,irFrame2,tFrame);
        // SLAM.TrackStereo(irLeftRect,irRightRect,tFrame);

        if ( (char)27 == (char) waitKey(1) ) break;
	}
	// Stop all the threads
	SLAM.Shutdown();

	// Save camera trajectory
    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

    return 0;
}