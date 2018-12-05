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

int main(int argc, char **argv){

	// RealSense Pipeline encapsulating actual device and sensors
	rs2::pipeline pipe;
	// Start streaming from the pipeline
	pipe.start();

	// Check the starting time
	#ifdef COMPILEDWITHC11
	        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
	#else
	        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
	#endif

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    for(;;){

    	rs2::frameset data = pipe.wait_for_frames();
		rs2::frame color = data.get_color_frame();
		rs2::frame depth = data.get_depth_frame();

		//Find the size of the frame
		int widthColor = color.as<rs2::video_frame>().get_width();
		int heightColor = color.as<rs2::video_frame>().get_height();
		int widthDepth = depth.as<rs2::video_frame>().get_width();
		int heightDepth = depth.as<rs2::video_frame>().get_height();

		// Create OpenCV matrix of size (w,h) from the color data
        Mat rgbFrame(Size(widthColor, heightColor), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
        Mat depthFrame(Size(widthDepth, heightDepth), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);
		
		// Check the current time
		#ifdef COMPILEDWITHC11
        		std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
		#else
        		std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
		#endif

        // Evaluate the timestamp
        double tFrame= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        if(rgbFrame.empty() || depthFrame.empty()){
			cerr << "Failed to read image from the camera" << endl;
			return 1;
		}

		//Send the image to SLAM System
		SLAM.TrackRGBD(rgbFrame, depthFrame, tFrame);

		if ( (char)27 == (char) waitKey(1) ) break;
    }

    // Stop all the threads
	SLAM.Shutdown();

	// Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");   

    return 0;
}
